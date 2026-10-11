#!/usr/bin/env python3
"""Four full-sequence given-reference frontend regressions on one frozen binary.

Reuse the established gate, service, pixel witness, isolated executable and serial
shutdown protections. Natural EOF only; this runner never rebuilds production.
"""
import argparse
import hashlib
import html
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
import uuid
import xmlrpc.client


def retained_finite_vio(row):
    if row.get('event')!='vio' or row.get('vio_steps',0)<=row.get('vio_reverted',0) or row.get('vio_pixels',0)<=0:
        return False
    if 'vio_retained_finite' in row:
        return bool(row['vio_retained_finite'])
    # Startup-only legacy-schema evidence; new experiments provide the explicit flag.
    values=list(row.get('position',[]))+list(row.get('quaternion_xyzw',[]))
    return row.get('vio_nonfinite',0)==0 and len(values)==7 and all(math.isfinite(float(value)) for value in values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', required=True, help='Case key in the frozen plan')
    parser.add_argument('--plan', required=True)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    plan_path = Path(args.plan).resolve()
    plan = json.loads(plan_path.read_text())
    if os.environ.get('LIVO2_REGRESSION_ENV') != plan['environment']:
        environment = dict(os.environ, LIVO2_REGRESSION_ENV=plan['environment'])
        return subprocess.call([plan['environment'], sys.executable, str(Path(__file__).resolve())] + sys.argv[1:], env=environment)
    port=int(plan['master_port'])
    os.environ['ROS_MASTER_URI']='http://localhost:%d'%port
    os.environ['ROS_HOSTNAME']='localhost'
    if args.case not in plan['cases']:
        parser.error('Case is not registered in the selected plan')
    registered=plan['cases'][args.case]
    args.feedback=registered['feedback'];args.policy=registered['policy'];assert args.policy=='managed'
    assert args.feedback=='measurement_commit_v1'
    args.schedule=registered['schedule'];assert args.schedule=='burst'
    args.fault='none' if not registered['events'] else registered['events'][0]['kind']
    if registered['events'] and all(e['kind']=='rgb' for e in registered['events']):
        args.fault='rgb'
    if registered['events'] and all(e['kind']=='occlusion' for e in registered['events']):
        args.fault='occlusion'
    if registered['events'] and all(e['kind']=='service' for e in registered['events']):
        args.fault='service'
    if any(e['kind'] not in ('rgb','occlusion','service') for e in registered['events']):
        parser.error('Only registered RGB, black image or model service events are permitted')
    assert plan['full_bag'] is True
    task=dict(plan, duration_sensor_s=plan['bag_metadata']['duration'])
    duration,rate=task['duration_sensor_s'],task['rate']
    assert rate==.5
    for previous in plan['run_order'][:plan['run_order'].index(args.case)]:
        manifest_path=Path(plan['root'])/'runs'/previous/'manifest.json'
        if not manifest_path.exists() or not json.loads(manifest_path.read_text()).get('completed'):
            parser.error('Previous registered case is not complete; preserve failure and stop: '+previous)
    root=Path(plan['root'])/'runs'/args.case
    freeze=json.loads(Path(plan['freeze']).read_text())
    sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    tool_freeze=Path(plan['root'])/'audit/run_tools_freeze.json'
    if tool_freeze.exists():
        for path,value in json.loads(tool_freeze.read_text())['sha256'].items():
            assert sha(path)==value,'Frozen run orchestration changed: '+path
    for path,value in freeze['current_sources'].items():
        assert sha(path)==value,'Frozen production/source changed: '+path
    for path,value in freeze['binaries'].items():
        assert sha(path)==value,'Frozen binary changed: '+path
    for path,value in freeze['configuration'].items():
        assert sha(path)==value,'Frozen configuration changed: '+path
    bag = Path(plan['bag']).resolve()
    ws = Path(plan['workspace'])
    socket.setdefaulttimeout(5)
    if os.environ.get('ROS_MASTER_URI') != 'http://localhost:%d'%port:
        parser.error('Use the registered isolated env.sh')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            parser.error('An existing master is running; do not stop other work')

    import rosbag
    import rosgraph
    import rospy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from inputs import TOPICS, inspect_bag
    preflight = inspect_bag(bag, 'B')
    with rosbag.Bag(str(bag), 'r', skip_index=True) as handle:
        first = handle.get_start_time()
        original_end = handle.get_end_time()
    if (first,original_end)!=(plan['bag_metadata']['start'],plan['bag_metadata']['end']):
        raise RuntimeError('Bag interval differs from frozen registration')
    observer_contract = Path(plan['observer_contract']).resolve()
    if not json.loads(observer_contract.read_text()).get('image_hashes'):
        raise RuntimeError('Observer contract has no registered decoded image hashes')
    root.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    started = time.time()
    began = time.monotonic()
    children, logs = [], []
    current_clock = [0.]
    actual_player_pid=None
    active_episode=None
    next_event=0
    completed_episodes=[]
    retained_at_observation_start=0
    latest = {}
    retained_seen=0
    tail = None
    tail_buffer = ''
    master = rosgraph.Master('/frontend_recovery_runner')
    events = open(root / 'runner_events.jsonl', 'x', buffering=1)
    complete = fault_injected = restored = False
    service = None
    service_sessions = []

    def event(kind, **fields):
        events.write(json.dumps(dict(event=kind, wall=time.time(), monotonic=time.monotonic(),
                                     clock=current_clock[0], **fields)) + '\n')

    def spawn(name, command):
        log = open(root / (name + '.log'), 'x')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))
        event('process_started', name=name, pid=process.pid, command=command)
        return process

    def alive(process, name):
        if process.poll() is not None:
            raise RuntimeError(name + ' exited ' + str(process.returncode))

    def until(test, seconds, label):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if test():
                return
            time.sleep(.1)
        raise RuntimeError('Timeout: ' + label)

    def master_up():
        try:
            master.getPid()
            return True
        except Exception:
            return False

    def nodes():
        return {node for entries in master.getSystemState() for _, names in entries for node in names}

    def stop(process, sig=signal.SIGINT):
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def proc_identity(pid):
        p=Path('/proc')/str(pid)
        return dict(pid=pid,exe=str((p/'exe').resolve()),
            cmdline=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(),
            status=(p/'status').read_text(),stat=(p/'stat').read_text(),pgid=os.getpgid(pid),sid=os.getsid(pid))

    def bind_player(wrapper):
        nonlocal actual_player_pid
        node=xmlrpc.client.ServerProxy(master.lookupNode('/m3dgr_player'))
        actual_player_pid=int(node.getPid('/frontend_regression_runner')[2])
        actual=proc_identity(actual_player_pid)
        if actual['exe']!='/opt/ros/noetic/lib/rosbag/play' or actual['pgid']!=wrapper.pid:
            raise RuntimeError('ROS player PID/executable/group mismatch: '+str(actual))
        ids=dict(pids=dict(wrapper=wrapper.pid,actual_player=actual_player_pid,service=service.pid),
                 actual_player=actual,wrapper=proc_identity(wrapper.pid),service=proc_identity(service.pid),
                 clock=current_clock[0],wall=time.time())
        temporary=root/'player_identity.tmp'
        temporary.write_text(json.dumps(ids,indent=2))
        temporary.replace(root/'player_identity.json')
        (root/'player_identity_initial.json').write_text(json.dumps(ids,indent=2))
        event('actual_player_identified',identity=ids)

    def snapshot(label):
        data = dict(wall=time.time(), clock=current_clock[0], system_state=master.getSystemState(), nodes={})
        for node_name in sorted(nodes()):
            try:
                node = xmlrpc.client.ServerProxy(master.lookupNode(node_name))
                pid = node.getPid('/frontend_recovery_runner')[2]
                proc = Path('/proc') / str(pid)
                data['nodes'][node_name] = dict(pid=pid, status=(proc/'status').read_text(), stat=(proc/'stat').read_text(),
                    cmdline=(proc/'cmdline').read_bytes().replace(b'\0', b' ').decode(),
                    libraries=sorted({line.split()[-1] for line in (proc/'maps').read_text().splitlines()
                                      if 'libopencv' in line or 'cv_bridge' in line or 'cv2.' in line}))
            except Exception as error:
                data['nodes'][node_name] = dict(error=str(error))
        (root / (label + '.json')).write_text(json.dumps(data, indent=2))
        subprocess.run(['rosparam', 'dump', str(root / (label + '_params.yaml'))], check=True, timeout=15)

    def start_service(index):
        session_id = root.name + '_s%02d_' % index + uuid.uuid4().hex
        session_dir = root / 'services' / ('session_%02d' % index)
        command = [sys.executable, str(repo/'scripts/fusion_parameter_provider.py'), '--run-dir', str(session_dir),
                   '--reference', str(repo/'config/m3dgr/reference.yaml'), '--camera', str(repo/'config/m3dgr/camera_info.yaml'),
                   '--common', str(repo/'config/m3dgr/common.yaml'), '--session-id', session_id,
                   '--attach-time', str(first + task['attach_after_s'])]
        process = spawn('reference_service_%02d' % index, command)
        service_sessions.append(dict(session_id=session_id, revision=1, directory=str(session_dir), pid=process.pid))
        return process

    # Same decode/gate path for every policy. Only the mapper subscription is remapped.
    launch_path = root / 'run.launch'
    esc = lambda value: html.escape(str(value), quote=True)
    launch_path.write_text('''<launch>
  <param name="use_sim_time" value="true"/>
  <rosparam file="$(find livo2_degeneration)/config/m3dgr/common.yaml"/>
  <param name="fusion_model/enabled" value="true"/>
  <param name="dynamic_visual/management_enabled" value="%s"/>
  <param name="dynamic_visual/schedule_trace_enabled" value="true"/>
  <param name="dynamic_visual/feedback_mode" value="%s"/>
  <param name="dynamic_visual/recovery_schedule" value="%s"/>
  <param name="dynamic_visual/trace_path" value="%s"/>
  <param name="runtime/output_directory" value="%s"/>
  <param name="evo/seq_name" value="%s"/>
  <param name="m3dgr/run_id" value="%s"/>
  <param name="m3dgr/purpose" value="given_reference_full_sequence_frontend_regression"/>
  <include file="$(find livo2_degeneration)/launch/m3dgr_input.launch"/>
  <node pkg="livo2_degeneration" type="fusion_mapping" name="fusion_mapping" output="screen" required="true">
    <remap from="/camera/color/image_raw" to="/frontend_recovery/image_raw"/>
  </node>
</launch>
''' % ('true' if args.policy == 'managed' else 'false', esc(args.feedback), esc(args.schedule), esc(root/'frontend.jsonl'), esc(root/'native_output'), esc(root.name), esc(root.name)))
    diagnostic_binary = Path(plan['diagnostic_binary'])
    # Policy thresholds are plan data, not per-run tuning.
    policy_parameters = task['policy_parameters']
    launch = ['roslaunch', str(launch_path)]
    play = ['rosbag', 'play', '--clock', '--delay', '3', '--wait-for-subscribers', '--rate', str(rate),
            '__name:=m3dgr_player', str(bag), '--topics'] + TOPICS['B']
    binaries = [diagnostic_binary] + [ws/('devel/lib/libdegeneration_'+name+'.so') for name in ['laser_mapping','vio','lio','imu_proc','pre']]
    sources = [Path(p) for p in freeze['current_sources']]
    manifest = dict(policy=args.policy, fault=args.fault, feedback_mode=args.feedback, recovery_schedule=args.schedule, scope='given_reference_full_sequence_frontend_regression_fixed_feedback_fixed_schedule',
        plan=str(plan_path), plan_sha256=hashlib.sha256(plan_path.read_bytes()).hexdigest(), bag=str(bag),
        observer_contract=str(observer_contract), observer_contract_sha256=sha(observer_contract),
        start_stamp=first, original_bag_end=original_end, duration_sensor_s=duration, rate=rate,
        attach_time=first+task['attach_after_s'], case=args.case, events=registered['events'],full_bag=True,termination_policy='natural_player_EOF_then_3_wall_seconds',diagnostic_binary=plan['diagnostic_binary'],diagnostic_build_manifest=plan['diagnostic_build_manifest'],
        launch=launch, play=play, started_epoch=started, policy_parameters=policy_parameters,
        binaries={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in binaries},
        sources={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources if path.is_file()},
        python=sys.executable, pythonpath=os.environ.get('PYTHONPATH'), ld_library_path=os.environ.get('LD_LIBRARY_PATH'))
    (root/'manifest.json').write_text(json.dumps(manifest, indent=2))
    (root/'input_preflight.json').write_text(json.dumps(preflight, indent=2))
    try:
        core = spawn('master', ['roscore','-p',str(port)])
        until(master_up, 20, 'master')
        master.setParam('/use_sim_time', True)
        master.setParam('/enable_statistics', True)
        for key, value in policy_parameters.items():
            master.setParam('/dynamic_visual/' + key, value)
        rospy.init_node('frontend_recovery_runner', disable_signals=True)
        def clock_received(msg):
            value=msg.clock.to_sec()
            current_clock[0]=value
        clock_sub = rospy.Subscriber('/clock', Clock, clock_received, queue_size=1000)
        gate_command = rospy.Publisher('/frontend_recovery/gate_command', String, queue_size=1, latch=True)
        gate = spawn('image_gate', [sys.executable, str(repo/'tools/regression/image_gate.py'), '--run-dir', str(root/'image_gate')])
        mapper = spawn('nodes', launch)
        def processing_up():
            alive(mapper, 'roslaunch'); alive(gate, 'image_gate')
            return {'/fusion_mapping','/m3dgr_rgb_decode','/dataset_camera_info','/frontend_recovery_image_gate'} <= nodes()
        until(processing_up, 40, 'processing nodes')
        observer = spawn('observer', [sys.executable, str(repo/'tools/regression/observer.py'), '--run-dir', str(root),
            '--contract',str(observer_contract),'--camera-model',str(repo/'config/m3dgr/camera_info.yaml')])
        monitor=spawn('monitor',[sys.executable,str(repo/'tools/regression/monitor.py'),'--run-dir',str(root)])
        pixel_witness=spawn('pixel_witness',[sys.executable,str(repo/'tools/regression/pixel_witness.py'),
            '--run-dir',str(root),'--contract',str(observer_contract)])
        service = start_service(1)
        def evidence_up():
            alive(service, 'reference_service'); alive(observer, 'observer'); alive(pixel_witness,'pixel_witness')
            return {'/m3dgr_observer','/fusion_parameter_provider','/frontend_baseline_monitor','/frontend_occlusion_pixel_witness'} <= nodes() and gate_command.get_num_connections() > 0
        until(evidence_up, 20, 'evidence and source nodes')
        gate_command.publish(String(data=json.dumps(dict(mode='pass', event_id='registered_initial_pass'))))
        snapshot('startup')
        mapper_pid=json.loads((root/'startup.json').read_text())['nodes']['/fusion_mapping']['pid']
        loaded=(Path('/proc')/str(mapper_pid)/'maps').read_text()
        (root/'diagnostic_loaded_maps.txt').write_text(loaded)
        for name in ['laser_mapping','vio','lio','imu_proc','pre']:
            expected=str(ws/('devel/lib/libdegeneration_'+name+'.so'))
            assert expected in loaded, 'Independent core library not loaded: '+expected
        core_maps={line.split()[-1] for line in loaded.splitlines() if any('/lib'+name+'.so' in line for name in ('laser_mapping','vio','lio','imu_proc','pre'))}
        assert not core_maps, 'Unexpected unnamespaced core libraries loaded: '+str(core_maps)
        assert Path('/proc/%s/exe'%mapper_pid).resolve()==diagnostic_binary.resolve(), 'Unexpected mapper executable'
        assert master.getParam('/dynamic_visual/feedback_mode')==args.feedback, 'Loaded feedback switch differs'
        assert master.getParam('/dynamic_visual/recovery_schedule')==args.schedule, 'Loaded recovery schedule differs'
        assert master.getParam('/dynamic_visual/management_enabled') is True, 'Management policy differs'
        player = spawn('player', play)
        playback_start = time.monotonic()
        until(lambda: '/m3dgr_player' in nodes(),15,'actual player node')
        bind_player(player)
        deadline = playback_start + duration / rate + 90
        while player.poll() is None:
            alive(mapper, 'roslaunch'); alive(observer, 'observer'); alive(gate, 'image_gate')
            if not (active_episode and active_episode['kind']=='service'):
                alive(service,'reference_service')
            alive(monitor,'monitor');alive(pixel_witness,'pixel_witness')
            if time.monotonic() > deadline:
                raise RuntimeError('Player exceeded registered bounded duration')
            if tail is None and (root/'frontend.jsonl').exists():
                tail = open(root/'frontend.jsonl')
            if tail is not None:
                tail_buffer += tail.read()
                while '\n' in tail_buffer:
                    line, tail_buffer = tail_buffer.split('\n', 1)
                    if line:
                        latest = json.loads(line)
                        retained_seen+=retained_finite_vio(latest)
            if active_episode is None and next_event<len(registered['events']):
                candidate=registered['events'][next_event]
                if current_clock[0]>=first+candidate['start_after_s']:
                    exposed=retained_seen>retained_at_observation_start
                    if not exposed:
                        event('fault_precondition_unexposed',episode_id=candidate['episode_id'],
                              retained_finite_vio_seen=retained_seen,
                              consequence='Inject at unchanged registered time; recovery unexposed, continue normal execution')
                    active_episode=dict(candidate)
                    eid=candidate['episode_id']
                    event('fault_precondition_met' if exposed else 'fault_precondition_record',episode_id=eid,retained_finite_vio_seen=retained_seen,
                          retained_since_previous_restore=retained_seen-retained_at_observation_start)
                    if candidate['kind']=='rgb':
                        event('rgb_drop_command',episode_id=eid,frontend=latest)
                        gate_command.publish(String(data=json.dumps(dict(mode='drop',event_id=eid+'_drop'))))
                    elif candidate['kind']=='occlusion':
                        event('occlusion_black_command',episode_id=eid,frontend=latest)
                        gate_command.publish(String(data=json.dumps(dict(mode='black',event_id=eid+'_black'))))
                    elif candidate['kind']=='service':
                        service_rows=[json.loads(line) for line in (Path(service_sessions[-1]['directory'])/'service.jsonl').read_text().splitlines() if line.strip()]
                        last_sent=next((row for row in reversed(service_rows) if row['event']=='sent'),None)
                        event('service_stop_request',episode_id=eid,pid=service.pid,signal='SIGTERM',process=proc_identity(service.pid),last_sent=last_sent,frontend=latest)
                        stop(service,signal.SIGTERM)
                        stopped_rows=[json.loads(line) for line in (Path(service_sessions[-1]['directory'])/'service.jsonl').read_text().splitlines() if line.strip()]
                        last_actual_sent=next((row for row in reversed(stopped_rows) if row['event']=='sent'),None)
                        event('service_exited',episode_id=eid,pid=service.pid,returncode=service.returncode,session=service_sessions[-1],last_actual_sent=last_actual_sent)
                        if service.returncode!=0:raise RuntimeError('Reference service did not exit normally on registered stop')
                    fault_injected=True
            if active_episode:
                e=active_episode;eid=e['episode_id']
                if e['kind'] in ('rgb','occlusion') and current_clock[0]>=first+e['restore_after_s']:
                    event('occlusion_restore_command' if e['kind']=='occlusion' else 'rgb_restore_command',episode_id=eid,frontend=latest)
                    gate_command.publish(String(data=json.dumps(dict(mode='pass',event_id=eid+'_restore'))))
                    completed_episodes.append(eid);next_event+=1;active_episode=None
                    retained_at_observation_start=retained_seen
                elif e['kind']=='service' and current_clock[0]>=first+e['restore_after_s']:
                    event('service_restart_request',episode_id=eid,previous_session=service_sessions[-1],frontend=latest)
                    service=start_service(2)
                    event('service_restart',episode_id=eid,pid=service.pid,session=service_sessions[-1])
                    def restarted_service_ready():
                        alive(service,'restarted_reference_service')
                        if '/fusion_parameter_provider' not in nodes():return False
                        try:
                            node=xmlrpc.client.ServerProxy(master.lookupNode('/fusion_parameter_provider'))
                            return int(node.getPid('/frontend_regression_runner')[2])==service.pid
                        except Exception:return False
                    until(restarted_service_ready,15,'new reference-service process registered')
                    restarted_identity=proc_identity(service.pid)
                    event('service_restart_registered',episode_id=eid,pid=service.pid,session=service_sessions[-1],process=restarted_identity)
                    current_identity=json.loads((root/'player_identity.json').read_text())
                    current_identity['pids']['service']=service.pid
                    current_identity.update(service=restarted_identity,service_session=service_sessions[-1],clock=current_clock[0],wall=time.time())
                    (root/'player_identity.tmp').write_text(json.dumps(current_identity,indent=2))
                    (root/'player_identity.tmp').replace(root/'player_identity.json')
                    event('monitor_service_identity_updated',episode_id=eid,pid=service.pid,initial_identity=str(root/'player_identity_initial.json'))
                    completed_episodes.append(eid);next_event+=1;active_episode=None
                    retained_at_observation_start=retained_seen
            restored=bool(registered['events']) and len(completed_episodes)==len(registered['events'])
            time.sleep(.05)
        manifest['player_returncode'] = player.returncode
        manifest['playback_wall_s'] = time.monotonic() - playback_start
        if player.returncode:
            raise RuntimeError('Bag player failed')
        event('playback_eof', natural=True, player_pid=actual_player_pid, player_returncode=player.returncode, bag_end=original_end, frontend=latest)
        if registered['events'] and not restored:
            raise RuntimeError('Not all registered faults and restorations completed before natural EOF')
        until(lambda: time.monotonic() >= playback_start + manifest['playback_wall_s'] + plan['eof_observe_wall_s'], 5., 'wall-only EOF observation')
        snapshot('finished')
        complete = True
    except Exception as error:
        manifest['failure'] = str(error)
        event('failure', reason=str(error))
        print(str(error), flush=True)
    finally:
        event('experiment_shutdown')
        # Normal shutdown is explicitly outside the registered fault interval.
        for label, process in children:
            if label == 'nodes':
                stop(process)
        for label, process in reversed(children):
            stop(process)
        if tail is not None:
            tail.close()
        try:
            fronts=[json.loads(line) for line in (root/'frontend.jsonl').read_text().splitlines()]
            vio_count=sum(v['event']=='vio' for v in fronts)
            assert fronts and vio_count>0, 'No frontend VIO trace'
            # Natural player completion and full required-stream counts, not a
            # synthetic endpoint or a partial timestamp-range approximation.
            assert manifest.get('player_returncode')==0, 'Player did not exit naturally with status zero'
            assert any(line.strip()=='Done.' for line in (root/'player.log').read_text(errors='replace').splitlines()), 'Native player Done marker missing'
            arrivals=[json.loads(line) for line in (root/'arrival_samples.jsonl').read_text().splitlines() if line.strip()]
            assert arrivals, 'No required-input count monitor'
            input_counts=[]
            for topic in preflight['required_topics']:
                expected=sum(row['count'] for row in preflight['connections'] if row['topic']==topic)
                observed=arrivals[-1]['counts'].get(topic,0)
                input_counts.append(dict(topic=topic,expected_full_bag=expected,observed=observed,equal=expected==observed))
            assert input_counts and all(row['expected_full_bag']>0 and row['equal'] for row in input_counts), 'Required input stream counts incomplete: '+str(input_counts)
            pixels=[json.loads(line) for line in (root/'pixel_delivery.jsonl').read_text().splitlines()]
            assert pixels and all(v['known_input'] and v['content'] in ['original','black'] for v in pixels), 'Unexpected delivered pixels'
            assert all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(pixels,pixels[1:])), 'Duplicate or backward delivered image'
            blacks=[v for v in pixels if v['content']=='black']
            gate_rows=[json.loads(line) for line in (root/'image_gate/gate.jsonl').read_text().splitlines() if line.strip()]
            if args.fault=='occlusion':
                assert blacks and all(first+42.9 <= v['clock'] <= first+51.2 for v in blacks), 'Black delivery interval missing/incorrect'
                assert any(v['content']=='original' and v['clock']>blacks[-1]['clock'] for v in pixels), 'No restored original image'
            else:
                assert not blacks, 'Unexpected black image outside registered black case'
            if args.fault=='rgb':
                eid=registered['events'][0]['episode_id']
                dropped=next(row for row in gate_rows if row['event']=='first_image_after_command' and row['command_id']==eid+'_drop')
                resumed=next(row for row in gate_rows if row['event']=='first_image_after_command' and row['command_id']==eid+'_restore')
                assert dropped['action']=='dropped' and resumed['action']=='forwarded'
                assert not any(dropped['message_stamp']<=v['raw_stamp']<resumed['message_stamp'] for v in pixels), 'Delivered image inside actual RGB drop interval'
                assert any(v['raw_stamp']>=resumed['message_stamp'] for v in pixels), 'No restored original RGB delivery'
            if args.fault=='service':
                assert len(service_sessions)==2 and service_sessions[0]['session_id']!=service_sessions[1]['session_id'], 'Missing distinct restarted service session'
                for session in service_sessions:
                    recorded=[json.loads(line) for line in (Path(session['directory'])/'service.jsonl').read_text().splitlines() if line.strip()]
                    assert any(row['event']=='sent' for row in recorded), 'Reference session never sent an actual message'
                first_model=json.loads((Path(service_sessions[0]['directory'])/'given_model.json').read_text());second_model=json.loads((Path(service_sessions[1]['directory'])/'given_model.json').read_text())
                first_model.pop('session_id');second_model.pop('session_id');assert first_model==second_model,'Restarted reference parameters changed'
            assert all(process.returncode==0 for _,process in children), 'Nonzero child exit: '+str({name:process.returncode for name,process in children})
            manifest['records_verified']=dict(calls=vio_count,frontend_rows=len(fronts),delivered_images=len(pixels),black_images=len(blacks),natural_eof_verified=True,required_input_counts=input_counts,final_clock=current_clock[0],bag_end=original_end,clock_to_bag_end_s=original_end-current_clock[0])
        except Exception as error:
            complete=False;manifest['failure']='Development record verification: '+str(error)
            event('failure',reason=manifest['failure'])
        manifest.update(completed=complete, fault_injected=fault_injected, restored=restored,
                        wall_s=time.monotonic()-began, service_sessions=service_sessions,completed_episodes=completed_episodes,
                        children={name:process.returncode for name,process in children})
        (root/'manifest.json').write_text(json.dumps(manifest, indent=2))
        events.close()
        for log in logs:
            log.close()
    print(json.dumps(dict(policy=args.policy, fault=args.fault, completed=complete,
                          fault_injected=fault_injected, restored=restored)))
    return 0 if complete else 1


if __name__ == '__main__':
    raise SystemExit(main())
