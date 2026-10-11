#!/usr/bin/env python3
"""Passive replay observer. Does not publish sensor or model messages."""
import argparse, collections, hashlib, json, math, pathlib, struct, threading, time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from rosgraph_msgs.msg import Log, TopicStatistics, Clock
from inputs import header
import yaml


def main():
    p = argparse.ArgumentParser(); p.add_argument('--run-dir', required=True); p.add_argument('--contract', required=True); p.add_argument('--camera-model', required=True)
    a = p.parse_args(); root = pathlib.Path(a.run_dir)
    expected = json.loads(pathlib.Path(a.contract).read_text())['image_hashes']
    model = yaml.safe_load(pathlib.Path(a.camera_model).read_text())
    output = open(root/'events.jsonl', 'x', buffering=1)
    odom = open(root/'published_odometry.tum', 'x', buffering=1)
    lock = threading.RLock(); started = time.monotonic(); counts = collections.Counter(); last = {}; first = {}; backward = collections.Counter(); duplicates = collections.Counter(); bad = collections.Counter(); empty = collections.Counter(); reasons = collections.Counter(); times = collections.defaultdict(set)
    current_clock = [0.]; max_lag = collections.defaultdict(float); valid_count = [0]
    def event(kind, **v):
        output.write(json.dumps(dict(event=kind, wall=time.monotonic()-started, clock=current_clock[0], **v), allow_nan=False)+'\n')
    def seen(topic, ns):
        counts[topic] += 1
        if topic in last:
            backward[topic] += ns < last[topic]; duplicates[topic] += ns == last[topic]
        else: first[topic] = ns
        last[topic] = ns
        max_lag[topic] = max(max_lag[topic], current_clock[0]-ns/1e9)
    def raw(msg, topic):
        with lock:
            seq, ns, frame, pos = header(msg._buff)
            seen(topic, ns)
            extra = {'points': struct.unpack_from('<I',msg._buff,pos+8)[0]} if topic.endswith('/lidar') else {}
            event('input',topic=topic, stamp_ns=ns, frame=frame, type=msg._connection_header.get('type'), **extra)
    def image(msg, topic):
        with lock:
            ns = msg.header.stamp.to_nsec(); seen(topic, ns)
            if topic == '/rgb_img':
                event('visual_image',stamp_ns=ns,width=msg.width,height=msg.height); return
            depth = 'depth' in topic; key = 'depth' if depth else 'rgb'
            times[key].add(ns)
            if msg.width != 640 or msg.height != 480 or msg.header.frame_id != model['frame_id']: bad['image_contract'] += 1
            if msg.encoding != ('16UC1' if depth else 'rgb8'): bad['encoding'] += 1
            digest = hashlib.sha256(msg.data).hexdigest()
            if expected.get(str(ns),{}).get(key) != digest: bad[key+'_content'] += 1
            event('image',topic=topic,stamp_ns=ns,frame=msg.header.frame_id,width=msg.width,height=msg.height,encoding=msg.encoding,sha256=digest)
    def info(msg):
        with lock:
            ns=msg.header.stamp.to_nsec(); seen('/camera/color/camera_info',ns); times['info'].add(ns)
            if (msg.width,msg.height,msg.header.frame_id)!=(640,480,model['frame_id']) or any(not np.allclose(getattr(msg,k),model[k],rtol=0,atol=1e-10) for k in ('K','D','R','P')): bad['camera_info']+=1
            event('camera_info',stamp_ns=ns,frame=msg.header.frame_id)
    def cloud(msg,topic):
        with lock:
            ns=msg.header.stamp.to_nsec(); seen(topic,ns); n=msg.width*msg.height; empty[topic]+=n==0
            fields={f.name:f for f in msg.fields}
            # Do not silently accept nonfinite geometry. Read strided XYZ fields.
            finite=True
            if n and all(k in fields for k in ('x','y','z')):
                for k in ('x','y','z'):
                    f=fields[k]; dtype={7:'f4',8:'f8'}.get(f.datatype)
                    if dtype:
                        values=np.ndarray((msg.height,msg.width),dtype=('>' if msg.is_bigendian else '<')+dtype,buffer=msg.data,offset=f.offset,strides=(msg.row_step,msg.point_step))
                        finite=finite and bool(np.isfinite(values).all())
            if not finite: bad['nonfinite_cloud']+=1
            event('cloud',topic=topic,stamp_ns=ns,frame=msg.header.frame_id,points=n,finite=finite)
    def pose(msg):
        with lock:
            ns=msg.header.stamp.to_nsec(); seen('/aft_mapped_to_init',ns)
            p,q=msg.pose.pose.position,msg.pose.pose.orientation; values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(x) for x in values): bad['nonfinite_pose']+=1
            else: odom.write('%.9f '%(ns/1e9)+' '.join('%.12g'%x for x in values)+'\n')
            event('odometry',stamp_ns=ns,frame=msg.header.frame_id,child=msg.child_frame_id,pose=values)
    def roslog(msg):
        if msg.level>=Log.WARN:
            with lock:event('rosout',level=msg.level,node=msg.name,message=msg.msg)
    def stats(msg):
        with lock:event('statistics',topic=msg.topic,publisher=msg.node_pub,subscriber=msg.node_sub,delivered=msg.delivered_msgs,dropped=msg.dropped_msgs,start=msg.window_start.to_sec(),stop=msg.window_stop.to_sec())
    def clock(msg):current_clock[0]=msg.clock.to_sec()
    rospy.init_node('m3dgr_observer',disable_signals=False)
    subs=[rospy.Subscriber(t,rospy.AnyMsg,raw,callback_args=t,queue_size=1000,buff_size=8*1024*1024) for t in ('/livox/mid360/lidar','/livox/mid360/imu')]
    subs += [rospy.Subscriber(t,Image,image,callback_args=t,queue_size=20,buff_size=32*1024*1024) for t in ('/camera/color/image_raw','/rgb_img')]
    subs += [rospy.Subscriber(t,PointCloud2,cloud,callback_args=t,queue_size=100,buff_size=32*1024*1024) for t in ('/cloud_registered','/cloud_registered_rgb')]
    subs += [rospy.Subscriber('/aft_mapped_to_init',Odometry,pose,queue_size=1000),rospy.Subscriber('/camera/color/camera_info',CameraInfo,info,queue_size=100),rospy.Subscriber('/rosout_agg',Log,roslog,queue_size=1000),rospy.Subscriber('/statistics',TopicStatistics,stats,queue_size=1000),rospy.Subscriber('/clock',Clock,clock,queue_size=100)]
    def save():
        with lock:
            summary=dict(wall_seconds=time.monotonic()-started,clock=current_clock[0],counts=dict(counts),first_ns=first,last_ns=last,backwards=dict(backward),duplicates=dict(duplicates),max_clock_lag_s=dict(max_lag),errors=dict(bad),empty_clouds=dict(empty),rgb_without_info=len(times['rgb']-times['info']),info_without_rgb=len(times['info']-times['rgb']))
            temp=root/'observer_summary.tmp'; temp.write_text(json.dumps(summary,indent=2)); temp.replace(root/'observer_summary.json')
    while not rospy.is_shutdown():
        time.sleep(1); save()
    save(); output.close(); odom.close()

if __name__=='__main__':main()
