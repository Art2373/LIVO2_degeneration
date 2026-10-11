#!/usr/bin/env python3
"""Explicit RGB stream fault injector. Never changes mapper state or bag data."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import time


def black_message(original):
    """Pure, reversible output construction; never mutates the subscribed message."""
    if original.encoding not in ('rgb8','bgr8'):
        raise ValueError('black occlusion requires unconverted rgb8/bgr8, received '+original.encoding)
    altered=copy.copy(original)
    altered.data=bytes(len(original.data))
    return altered


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--input', default='/camera/color/image_raw')
    parser.add_argument('--output', default='/frontend_recovery/image_raw')
    args = parser.parse_args()
    if args.input == args.output:
        parser.error('Input and output topics must differ')
    root = Path(args.run_dir)
    root.mkdir(parents=True, exist_ok=False)
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    rospy.init_node('frontend_recovery_image_gate')
    lock = threading.RLock()
    stop = threading.Event()
    log = open(root / 'gate.jsonl', 'x', buffering=1)
    state = dict(mode='pass', command_id='initial_pass', received=0, forwarded=0, dropped=0,
                 altered=0, last_stamp=None, duplicates=0, backward=0, first_after_command=True,
                 failure=None)

    def event(kind, **fields):
        log.write(json.dumps(dict(event=kind, pid=os.getpid(), wall=time.time(),
                                  monotonic=time.monotonic(), clock=rospy.Time.now().to_sec(),
                                  **fields)) + '\n')

    pub = rospy.Publisher(args.output, Image, queue_size=10, latch=False)

    def command(msg):
        try:
            data = json.loads(msg.data)
            if data['mode'] not in ('pass', 'drop', 'black') or not data['event_id']:
                raise ValueError('Only registered pass/drop/black commands are allowed')
        except Exception as error:
            with lock:
                event('invalid_command', error=str(error))
            return
        with lock:
            if data['event_id'] == state['command_id']:
                return
            previous = state['mode']
            state.update(mode=data['mode'], command_id=data['event_id'], first_after_command=True)
            event('command_applied', previous_mode=previous, mode=state['mode'],
                  command_id=state['command_id'], counts=dict(state))

    def image(msg):
        with lock:
            stamp = msg.header.stamp.to_sec()
            state['received'] += 1
            if state['last_stamp'] is not None:
                state['duplicates'] += stamp == state['last_stamp']
                state['backward'] += stamp < state['last_stamp']
            state['last_stamp'] = stamp
            evidence={}
            if state['mode'] == 'pass':
                # Publish the original message object unchanged, including header.
                pub.publish(msg)
                state['forwarded'] += 1
                action='forwarded'
            elif state['mode'] == 'black':
                if msg.encoding not in ('rgb8', 'bgr8'):
                    state['failure']='black occlusion requires unconverted rgb8/bgr8, received '+msg.encoding
                    event('encoding_contract_error', encoding=msg.encoding, message_stamp=stamp,
                          command_id=state['command_id'])
                    stop.set()
                    rospy.signal_shutdown(state['failure'])
                    return
                altered=black_message(msg)
                if state['first_after_command']:
                    evidence=dict(original_sha256=hashlib.sha256(msg.data).hexdigest(),
                                  altered_sha256=hashlib.sha256(altered.data).hexdigest(),
                                  payload_bytes=len(msg.data),all_zero_payload=not any(altered.data),
                                  step=msg.step,is_bigendian=msg.is_bigendian)
                pub.publish(altered)
                state['forwarded']+=1
                state['altered']+=1
                action='altered_published'
            else:
                state['dropped'] += 1
                action='dropped'
            if state['first_after_command']:
                event('first_image_after_command', mode=state['mode'], command_id=state['command_id'],
                      message_stamp=stamp, frame_id=msg.header.frame_id, encoding=msg.encoding,
                      width=msg.width, height=msg.height, action=action,**evidence)
                state['first_after_command'] = False

    image_sub = rospy.Subscriber(args.input, Image, image, queue_size=10, buff_size=16 * 1024 * 1024)
    command_sub = rospy.Subscriber('/frontend_recovery/gate_command', String, command, queue_size=4)
    rospy.on_shutdown(stop.set)
    with lock:
        event('started', input=args.input, output=args.output, policy='registered pass/drop or one all-black occlusion; original input preserved')
    try:
        while not stop.wait(1.) and not rospy.is_shutdown():
            with lock:
                event('counts', **state)
    finally:
        with lock:
            event('stopped', **state)
            (root / 'summary.json').write_text(json.dumps(dict(state), indent=2))
            log.close()
    return 1 if state['failure'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
