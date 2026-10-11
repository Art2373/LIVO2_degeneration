#!/usr/bin/env python3
"""Passive subscriber: exact image identity and original/zero payload witness.

Does not publish, modify input, or tell the frontend/manager to change state.
Uses the previously verified RGB contract and records every received gate output.
"""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time


def describe(msg, expected, zero_hashes):
    stamp = msg.header.stamp.to_nsec()
    count = len(msg.data)
    digest = hashlib.sha256(msg.data).hexdigest()
    if count not in zero_hashes:
        zero_hashes[count] = hashlib.sha256(bytes(count)).hexdigest()
    zero = digest == zero_hashes[count]
    original = digest == expected.get(str(stamp), {}).get('rgb')
    return dict(stamp_ns=stamp, raw_stamp=msg.header.stamp.to_sec(), seq=msg.header.seq, frame_id=msg.header.frame_id,
                width=msg.width, height=msg.height, encoding=msg.encoding,
                step=msg.step, is_bigendian=msg.is_bigendian, payload_bytes=count,
                sha256=digest, zero_payload=zero, original_payload=original,
                known_input=str(stamp) in expected,
                content='original' if original else 'black' if zero else 'unexpected')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run-dir',required=True);ap.add_argument('--contract',required=True)
    args=ap.parse_args();root=Path(args.run_dir)
    expected=json.loads(Path(args.contract).read_text())['image_hashes']
    import rospy
    from sensor_msgs.msg import Image
    rospy.init_node('frontend_occlusion_pixel_witness')
    stop=threading.Event();lock=threading.Lock();cache={};rows=0;total_ns=0
    log=(root/'pixel_delivery.jsonl').open('x',buffering=1)
    def receive(msg):
        nonlocal rows,total_ns
        start=time.monotonic_ns();wall=time.time();clock=rospy.Time.now().to_sec()
        with lock:
            item=describe(msg,expected,cache)
            spent=time.monotonic_ns()-start;total_ns+=spent;rows+=1
            log.write(json.dumps(dict(wall=wall,clock=clock,elapsed_hash_and_description_ns=spent,**item))+'\n')
    sub=rospy.Subscriber('/frontend_recovery/image_raw',Image,receive,queue_size=100,buff_size=16*1024*1024)
    rospy.on_shutdown(stop.set)
    try:
        while not stop.wait(.5) and not rospy.is_shutdown():pass
    finally:
        sub.unregister()
        with lock:
            log.close()
            with (root/'pixel_delivery_summary.json').open('x') as f:
                json.dump(dict(received=rows,total_processing_wall_ns=total_ns,
                   average_processing_wall_ms=total_ns/rows/1e6 if rows else None,
                   passive=True,no_mapper_state_publication=True),f,indent=2)


if __name__=='__main__':main()
