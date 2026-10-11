#!/usr/bin/env python3
"""Passive wall-clock input/clock/process evidence for the frozen baseline."""
import argparse
import collections
import json
from pathlib import Path
import struct
import threading
import time


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run-dir',required=True)
    args=parser.parse_args();root=Path(args.run_dir)
    import rospy
    from rosgraph_msgs.msg import Clock
    lock=threading.RLock();stop=threading.Event()
    counts=collections.Counter();last={};arrival={};current=[0.]
    samples=open(root/'arrival_samples.jsonl','x',buffering=1)
    clocks=open(root/'clock_arrivals.jsonl','x',buffering=1)
    rospy.init_node('frontend_baseline_monitor')
    def clock(msg):
        with lock:
            current[0]=msg.clock.to_sec();counts['/clock']+=1
            clocks.write(json.dumps(dict(wall=time.time(),monotonic=time.monotonic(),clock=current[0]))+'\n')
    def message(msg,topic):
        # Every registered sensor type starts with std_msgs/Header.
        _,sec,nsec=struct.unpack_from('<III',msg._buff,0)
        with lock:
            counts[topic]+=1;last[topic]=sec+nsec*1.e-9;arrival[topic]=time.time()
    topics=['/livox/mid360/lidar','/livox/mid360/imu','/camera/color/image_raw/compressed',
            '/camera/color/image_raw','/frontend_recovery/image_raw','/camera/color/camera_info']
    subscribers=[rospy.Subscriber('/clock',Clock,clock,queue_size=1000)]
    subscribers += [rospy.Subscriber(t,rospy.AnyMsg,message,callback_args=t,
                       queue_size=1000 if 'imu' in t else 100,buff_size=8*1024*1024) for t in topics]
    rospy.on_shutdown(stop.set)
    def sample():
        process={}
        identity=root/'player_identity.json'
        if identity.exists():
            for label,pid in json.loads(identity.read_text())['pids'].items():
                p=Path('/proc')/str(pid)
                try:
                    status=(p/'status').read_text()
                    process[label]=dict(pid=pid,status={line.split(':',1)[0]:line.split(':',1)[1].strip()
                        for line in status.splitlines() if line.startswith(('Name:','State:','PPid:','Threads:'))})
                except OSError:process[label]=dict(pid=pid,exited=True)
        with lock:
            samples.write(json.dumps(dict(wall=time.time(),monotonic=time.monotonic(),clock=current[0],
                counts=dict(counts),last_stamp=dict(last),last_arrival_wall=dict(arrival),processes=process))+'\n')
    try:
        while not stop.wait(.2) and not rospy.is_shutdown():sample()
    finally:sample();samples.close();clocks.close()


if __name__=='__main__':main()
