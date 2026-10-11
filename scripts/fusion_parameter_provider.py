#!/usr/bin/env python3
"""Dataset-given reference heartbeat; one real process is one explicit session."""
import argparse
import copy
import json
import os
from pathlib import Path
import threading
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    for name in ('reference', 'camera', 'common', 'run-dir', 'session-id'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--attach-time', type=float, required=True)
    args = parser.parse_args()
    if not args.session_id:
        parser.error('A nonempty unique session identity is required')
    root = Path(args.run_dir)
    root.mkdir(parents=True, exist_ok=False)
    from livo2_degeneration.fusion_model import load_reference
    model = load_reference(args.reference, args.camera, args.common)
    model.update(session_id=args.session_id, revision=1)
    (root / 'given_model.json').write_text(json.dumps(model, indent=2))

    import rospy
    from sensor_msgs.msg import CameraInfo
    from livo2_degeneration.msg import FusionModel
    if not hasattr(FusionModel(), 'session_id'):
        raise RuntimeError('Rebuild required: FusionModel.session_id is missing')
    rospy.init_node('fusion_parameter_provider')
    if not rospy.get_param('/fusion_model/enabled', False):
        raise RuntimeError('Fusion parameter provider requires fusion_model/enabled')
    lock = threading.RLock()
    latest = [None]
    stop = threading.Event()
    log = open(root / 'service.jsonl', 'x', buffering=1)

    def event(kind, **fields):
        with lock:
            log.write(json.dumps(dict(event=kind, pid=os.getpid(), session_id=args.session_id,
                                      revision=1, wall=time.time(), monotonic=time.monotonic(),
                                      clock=rospy.Time.now().to_sec(), **fields)) + '\n')

    def camera(info):
        expected = model['camera']
        valid = ((info.width, info.height, info.header.frame_id)
                 == (expected['width'], expected['height'], expected['frame_id']))
        valid = valid and all(np.array_equal(getattr(info, key), expected[key]) for key in ('K', 'D', 'R', 'P'))
        if not valid:
            event('camera_contract_error')
            stop.set()
            rospy.signal_shutdown('CameraInfo differs from fixed official model')
            return
        with lock:
            latest[0] = copy.deepcopy(info)

    sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera, queue_size=4)
    pub = rospy.Publisher('/livo2_degeneration/fusion_model', FusionModel, queue_size=1, latch=False)
    rospy.on_shutdown(stop.set)
    event('started', attach_time=args.attach_time, source=model['source'], reference_sha256=model['reference_sha256'])
    sequence = 0
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            with lock:
                info = copy.deepcopy(latest[0])
            if info is not None and rospy.Time.now().to_sec() >= args.attach_time:
                msg = FusionModel()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'body'
                msg.header.seq = sequence
                msg.session_id = args.session_id
                msg.revision = 1
                msg.source = model['source']
                msg.reference_sha256 = model['reference_sha256']
                msg.camera_info = info
                tr = np.asarray(model['T_IC_applied'])[:3, 3]
                quat = model['quaternion_xyzw']
                msg.imu_from_camera.translation.x, msg.imu_from_camera.translation.y, msg.imu_from_camera.translation.z = tr
                msg.imu_from_camera.rotation.x, msg.imu_from_camera.rotation.y, msg.imu_from_camera.rotation.z, msg.imu_from_camera.rotation.w = quat
                msg.camera_time_offset = model['offset']
                pub.publish(msg)
                event('sent', sequence=sequence, image_stamp=info.header.stamp.to_sec())
                sequence += 1
            stop.wait(.5)  # Real wall schedule, not a simulated-time timer.
    finally:
        event('stopped', sent=sequence)
        log.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
