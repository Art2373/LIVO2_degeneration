#!/usr/bin/env python3
"""Publish verified full-resolution intrinsics on each decoded RGB timestamp."""
import copy
import sys
import rospy
from sensor_msgs.msg import CameraInfo, Image
from livo2_degeneration.camera_model import validate_model, validate_image


class DatasetCameraInfo:
    def __init__(self):
        self.model = {k: rospy.get_param('~' + k) for k in
                      ('width', 'height', 'frame_id', 'distortion_model', 'K', 'D', 'R', 'P')}
        validate_model(self.model)
        self.last_stamp = -1
        self.failed = False
        self.publisher = rospy.Publisher(rospy.get_param('~camera_info_topic'), CameraInfo, queue_size=50)
        self.subscriber = rospy.Subscriber(rospy.get_param('~color_topic'), Image,
                                          self.receive, queue_size=50, buff_size=2**24)

    def receive(self, image):
        try:
            stamp = image.header.stamp.to_nsec()
            validate_image(self.model, image.width, image.height, image.header.frame_id,
                           stamp, image.encoding)
            if stamp <= self.last_stamp:
                raise ValueError('Color timestamp repeated or moved backwards')
            self.last_stamp = stamp
            info = CameraInfo()
            info.header = copy.deepcopy(image.header)
            for key in ('width', 'height', 'distortion_model', 'K', 'D', 'R', 'P'):
                setattr(info, key, self.model[key])
            self.publisher.publish(info)
        except ValueError as exc:
            self.failed = True
            rospy.logfatal('Dataset camera input contract failed: %s', exc)
            rospy.signal_shutdown(str(exc))


if __name__ == '__main__':
    rospy.init_node('dataset_camera_info')
    node = DatasetCameraInfo()
    rospy.spin()
    sys.exit(1 if node.failed else 0)
