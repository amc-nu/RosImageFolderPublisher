#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('image_folder_publisher')

import sys
import os
from os import listdir
from os.path import isfile, join

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_folder_publisher:
    def __init__(self):
        self.__app_name = "image_folder_publisher"

        self._cv_bridge = CvBridge()

        self._topic_name = rospy.get_param('~topic_name', '/image_raw')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s", self.__app_name, self._topic_name)

        self._image_publisher = rospy.Publisher(self._topic_name, Image, queue_size=1)

        self._rate = rospy.get_param('~publish_rate', 1)
        rospy.loginfo("[%s] (publish_rate) Publish rate set to %s hz", self.__app_name, self._rate)

        self._sort_files = rospy.get_param('~sort_files', True)
        rospy.loginfo("[%s] (sort_files) Sort Files: %r", self.__app_name, self._sort_files)

        self._frame_id = rospy.get_param('~frame_id', 'camera')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s", self.__app_name, self._frame_id)

        self._loop = rospy.get_param('~loop', 1)
        rospy.loginfo("[%s] (loop) Loop  %d time(s) (set it -1 for infinite)", self.__app_name, self._loop)

        self._image_folder = rospy.get_param('~image_folder', '')
        if self._image_folder == '' or not os.path.exists(self._image_folder) or not os.path.isdir(self._image_folder):
            rospy.logfatal("[%s] (image_folder) Invalid Image folder", self.__app_name)
            sys.exit(0)
        rospy.loginfo("[%s] Reading images from %s", self.__app_name, self._image_folder)

    def run(self):
        ros_rate = rospy.Rate(self._rate)

        files_in_dir = [f for f in listdir(self._image_folder) if isfile(join(self._image_folder, f))]
        if self._sort_files:
            files_in_dir.sort()
        try:
            while self._loop != 0:
                for f in files_in_dir:
                    if not rospy.is_shutdown():
                        if isfile(join(self._image_folder, f)):
                            cv_image = cv2.imread(join(self._image_folder, f))
                            if cv_image is not None:
                                ros_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
                                ros_msg.header.frame_id = self._frame_id
                                ros_msg.header.stamp = rospy.Time.now()
                                self._image_publisher.publish(ros_msg)
                                rospy.loginfo("[%s] Published %s", self.__app_name, join(self._image_folder, f))
                            else:
                                rospy.loginfo("[%s] Invalid image file %s", self.__app_name, join(self._image_folder, f))
                            ros_rate.sleep()
                    else:
                        return
                self._loop = self._loop - 1
        except CvBridgeError as e:
            rospy.logerr(e)


def main(args):
    rospy.init_node('image_folder_publisher', anonymous=True)

    image_publisher = image_folder_publisher()
    image_publisher.run()


if __name__ == '__main__':
    main(sys.argv)