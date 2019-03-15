#! /usr/bin/env python

import rospy

import actionlib

import refills_msgs.msg

import numpy as np


# OpenCV
import cv2

class VerifyProduct(object):
    # create messages that are used to publish feedback/result
    _feedback = refills_msgs.msg.ProductIdentificationFeedback()
    _result = refills_msgs.msg.ProductIdentificationResult()
    _goal = None

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, refills_msgs.msg.ProductIdentificationAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


    def execute_cb(self, goal):
        self._goal = goal
        # helper variables
        success = True
        print 'Received request: '
        print 'Roi : %r' % goal.roi
        # append the seeds for the fibonacci sequence
        self._feedback.status = 'Counting'
        print 'Gtin: %s' % goal.gtin

        np_arr = np.fromstring(goal.image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #visualize
        cv2.rectangle(image_np,(goal.roi.x_offset,goal.roi.y_offset),(goal.roi.x_offset + goal.roi.width, goal.roi.y_offset + goal.roi.height),(0,255,0),3)
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(0)
        cv2.destroyWindow('cv_img')

        self._result.confidence = 1.0
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('refills_perception')
    print 'Startin test action server'
    server = VerifyProduct('/refills_perception/detect_facing')
    rospy.spin()