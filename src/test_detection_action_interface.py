#!/usr/bin/env python

from os import sys, listdir
import os
from os.path import isfile, join

from pprint import pprint
import json

# numpy and scipy
import numpy as np

# OpenCV
import cv2

import actionlib
import refills_msgs.msg

import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import RegionOfInterest

import argparse

def readFiles(path):
    list_of_file_pairs = []
    for f in listdir(path):
        filename, file_extension = os.path.splitext(f)
        if file_extension == ".png":
            img_file = join(path,f)
            json_file = join(path,filename[:-4]+'_meta.json')
            list_of_file_pairs.append((img_file,json_file))
    return list_of_file_pairs

def call_action_client(file_pairs,  topic_name='/refills_perception/detect_facing'):

    client = actionlib.SimpleActionClient(topic_name, refills_msgs.msg.ProductIdentificationAction)
    print 'Waiting for action [%s] to come up.' % (topic_name)
    client.wait_for_server()
    print ('Server online. Continuing')
    for img_file, meta in file_pairs:
        print 'Calling object detection action for: %s' % img_file
        img = cv2.imread(img_file,cv2.IMREAD_COLOR)
        print 'Image is of size: %d x %d ; Number of channels: %d'% img.shape
        print 'Opening json file: %s ' % meta

        with open(meta) as f:
            json_data = json.load(f)
        pprint(json_data)

        goal = refills_msgs.msg.ProductIdentificationGoal()

        goal.image = CompressedImage()
        goal.image.header.stamp = rospy.Time.now()
        goal.image.format = "jpeg"
        goal.image.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

        goal.gtin = str(json_data['dan'])

        goal.roi = RegionOfInterest()
        goal.roi.width = json_data['rect']['w']
        goal.roi.height = json_data['rect']['h']

        goal.roi.x_offset= json_data['rect']['x']
        goal.roi.y_offset = json_data['rect']['y']

        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        result = client.get_result()
        print result# A FibonacciResult


if __name__ == '__main__':

    parser = argparse.ArgumentParser("Test the detection action client on recorded images")
    parser.add_argument('-p','--path',help='path to folder holding images and json files')

    args = parser.parse_args()
    if args.path == None:
        parser.print_help()
        exit()

    rospy.init_node('test_verification_client')

    file_pairs = readFiles(args.path)
    call_action_client(file_pairs)
