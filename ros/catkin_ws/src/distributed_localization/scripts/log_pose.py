#!/usr/bin/env python
# license removed for brevity

# sync_loc_on_detect.py
# Synchronize probabilistic locations of multiple robots using detection data

import rospy
from std_msgs.msg import String
from amcl_msgs.msg import *
from amcl.srv import GetParticlecloud
from geometry_msgs.msg import PoseArray, Pose2D, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
import argparse
import tf.transformations
import math
import numpy as np
from pose_conversions import *

### GLOBAL VARIABLES ### 
VERBOSE = False 

X_VARIANCE = 0.1 # meters^2
Y_VARIANCE = 0.1 # meters^2
THETA_VARIANCE = 0.01  # radians^2
X_DOMAIN = 10 # meters
Y_DOMAIN = 10 # meters
THETA_DOMAIN = math.pi # radians

# map of robot names to global pose topic 
robot_names = set()

amcl_pose_data = {

}

actual_pose_data = {

}

def amcl_pose_callback(amcl_pose,robot_name):
    """
    Callback function for amcl pose topics
    """
    global amcl_pose_data
    amcl_pose_data[robot_name] = pose2d_to_numpy(pose_to_pose2d_rad(amcl_pose.pose.pose))

def actual_pose_callback(model_states):
    """
    Callback function for actual pose topics
    """
    global actual_pose_data
    for i, model_name in enumerate(model_states.name):
        if model_name in robot_names:
            actual_pose_data[model_name] = pose2d_to_numpy(pose_to_pose2d_rad(model_states.pose[i]))

def save_data(test_num, robot_name):
    """
    Appends the most recent data from amcl and actual to a file
    """
    current_time = rospy.get_time()
    if actual_pose_data[robot_name] is not None and amcl_pose_data[robot_name] is not None:
        with open('testing_data/test-{}-{}-actual.txt'.format(int(test_num), robot_name), 'a+') as actual_fout:
            actual_pose = actual_pose_data[robot_name]
            actual_fout.write('{} {} {} {}\n'.format(current_time, actual_pose[0], actual_pose[1], actual_pose[2]))
        
        with open('testing_data/test-{}-{}-amcl.txt'.format(int(test_num), robot_name), 'a+') as amcl_fout:
            amcl_pose = amcl_pose_data[robot_name]
            amcl_fout.write('{} {} {} {}\n'.format(current_time, amcl_pose[0], amcl_pose[1], amcl_pose[2]))

def run_sync(num_bots,index, test_num):
    """
    Run the localization synchronization node
    """
    global amcl_pose_data
    global actual_pose_data
    global robot_names

    rospy.init_node('log_pose', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # set up callbacks for localization data 
    for i in range(index, num_bots+index):
        bot_name = "tb3_" + str(i)
        robot_names.add(bot_name)
        # create a key for this robot in the global maps 
        amcl_pose_data[bot_name] = None
        actual_pose_data[bot_name] = None 

        bot_amcl_topic = bot_name + "/amcl_pose"
        sub = rospy.Subscriber(bot_amcl_topic, PoseWithCovarianceStamped, amcl_pose_callback, callback_args=bot_name)

    # Set up callbacks for gazebo data
    gazebo_topic = "/gazebo/model_states"
    sub = rospy.Subscriber(gazebo_topic, ModelStates, actual_pose_callback) 

    while not rospy.is_shutdown():
        for robot in robot_names:
            save_data(test_num=test_num, robot_name=robot)
        rate.sleep()


if __name__ == '__main__':
    try:
        # process command line arguments
        parser = argparse.ArgumentParser(description='Logs the estimated and actual poses of the robots')
        parser.add_argument('-t', '--test_id', type=int, help='the testing number', default=1)
        args = parser.parse_args()

        run_sync(3, 1, args.test_id)
    except rospy.ROSInterruptException:
        pass
