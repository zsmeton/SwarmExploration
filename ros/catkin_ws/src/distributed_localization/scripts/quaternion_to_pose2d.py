#!/usr/bin/env python
# license removed for brevity

# sync_loc_on_detect.py
# Synchronize probabilistic locations of multiple robots using detection data

import rospy
from std_msgs.msg import String
from amcl_msgs.msg import *
from amcl.srv import GetParticlecloud
from geometry_msgs.msg import PoseArray, Pose2D, PoseWithCovarianceStamped
import argparse
import tf.transformations
import math
from scipy.stats import multivariate_normal
import numpy as np
from pose_conversions import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

def run(num_bots,index):
    """
    Run the quaternion to pose2d node
    """

    rospy.init_node('convert', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    publishers = []
    # set up publishers for pose data 
    for i in range(index, num_bots+index):
        bot_name = "tb3_" + str(i)
        topic_name = bot_name + "/pose2d"
        pub = rospy.Publisher(topic_name, Pose2D, queue_size=10)
        publishers.append(pub)

    while not rospy.is_shutdown():

        # get each robots current gazebo pose and convert to pose2d 
        num = 0
        for i in range(index, num_bots+index):
            robot_pose_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            robot_name = "tb3_" + str(i)
            pose = robot_pose_service.call(robot_name, '').pose

            pose2d = pose_to_pose2d_degree(pose)
            print(robot_name)
            print(pose2d)
            publishers[num].publish(pose2d)
            num += 1

        rate.sleep()


if __name__ == '__main__':
    try:
        # process command line arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-n", "--num", default=3,
            help="The number of robots in this simulation")
        ap.add_argument("-i", "--index", default=1,
            help="The starting index for robot topic names") 
        ap.add_argument("-v", "--verbose", default=False,
            help="Run in verbose mode")
        args = vars(ap.parse_args())

#        VERBOSE = args["verbose"]

        run(args["num"],args["index"])
    except rospy.ROSInterruptException:
        pass
