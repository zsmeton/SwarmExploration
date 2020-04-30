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

### GLOBAL VARIABLES ### 
VERBOSE = False 

X_VARIANCE = 0.1 # meters^2
Y_VARIANCE = 0.1 # meters^2
THETA_VARIANCE = 0.01  # radians^2
X_DOMAIN = 10 # meters
Y_DOMAIN = 10 # meters
THETA_DOMAIN = math.pi # radians

# map of robot names to global pose topic 
global_pose_topic = {

}
# map of robot names to current localization data 
current_loc_data = {

}
# map of robot names to current particle cloud data 
current_particle_cloud_data = {

}
# map of current relative position data 
current_relative_pose_data = {

}


def sync_loc_data(robot_n, robot_m):
    """
    Sync robot_n and robot_m's maps using detection data between them
    Note: This function should be called on new detection data 
    """
    global current_loc_data
    global current_relative_pose_data

#    rospy.loginfo("Syncing localization data between robot " + robot_n + " and robot " + robot_m)

    # use global variables for current data 
    # TODO: SIMULATE SOME SORT OF COMMS HERE FOR "SHARING" OF DATA

    robot_n_loc = current_loc_data[robot_n]
    robot_m_loc = current_loc_data[robot_m]

    # get yaw from orientation quaternion
    PI = 3.1415926

    robot_n_pose = pose_to_pose2d_degree(robot_n_loc.pose.pose)
    robot_m_pose = pose_to_pose2d_degree(robot_m_loc.pose.pose)

    n_detect_m = current_relative_pose_data[robot_n][robot_m]
    if None == n_detect_m:
        rospy.logerr("Sync localization called but no relative pose data. This shouldn't happen.")
        return

    # TODO: Sync localization data! 
    # publish to the estimated global position topic of the observed robot 
    rospy.loginfo("AMCL Location for " + str(robot_n) + " at: " + str(robot_n_pose.x) + "," + str(robot_n_pose.y) + "," + str(robot_n_pose.theta))
    rospy.loginfo("AMCL Location for " + str(robot_m) + " at: " + str(robot_m_pose.x) + "," + str(robot_m_pose.y) + "," + str(robot_m_pose.theta))

    rospy.loginfo("Relative position: " + str(n_detect_m.x) + "," + str(n_detect_m.y)) 

    # calculate new global estimate pose for the observed robot 
    robot_m_estimated_pose = Pose2D()
    robot_m_estimated_pose.x = robot_n_pose.x + math.cos(robot_n_pose.theta*PI / 180.0)*n_detect_m.x + math.sin(robot_n_pose.theta*PI / 180.0)*n_detect_m.y 
    robot_m_estimated_pose.y = robot_n_pose.y - math.sin(robot_n_pose.theta*PI / 180.0)*n_detect_m.x + math.cos(robot_n_pose.theta*PI / 180.0)*n_detect_m.y

    rospy.loginfo("Estimated global pose of robot " + str(robot_m) + ": " + str(robot_m_estimated_pose.x) + "," + str(robot_m_estimated_pose.y) + "," + str(robot_m_estimated_pose.theta))

    global_pose_topic[robot_n][robot_m].publish(robot_m_estimated_pose)


def amcl_pose_callback(amcl_pose,robot_name):
    """
    Callback function for amcl pose topics
    """
    global current_loc_data

    current_loc_data[robot_name] = amcl_pose


def particlecloud_callback(particlecloud,robot_name):
    """
    Callback function for particlecloud topics
    """
    global current_particle_cloud_data

    # update the global current location particle data of this robot 
    current_particle_cloud_data[robot_name] = particlecloud.poses


def relative_pose_callback(relative_pose,bot_names_tuple):
    """
    Callback function for relative pose topics 
    """
    global current_relative_pose_data

    # parse bot names out of tuple argument
    observing_bot = bot_names_tuple[0]
    observed_bot = bot_names_tuple[1]

    # update global map
    current_relative_pose_data[observing_bot][observed_bot] = relative_pose 

    # sync localization data of these bots because they detected each other
    sync_loc_data(observing_bot, observed_bot)


def run_sync(num_bots,index):
    """
    Run the localization synchronization node
    """
    global current_loc_data
    global current_relative_pose_data
    global global_pose_topic

    rospy.init_node('sync_loc', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # set up callbacks for localization data 
    for i in range(index, num_bots+index):
        bot_name = "tb3_" + str(i)
        # create a key for this robot in the global maps 
        current_loc_data[bot_name] = None 
        # equals a map of other robot names with relative pose data
        current_relative_pose_data[bot_name] = {}
        # equals a map of other robot names for publishers 
        global_pose_topic[bot_name] = {}

        bot_pc_topic = bot_name + "/particlecloud"
        sub = rospy.Subscriber(bot_pc_topic, PoseArray, particlecloud_callback, callback_args=bot_name) 
        bot_amcl_topic = bot_name + "/amcl_pose"
        sub = rospy.Subscriber(bot_amcl_topic, PoseWithCovarianceStamped, amcl_pose_callback, callback_args=bot_name) 

    # set up callbacks for relative position data
    for bot_name in current_relative_pose_data.keys():
        for other_bot_name in current_relative_pose_data.keys():
            if other_bot_name != bot_name:
                topic_start = bot_name + "/" + other_bot_name

                global_pose_topic_name = topic_start + "/global_pose_estimate"
                pub = rospy.Publisher(global_pose_topic_name, Pose2D, queue_size=10)
                global_pose_topic[bot_name][other_bot_name] = pub

                relative_pose_topic_name = topic_start + "/relative_pose"                
                current_relative_pose_data[bot_name][other_bot_name] = None
                sub = rospy.Subscriber(relative_pose_topic_name, Pose2D, relative_pose_callback, callback_args=(bot_name,other_bot_name))

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        # process command line arguments
#        ap = argparse.ArgumentParser()
#        ap.add_argument("-n", "--num", default=3,
#            help="The number of robots in this simulation")
#        ap.add_argument("-i", "--index", default=1,
#            help="The starting index for robot topic names") 
#        ap.add_argument("-v", "--verbose", default=False,
#            help="Run in verbose mode")
#        args = vars(ap.parse_args())

#        VERBOSE = args["verbose"]

#        run_sync(args["num"],args["index"])
        run_sync(3, 1)
    except rospy.ROSInterruptException:
        pass
