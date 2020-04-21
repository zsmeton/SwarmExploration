#!/usr/bin/env python
# license removed for brevity
# sync_loc_on_detect.py
# Synchronize probabilistic locations of multiple robots using detection data

import rospy
from std_msgs.msg import String
from amcl_msgs.msg import *
from amcl.srv import GetParticlecloud
from geometry_msgs.msg import PoseArray
import argparse

# map of robot names to current localization data 
current_loc_data = {

}
# map of current relative position data 
current_relative_pose_data = {

}

def sync_loc_data(robot_n, robot_m):
    """
    Sync robot_n and robot_m's maps using detection data between them
    Note: This function should be called on new detection data 
    """

    # use global variables for current data 
    # TODO: SIMULATE SOME SORT OF COMMS HERE FOR "SHARING" OF DATA


def particlecloud_callback(particlecloud,robot_name):
    """
    Callback function for particlecloud topics
    """
    print(particlecloud)
    print(robot_name)


def run_sync(num_bots):
    rospy.init_node('sync_loc', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # set up callbacks for localization data 
    for i in range(0, num_bots):
        bot_name = "tb3_" + str(i)
        bot_loc_topic = "/" + bot_name + "/particlecloud"
        sub = rospy.Subscriber(bot_loc_topic, PoseArray, particlecloud_callback, callback_args=bot_name) 

    # set up callbacks for relative position data

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        # process command line arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-n", "--num", default=3,
            help="The number of robots in this simulation")
        args = vars(ap.parse_args())

        run_sync(args["num"])
    except rospy.ROSInterruptException:
        pass
