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

### GLOBAL VARIABLES ### 
X_VARIANCE = 0.01 # meters^2
Y_VARIANCE = 0.01 # meters^2
THETA_VARIANCE = 25 # degrees^2

# map of robot names to current localization data 
current_loc_data = {

}
# map of robot names to current particle cloud data 
current_particle_cloud_data = {

}
# map of current relative position data 
current_relative_pose_data = {

}


def get_probability_from_detection(pose_n, pose_m, detection_m):
    """
    Get the probability of robot n in pose_n given robot m is pose_m and
    robot n detected robot m with detection_m
    """
    pass


def sync_loc_data(robot_n, robot_m):
    """
    Sync robot_n and robot_m's maps using detection data between them
    Note: This function should be called on new detection data 
    """
    global current_loc_data
    global current_relative_pose_data

    print("Syncing localization data between robot " + robot_n + " and robot " + robot_m)

    # use global variables for current data 
    # TODO: SIMULATE SOME SORT OF COMMS HERE FOR "SHARING" OF DATA
    robot_n_loc = current_loc_data[robot_n]
    robot_m_loc = current_loc_data[robot_m]

    if None == robot_n_loc or None == robot_m_loc:
        rospy.logwarn("Cannot sync particlecloud data for " + robot_n + " and " + robot_m + ": Particle cloud is None")
        return 

    n_detect_m = current_relative_pose_data[robot_n][robot_m]
    if None == n_detect_m:
        rospy.logerr("Sync localization called but no relative pose data. This shouldn't happen.")

    # TODO: Sync localization data! 


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

    print("Robot " + bot_names_tuple[0] + " observed robot " + bot_names_tuple[1])

    # parse bot names out of tuple argument
    observing_bot = bot_names_tuple[0]
    observed_bot = bot_names_tuple[1]

    # update global map
    current_relative_pose_data[observing_bot][observed_bot] = relative_pose 

    # sync localization data of these bots because they detected each other
    sync_loc_data(observing_bot, observed_bot)


def run_sync(num_bots):
    """
    Run the localization synchronization node
    """
    global current_loc_data
    global current_relative_pose_data

    rospy.init_node('sync_loc', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # set up callbacks for localization data 
    for i in range(0, num_bots):
        bot_name = "tb3_" + str(i)
        # create a key for this robot in the global maps 
        current_loc_data[bot_name] = None 
        # equals a map of other robot names with relative pose data
        current_relative_pose_data[bot_name] = {}

        bot_pc_topic = "/" + bot_name + "/particlecloud"
        sub = rospy.Subscriber(bot_pc_topic, PoseArray, particlecloud_callback, callback_args=bot_name) 
        bot_amcl_topic = "/" + bot_name + "/amcl_pose"
        sub = rospy.Subscriber(bot_amcl_topic, PoseWithCovarianceStamped, amcl_pose_callback, callback_args=bot_name) 

    # set up callbacks for relative position data
    for bot_name in current_relative_pose_data.keys():
        for other_bot_name in current_relative_pose_data.keys():
            if other_bot_name != bot_name:
                topic_name = "/" + bot_name + "/" + other_bot_name + "/relative_pose"                
                current_relative_pose_data[bot_name][other_bot_name] = None
                sub = rospy.Subscriber(topic_name, Pose2D, relative_pose_callback, callback_args=(bot_name,other_bot_name))

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
