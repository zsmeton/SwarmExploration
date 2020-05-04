#!/usr/bin/env python
# license removed for brevity

# sync_loc_on_detect.py
# Synchronize probabilistic locations of multiple robots using detection data

import rospy
from std_msgs.msg import String, Header
from amcl_msgs.msg import *
from amcl.srv import GetParticlecloud, SetParticlecloud
from geometry_msgs.msg import PoseArray, Pose2D, PoseWithCovarianceStamped
import tf.transformations

from scipy.stats import multivariate_normal
import numpy as np
from sklearn.neighbors import KernelDensity
import matplotlib.pyplot as plt

import math
import argparse
from copy import copy

from pose_conversions import *
from particle_filter_ops import *

### GLOBAL VARIABLES ### 
VERBOSE = False 
X_DOMAIN = 1.5 # meters
Y_DOMAIN = 1.5 # meters
THETA_DOMAIN = math.pi  # radians
MIN_PROB = 1e-10
NUM_UNIFORM_PARTICLES = 50
SYMETRIC_UPDATE = False
UPDATE_DISTANCE = 2  # meters
UPDATE_TIME = rospy.Duration(2.0) # Seconds

# map of robot names to global pose topic 
global_pose_topic = {

}
# map of robot names to current localization data 
current_loc_data = {

}
# map of robot names to current particle cloud data 
current_particlecloud_data = {

}
# map of current relative position data 
current_relative_pose_data = {

}
# map of last position when detection occurred and distributed localization was performed
last_update_location_data = {

}
# map of last time that detection occurred and distributed localization was performed
last_update_time = {

}

def sync_loc_data(robot_n, robot_m):
    """
    Sync robot_n and robot_m's maps using detection data between them
    Note: This function should be called on new detection data 
    """
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

    # TODO: FIX THREADING CREATING 100 INSTANCES
    # Run probability for particle_cloud
    # Only run reweight for robot every UPDATE_TIME
    curr_time = rospy.Time.now()
    if robot_m not in last_update_time or (curr_time - last_update_time[robot_m]) > UPDATE_TIME:
        last_update_time[robot_m] = curr_time
        
        update_particlecloud(robot_n)
        update_particlecloud(robot_m)
        reweight_particlecloud(robot_n, robot_m, n_detect_m)
        
        if SYMETRIC_UPDATE and (robot_n not in last_update_time or last_update_time[robot_n] - curr_time > UPDATE_TIME):
            last_update_time[robot_n] = curr_time
            m_detect_n = n_detect_m
            m_detect_n.x = -m_detect_n.x
            m_detect_n.y = -m_detect_n.y
            m_detect_n.theta += math.pi
            reweight_particlecloud(robot_m, robot_n, m_detect_n)


def reweight_particlecloud(robot_n, robot_m, detection_m):
    global last_update_location_data

    # get position of robot n
    position_m = pose2d_to_numpy(pose_to_pose2d_rad(current_loc_data[robot_m].pose.pose))[0:2]  
    # convert detection to radians
    detection_m = pose2d_degree_to_pose2d_rad(detection_m)
    # Check that the robot has moved far enough for reweight
    if robot_m not in last_update_location_data or np.linalg.norm(last_update_location_data[robot_m] - position_m) > UPDATE_DISTANCE:
        rospy.loginfo("Attempting update of particle cloud for " + robot_m + " with detection from " + robot_n)
        # Get particle clouds for robot m and n
        particles_n = particles_from_particlecloud(current_particlecloud_data[robot_n])
        particles_m = particles_from_particlecloud(current_particlecloud_data[robot_m])
        # Add in uniform distribution of particles to robot m
        particles_m = add_uniform_data(particles_m, n=NUM_UNIFORM_PARTICLES, min_x=-X_DOMAIN, max_x=X_DOMAIN, min_y=-Y_DOMAIN, max_y=Y_DOMAIN, min_t=0, max_t=2*THETA_DOMAIN)
        # Translate robot n's particles by the detection
        translate = [detection_m.x, detection_m.y, detection_m.theta, 0]
        translated_data = translate_data(particles_n, translate)
        # Update the weights by multiplying the the translated particle filer and robot m's filter
        particles_m = update_weights(translated_data, particles_m, min_prob=MIN_PROB)
        if particles_m is not None:
            # Update the particle filter
            rospy.loginfo("Updating particle cloud for " + robot_m)
            particles_m = get_best_particles(particles_m, len(particles_m) - NUM_UNIFORM_PARTICLES)
            normalize_weights(particles_m)
            # Sort the particles from high to low probability
            if set_particlecloud(robot_m, particlecloud_from_particles(particles_m)):
                # Set updated location
                last_update_location_data[robot_m] = position_m
        else:
            rospy.loginfo("Reweighing the particle cloud for robot " + robot_m + " resulted in error. Particle cloud will not be updated")


def update_particlecloud(robot):
    """
    Calls the get_particlecloud service
    """
    rospy.wait_for_service(str(robot) + '/get_particlecloud')
    try:
        get_particles = rospy.ServiceProxy(str(robot) + '/get_particlecloud', GetParticlecloud)
        current_particlecloud_data[robot] = get_particles()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_particlecloud(robot, particlecloud):
    """
    Calls the set_particlecloud service
    """
    rospy.wait_for_service(str(robot) + '/set_particlecloud')
    try:
        set_particlecloud = rospy.ServiceProxy(str(robot) + '/set_particlecloud', SetParticlecloud)
        set_particlecloud(particlecloud)
        return True
    except rospy.ServiceException, e:
        print(particlecloud)
        print "Service call failed: %s" % e
        return False


def amcl_pose_callback(amcl_pose,robot_name):
    """
    Callback function for amcl pose topics
    """
    global current_loc_data

    current_loc_data[robot_name] = amcl_pose


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

    rospy.init_node('distrib_loc', anonymous=True)
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

        bot_amcl_topic = bot_name + "/amcl_pose"
        sub = rospy.Subscriber(bot_amcl_topic, PoseWithCovarianceStamped, amcl_pose_callback, callback_args=bot_name) 

    # set up callbacks for relative position data
    for bot_name in current_relative_pose_data.keys():
        for other_bot_name in current_relative_pose_data.keys():
            if other_bot_name != bot_name:
                topic_start = bot_name + "/" + other_bot_name
                relative_pose_topic_name = topic_start + "/relative_pose"                
                current_relative_pose_data[bot_name][other_bot_name] = None
                sub = rospy.Subscriber(relative_pose_topic_name, Pose2D, relative_pose_callback, callback_args=(bot_name,other_bot_name))

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        run_sync(3, 1)
    except rospy.ROSInterruptException:
        pass
