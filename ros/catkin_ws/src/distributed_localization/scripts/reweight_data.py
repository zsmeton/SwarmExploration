#!/usr/bin/env python
# license removed for brevity

# sync_loc_on_detect.py
# Synchronize probabilistic locations of multiple robots using detection data

import rospy
from std_msgs.msg import String, Header
from amcl_msgs.msg import *
from amcl.srv import GetParticlecloud, SetParticlecloud
from geometry_msgs.msg import PoseArray, Pose2D, PoseWithCovarianceStamped
import argparse
import tf.transformations
import math
from scipy.stats import multivariate_normal
import numpy as np
from pose_conversions import *
from copy import copy
from sklearn.neighbors import KernelDensity
import matplotlib.pyplot as plt

### GLOBAL VARIABLES ### 
VERBOSE = False 

X_VARIANCE = 0.5 # meters^2
Y_VARIANCE = 0.5 # meters^2
THETA_VARIANCE = 0.1  # radians^2
X_DOMAIN = 10 # meters
Y_DOMAIN = 10 # meters
THETA_DOMAIN = math.pi  # radians
MIN_PROB = 0.001
UPDATE_DISTANCE = 2 # meters

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

    # Run probability for particle_cloud
    update_particlecloud(robot_n)
    update_particlecloud(robot_m)
    reweight_particlecloud(robot_n, robot_m, n_detect_m)
    #m_detect_n = n_detect_m
    #m_detect_n.x = -m_detect_n.x
    #m_detect_n.y = -m_detect_n.y
    #m_detect_n.theta += math.pi
    #reweight_particlecloud(robot_m, robot_n, m_detect_n)


def reweight_particlecloud(robot_n, robot_m, detection_m):
    global last_update_location_data

    position_m = pose2d_to_numpy(pose_to_pose2d_rad(current_loc_data[robot_m].pose.pose))[0:2] # get position of robot n
    # Check that the robot has moved far enough for reweight
    if robot_m not in last_update_location_data or np.linalg.norm(last_update_location_data[robot_m] - position_m) > UPDATE_DISTANCE:
        # convert detection to radians
        detection_m = pose2d_degree_to_pose2d_rad(detection_m)
        # Get particle clouds for robot m and n
        particles_n = particles_from_particlecloud(current_particlecloud_data[robot_n])
        particles_m = particles_from_particlecloud(current_particlecloud_data[robot_m])
        # Translate robot n's particles by the detection
        translate = [detection_m.x, detection_m.y, detection_m.theta, 0]
        translated_data = translate_data(particles_n, translate)
        # Update the weights by multiplying the the translated particle filer and robot m's filter
        particles_m = update_weights(translated_data, particles_m)
        if particles_m is not None:
            # Set updated location
            last_update_location_data[robot_m] = position_m
            # Update the particle filter
            rospy.loginfo("Updating particle cloud for " + robot_m)
            set_particlecloud(robot_m, particlecloud_from_particles(particles_m))
        else:
            rospy.loginfo("Reweighing the particle cloud for robot " + robot_m + " resulted in error. Particle cloud will not be updated")
        

def plot_density_estimation(particle_data, kernel='gaussian'):
    # Fit the kernel
    samples = particle_data[:, 0:2] # Sample data should be of shape (num samples, features)
    weights = particle_data[:, 3] # Weight data should be of shape (num samples, )
    # Normalize the weights
    weights = np.array(weights, np.float)
    weights /= np.sum(weights)

    print "bin width", silverman_rule(*particle_data[:,0:2].shape)
    pdf = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*particle_data[:,0:2].shape)).fit(particle_data[:, 0:2])

    # Evaluate the kde on a grid
    xmin, xmax = min(particle_data[:, 0]), max(particle_data[:, 0])
    print(xmin, xmax)
    ymin, ymax = min(particle_data[:, 1]), max(particle_data[:, 1])
    print(ymin, ymax)
    x = np.linspace(xmin, xmax, 100)
    y = np.linspace(ymin, ymax, 100)
    xx, yy = np.meshgrid(x, y)

    zz = pdf.score_samples(np.vstack([xx.ravel(), yy.ravel()]).T)
    print "Grid Size: ", len(xx), "x", len(yy)
    
    zz = np.reshape(zz, xx.shape)
    # Plot the density estimation
    levels = np.linspace(zz.min(), zz.max(), 25)
    plt.contourf(xx, yy, zz, levels=levels, cmap=plt.cm.Reds)
    plt.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3])
    plt.title('kde')
    plt.show()


def plot_particle_data(particle_data, name=None):
    # Plot the data
    fig, ax = plt.subplots()
    # Get list of x,y directions (from yaw)
    arrow_x = [math.cos(yaw) for yaw in particle_data[:, 2]]
    arrow_y = [math.sin(yaw) for yaw in particle_data[:, 2]]
    im = ax.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3])
    im = ax.quiver(particle_data[:,0], particle_data[:,1], arrow_x, arrow_y, particle_data[:,3])

    fig.colorbar(im)
    if name is not None:
        plt.title(name)
    fig.tight_layout()
    plt.show()


def update_weights(particles_n, particles_m, kernel='gaussian'):
    """
    Multiplies the particle filter n onto particle m, updating particle m's weights
    returns: updated particle filter m
    """
    #plot_density_estimation(particles_n)
    #plot_density_estimation(particles_m)

    # Fit the density estimation kernels
    samples_n = particles_n[:, 0:3] # Sample data should be of shape (num samples, features)
    kde_n = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*samples_n.shape)).fit(samples_n)
    samples_m = particles_m[:, 0:3] # Sample data should be of shape (num samples, features)
    kde_m = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*samples_m.shape)).fit(samples_m)
    # Update the weights by multiplying the kdes together
    new_density = particles_m
    positions = particles_m[:,0:3]
    new_density[:, 3] = np.exp(kde_m.score_samples(positions)) * np.exp(kde_n.score_samples(positions))
    print(np.sum(new_density[:, 3]), max(new_density[:, 3]))
    if max(new_density[:, 3]) > MIN_PROB:
        if (min(new_density[:, 3]) < MIN_PROB):
            new_density[:, 3] += MIN_PROB
        new_density[:, 3] /= np.sum(new_density[:,3])
        return new_density
    else:
        return None


def silverman_rule(num_samples, num_features):
    return (num_samples * (num_features + 2) / 4.)**(-1. / (num_features + 4))


def rotate_via_numpy(xy, radians):
    """Use numpy to build a rotation matrix and take the dot product."""
    x, y = xy
    c, s = np.cos(radians), np.sin(radians)
    j = np.matrix([[c, s], [-s, c]])
    m = np.dot(j, [x, y])

    return float(m.T[0]), float(m.T[1])


def translate_data(related_data, translate):
    """
    Translates the positions of related_data by translate and then rotates the yaw by angle
    related_data: [[x,y,yaw,weight],...]
    translate: [x_offset, y_offset, yaw_offset, weight_offset]
    """
    related_data = copy(related_data)
    translate = np.array(translate)
    t_data = []

    for data in related_data:
        updated_data = data
        # Translate
        theta = data[2] - math.pi/2
        r = np.array(((np.cos(theta), -np.sin(theta)), (np.sin(theta), np.cos(theta))))
        updated_data[0:2] += r.dot([translate[0], translate[1]])
        updated_data[2] += translate[2]
        t_data.append(updated_data)
        updated_data[3] += translate[3]
    t_data = np.asarray(t_data)

    return t_data


def particles_from_particlecloud(particlecloud):
    """
    Turns GetParticlecloudResponse into a list of particle data
    [[x,y,yaw,weight], ...]
    """
    particle_data = []
    for pose, weight in zip(particlecloud.cloud.poses, particlecloud.cloud.weights):
        # get x,y
        x = pose.position.x
        y = pose.position.y
        # get yaw
        # euler_from_quaternion -> (roll, pitch, yaw)
        yaw = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
        # append data to list
        particle_data.append([x, y, yaw, weight])
    return np.array(particle_data)


def particlecloud_from_particles(particles):
    """
    Turns GetParticlecloudResponse into a list of particle data
    [[x,y,yaw,weight], ...]
    """
    weights = particles[:, 3]
    h = Header(stamp=rospy.Time.now())
    poses = [pose2d_rad_to_pose(numpy_to_pose2d(particle[0:3])) for particle in particles]
    return WeightedParticlecloud(header=h, poses=poses, weights=weights)


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
    rospy.loginfo(particlecloud)
    rospy.wait_for_service(str(robot) + '/set_particlecloud')
    try:
        set_particlecloud = rospy.ServiceProxy(str(robot) + '/set_particlecloud', SetParticlecloud)
        set_particlecloud(particlecloud)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


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
