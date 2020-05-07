#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from amcl_msgs.msg import WeightedParticlecloud
from amcl.srv import GetParticlecloud, SetParticlecloud
import matplotlib as mpl
import matplotlib.pyplot as plt
import tf
import numpy as np
import math
import time


def get_particlecloud(robot):
    """
    Calls the get_particlecloud service
    Returns: a GetParticlecloudResponse class
    """
    rospy.loginfo(robot)
    rospy.wait_for_service(str(robot) + '/get_particlecloud')
    try:
        get_particles = rospy.ServiceProxy(str(robot) + '/get_particlecloud', GetParticlecloud)
        return get_particles()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


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


if __name__ == '__main__':
    try:
        # Get the particle data
        # Convert data from particlecloud to particle data numpy array
        particle_data = particles_from_particlecloud(get_particlecloud('tb3_1'))
        required_data = particles_from_particlecloud(get_particlecloud('tb3_2'))
        #trans_data = translate_data(particle_data, [-0.3232, 0.80197, 0, 0], 1.75)

        # Plot the particle data
        plot_particle_data(particle_data)
        plot_particle_data(required_data)
        #plot_particle_data(trans_data)
        # Fit and plot kde density estimations
        #plot_density_estimation(trans_data, required_data)
        np.savetxt("saved_data/particle_cloud_1_post.txt", np.array(particle_data))
        np.savetxt("saved_data/particle_cloud_2_post.txt", np.array(required_data))
        
    except rospy.ROSInterruptException:
        pass
