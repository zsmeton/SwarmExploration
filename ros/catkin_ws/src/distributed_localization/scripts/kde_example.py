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
import kernel_density_estimator as kde
import time
from sklearn.neighbors import KernelDensity

def get_particlecloud():
    """
    Calls the get_particlecloud service
    Returns: a GetParticlecloudResponse class
    """
    rospy.wait_for_service('tb3_0/get_particlecloud')
    try:
        get_particles = rospy.ServiceProxy('tb3_0/get_particlecloud', GetParticlecloud)
        resp1 = get_particles()
        return resp1
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


def plot_particle_data(particle_data):
    # Plot the data
    fig, ax = plt.subplots()
    # Get list of x,y directions (from yaw)
    arrow_x = [math.cos(yaw) for yaw in particle_data[:, 2]]
    arrow_y = [math.sin(yaw) for yaw in particle_data[:, 2]]
    im = ax.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3])
    im = ax.quiver(particle_data[:,0], particle_data[:,1], arrow_x, arrow_y, particle_data[:,3])
    fig.colorbar(im)
    plt.title("Particle Data")
    fig.tight_layout()
    plt.show()


def silverman_rule(num_samples, num_features):
    return (num_samples * (num_features + 2) / 4.)**(-1. / (num_features + 4))

def plot_density_estimation(particle_data, kernel='gaussian'):
    # Fit the kernel
    samples = particle_data[:, 0:2] # Sample data should be of shape (num samples, features)
    weights = particle_data[:, 3] # Weight data should be of shape (num samples, )
    # Normalize the weights
    weights = np.array(weights, np.float)
    weights /= np.sum(weights)

    start_time = time.time()
    pdf = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*particle_data[:,0:2].shape)).fit(particle_data[:, 0:2])
    end_time = time.time()
    print "KDE Fit Elapsed Time: ", end_time-start_time 

    # Evaluate the kde on a grid
    xmin = min(min(particle_data[:, 0]), min(particle_data[:, 1]))
    xmax = max(max(particle_data[:, 0]), max(particle_data[:, 1]))
    x = np.linspace(xmin, xmax, 100)
    xx, yy = np.meshgrid(x, x)
    print(np.ravel(xx.shape))

    start_time = time.time()
    zz = pdf.score_samples(np.vstack((np.ravel(xx), np.ravel(yy))).T)
    end_time = time.time()
    print "Grid Size: ", len(xx), "x", len(yy)
    print "KDE Eval Grid Elapsed Time: ", end_time-start_time 
    
    zz = np.reshape(zz, xx.shape)
    # Plot the density estimation
    kwargs = dict(extent=(xmin, xmax, xmin, xmax), cmap='hot', origin='lower')
    plt.imshow(zz.T, **kwargs)
    plt.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3])
    plt.title('kde')
    plt.show()

def get_user_yes_no(prompt):
    response = raw_input(prompt)
    response = response.capitalize()
    while response != "Y" and response != "N":
        response = raw_input(prompt)
        response = response.capitalize()
    return response == "Y"

if __name__ == '__main__':
    try:
        if get_user_yes_no("Would you like to load the data from a file [y/n]? "):
            #filename = raw_input("Filename: ")
            particle_data = np.loadtxt("../saved_data/particle_cloud_2.txt")
        else:
            # Get the particle data
            particlecloud = get_particlecloud()
            # Convert data from particlecloud to particle data numpy array
            particle_data = particles_from_particlecloud(particlecloud)
            # Save particle data to file
            if get_user_yes_no("Would you like to save the data [y/n]? "):
                filename = raw_input("Filename: ")
                np.savetxt(filename, particle_data)

        # Plot the particle data
        plot_particle_data(particle_data)
        # Fit and plot kde density estimation
        plot_density_estimation(particle_data)
        
        

    except rospy.ROSInterruptException:
        pass
