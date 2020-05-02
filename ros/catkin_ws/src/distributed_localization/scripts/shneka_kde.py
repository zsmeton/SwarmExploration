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


def generate_random_no(particle_data):
    # Generate random two dimensional data
    # n represents the number of data that must be generated
    minimum = min(min(particle_data[:, 0]), min(particle_data[:, 1]))
    maximum = max(max(particle_data[:, 0]), max(particle_data[:, 1])) 
    
    n = len(particle_data[:,0])
    # Generating random data within the same interval
    m1 = np.random.uniform(minimum, maximum, n)
    m2 = np.random.uniform(minimum, maximum, n)
    m3 = np.random.uniform(min(particle_data[:,2]), max(particle_data[:, 2]), n)
    m4 = np.ones(n) / n

    # Convert to stack the arrays as a list
    m1 = np.array([[i] for i in m1])
    m2 = np.array([[i] for i in m2])
    m3 = np.array([[i] for i in m3])
    m4 = np.array([[i] for i in m4])
    required_data = np.hstack([m1,m2,m3,m4])   
    
    #translate_data (required_data, [0.5, 0.5, 0, 0], 1.57)
    return required_data

def translate_data(related_data, translate, angle):
    translate_data = []
    
    for i in range(len(related_data)):
        #print("related data", related_data[i,:])
        rotational_matrix = np.matrix([[math.cos(angle),-math.sin(angle), 0, 0], [math.sin(angle), math.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        data = rotational_matrix.dot(related_data[i, :]) +  translate 
        data = np.squeeze(np.asarray(data))
        translate_data.append(data)
    translate_data = np.asarray(translate_data)

    return translate_data
    #print("translational data",translate_data)

    #sub_plot_particle_data(related_data, translate_data)
    #plot_particle_data(related_data)
    #plot_particle_data(translate_data)   


def sub_plot_particle_data(particle_data, translate_data):
    # Plot the data
    fig, (ax1, ax2) = plt.subplots(2)
    # Get list of x,y directions (from yaw)
    arrow_x = [math.cos(yaw) for yaw in particle_data[:, 2]]
    arrow_y = [math.sin(yaw) for yaw in particle_data[:, 2]]
    arrow_x_1 = [math.cos(yaw) for yaw in translate_data[:, 2]]
    arrow_y_1 = [math.sin(yaw) for yaw in translate_data[:, 2]]

    im = ax1.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3])
    im = ax1.quiver(particle_data[:,0], particle_data[:,1], arrow_x, arrow_y, particle_data[:,3])
    fig.colorbar(im)
    
    im1 = ax2.scatter(translate_data[:, 0], translate_data[:, 1], c=translate_data[:, 3])
    im1 = ax2.quiver(translate_data[:,0], translate_data[:,1], arrow_x_1, arrow_y_1, translate_data[:,3])
    fig.colorbar(im1)


    plt.title("Particle Data")
    fig.tight_layout()
    plt.show()


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

# Predict the bin that the point belongs to
def predict_mesh_grid(minimum, maximum, division, coord):
    division_length = (maximum - minimum)/ division
    grid_1 = math.floor((coord[0] - minimum)/division_length)
    grid_2 = math.floor((coord[1] - minimum)/division_length)
    
    if grid_1 >= division:
        grid_1 = division - 1
    elif grid_1 < 0:
        grid_1 = 0

    if grid_2 >= division:
        grid_2 = division - 1
    elif grid_2 < 0:
        grid_2 = 0

    return grid_1, grid_2

def map_integration(particle_data, required_data, zz_1):
    minimum = min(min(particle_data[:,0]), min(particle_data[:, 1]))
    maximum = max(max(particle_data[:,0]), max(particle_data[:, 1]))

    for i in range(len(required_data[:,0])):
        grid_1 , grid_2 = predict_mesh_grid(minimum, maximum, 100, [required_data[i,0], required_data[i,1]])
        required_data[i,3] *= zz_1[grid_1][grid_2]

    required_data[:,3] = required_data[:,3]/np.sum(required_data[:,3])

    print(np.sum(required_data[:,3]))
    return required_data


def plot_density_estimation(particle_data, required_data, kernel='gaussian'):
    # Fit the kernel
    # Samples just has x and y (no orientation)
    samples = particle_data[:, 0:2].T # Sample data should be of shape (features, num samples)
    print samples
    weights = particle_data[:, 3] # Weight data should be of shape (num samples, )
    # Normalize the weights
    weights = np.array(weights, np.float)
    weights /= np.sum(weights)

    start_time = time.time()
    pdf = kde.gaussian_kde(samples, weights=weights)
    end_time = time.time()
    
    print "KDE Fit Elapsed Time: ", end_time-start_time 
    # Evaluate the kde on a grid
    xmin = min(min(particle_data[:, 0]), min(particle_data[:, 1]))
    xmax = max(max(particle_data[:, 0]), max(particle_data[:, 1]))
    x = np.linspace(xmin, xmax, 100)
    # Meshgrid creates points from x_min to x_max with 100 points in between.
    xx, yy = np.meshgrid(x, x)

    start_time = time.time()
    # Creates list of a list to a list with continuous values.
    # Evaluates PDF on a particular set of points
    zz = pdf((np.ravel(xx), np.ravel(yy)))
    end_time = time.time()
    print "Grid Size: ", len(xx), "x", len(yy)
    print "KDE Eval Grid Elapsed Time: ", end_time-start_time 
    zz = np.reshape(zz, xx.shape)
    zz_1 = np.rot90(zz,3)

    # Multiply map data to known information.
    new_density = map_integration(particle_data, required_data, zz_1)
    plot_particle_data(new_density)


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
        np.savetxt("saved_data/particle_cloud_1.txt", np.array(particle_data))
        np.savetxt("saved_data/particle_cloud_2.txt", np.array(required_data))
        
    except rospy.ROSInterruptException:
        pass
