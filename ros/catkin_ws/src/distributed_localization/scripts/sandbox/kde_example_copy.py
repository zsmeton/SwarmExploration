#!/usr/bin/env python
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
import tf
import numpy as np
import math
import kernel_density_estimator as kde
import time

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


def get_user_yes_no(prompt):
    response = raw_input(prompt)
    response = response.capitalize()
    while response != "Y" and response != "N":
        response = raw_input(prompt)
        response = response.capitalize()
    return response == "Y"

if __name__ == '__main__':
    try:
        if True:
            # Robot 1 detects robot 2, robot 2's weight gets updated
            filename1 = "../saved_data/particle_cloud_1.txt"
            filename2 = "../saved_data/particle_cloud_2.txt"
            #filename = raw_input("Filename: ")
            particle_data = np.loadtxt(filename1)
            required_data = np.loadtxt(filename2)
            trans_data = translate_data(particle_data, [-0.77502, 1.999, 0, 0], 2.718875219021052)

        else:
            pass
            # Get the particle data
            # particlecloud = get_particlecloud()
            # Convert data from particlecloud to particle data numpy array
            #particle_data = particles_from_particlecloud(particlecloud)

        # Plot the particle data
        plot_particle_data(particle_data, "Particle Data")
        plot_particle_data(required_data, "Required Data")
        plot_particle_data(trans_data, "Translated Data")
        # Fit and plot kde density estimation
        plot_density_estimation(trans_data, required_data)

    except rospy.ROSInterruptException:
        pass
