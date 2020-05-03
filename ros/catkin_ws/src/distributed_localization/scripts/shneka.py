#!/usr/bin/env python
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
import tf
import numpy as np
import math
from sklearn.neighbors import KernelDensity
import time
from copy import copy

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
    translate_data = []

    for data in related_data:
        updated_data = data
        # Translate
        theta = data[2] - math.pi/2
        r = np.array(((np.cos(theta), -np.sin(theta)), (np.sin(theta), np.cos(theta))))
        updated_data[0:2] += r.dot([translate[0], translate[1]])
        updated_data[2] += translate[2]
        translate_data.append(updated_data)
    translate_data = np.asarray(translate_data)

    return translate_data

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
    print "bin width", silverman_rule(*particle_data[:,0:2].shape)
    pdf = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*particle_data[:,0:2].shape)).fit(particle_data[:, 0:2])
    end_time = time.time()
    print "KDE Fit Elapsed Time: ", end_time-start_time 

    # Evaluate the kde on a grid
    xmin, xmax = min(particle_data[:, 0]), max(particle_data[:, 0])
    print(xmin, xmax)
    ymin, ymax = min(particle_data[:, 1]), max(particle_data[:, 1])
    print(ymin, ymax)
    x = np.linspace(xmin, xmax, 100)
    y = np.linspace(ymin, ymax, 100)
    xx, yy = np.meshgrid(x, y)

    start_time = time.time()
    zz = pdf.score_samples(np.vstack([xx.ravel(), yy.ravel()]).T)
    end_time = time.time()
    print "Grid Size: ", len(xx), "x", len(yy)
    print "KDE Eval Grid Elapsed Time: ", end_time-start_time 
    
    zz = np.reshape(zz, xx.shape)
    # Plot the density estimation
    levels = np.linspace(zz.min(), zz.max(), 25)
    plt.contourf(xx, yy, zz, levels=levels, cmap=plt.cm.Reds)
    plt.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3])
    plt.title('kde')
    plt.show()


def reweight_data(particle_data, required_data, kernel='gaussian'):
    # Fit the kernels
    start_time = time.time()
    particle_samples = particle_data[:, 0:3] # Sample data should be of shape (num samples, features)
    print "bin width", silverman_rule(*particle_samples.shape)
    particle_pdf = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*particle_samples.shape)).fit(particle_samples)
    required_samples = required_data[:, 0:3] # Sample data should be of shape (num samples, features)
    print "bin width", silverman_rule(*required_samples.shape)
    required_pdf = KernelDensity(kernel='gaussian', bandwidth=silverman_rule(*required_samples.shape)).fit(required_samples)
    end_time = time.time()
    
    print "KDE Fit Elapsed Time: ", end_time-start_time 
    
    start_time = time.time()
    new_density = required_data
    positions = required_data[:,0:3]
    new_density[:, 3] = np.exp(required_pdf.score_samples(positions)) * np.exp(particle_pdf.score_samples(positions))
    new_density[:, 3] /= np.sum(new_density[:,3])
    end_time = time.time()
    print(sum(new_density[:3]))
    print "Reweight Elapsed Time: ", end_time-start_time 
    
    return new_density


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
            particle_data = np.loadtxt(filename1) # robot 1
            required_data = np.loadtxt(filename2) # robot 2
            trans_data = translate_data(particle_data, [-0.085904, 1.96416, 3.38, 0])

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
        plot_density_estimation(trans_data)
        plot_density_estimation(required_data)
        new_density = reweight_data(trans_data, required_data)
        plot_particle_data(new_density)

    except rospy.ROSInterruptException:
        pass
