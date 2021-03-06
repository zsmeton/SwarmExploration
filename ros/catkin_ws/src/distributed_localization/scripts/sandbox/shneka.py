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
import re

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
    return required_data


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


def plot_particle_data(particle_data, name=None):
    # Plot the data
    fig, ax = plt.subplots()
    # Get list of x,y directions (from yaw)
    arrow_x = [math.cos(yaw) for yaw in particle_data[:, 2]]
    arrow_y = [math.sin(yaw) for yaw in particle_data[:, 2]]
    im = ax.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3], s=1)
    im = ax.quiver(particle_data[:,0], particle_data[:,1], arrow_x, arrow_y, particle_data[:,3], cmap=plt.cm.Reds)

    fig.colorbar(im)
    if name is not None:
        plt.title(name)
    
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)

    plt.savefig(name+'.png')
    plt.show()

    


def silverman_rule(num_samples, num_features):
    return (num_samples * (num_features + 2) / 4.)**(-1. / (num_features + 4))


def plot_density_estimation(particle_data, kernel='gaussian', name=None):
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
    xmin, xmax = (-2.5,2.5) #min(particle_data[:, 0]), max(particle_data[:, 0])
    print(xmin, xmax)
    ymin, ymax = (-2.5,2.5) #min(particle_data[:, 1]), max(particle_data[:, 1])
    print(ymin, ymax)
    x = np.linspace(xmin, xmax, 100)
    y = np.linspace(ymin, ymax, 100)
    xx, yy = np.meshgrid(x, y)

    start_time = time.time()
    zz = np.exp(pdf.score_samples(np.vstack([xx.ravel(), yy.ravel()]).T))
    end_time = time.time()
    print "Grid Size: ", len(xx), "x", len(yy)
    print "KDE Eval Grid Elapsed Time: ", end_time-start_time 
    
    zz = np.reshape(zz, xx.shape)
    # Plot the density estimation
    levels = np.linspace(0, 1, 25)

    fig, ax = plt.subplots()
    im = ax.contourf(xx, yy, zz, levels=levels, cmap=plt.cm.Reds)
    # Get list of x,y directions (from yaw)
    arrow_x = [math.cos(yaw) for yaw in particle_data[:, 2]]
    arrow_y = [math.sin(yaw) for yaw in particle_data[:, 2]]

    ax.scatter(particle_data[:, 0], particle_data[:, 1], c=particle_data[:, 3], s=1)
    ax.quiver(particle_data[:,0], particle_data[:,1], arrow_x, arrow_y, particle_data[:,3])

    fig.colorbar(im)
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)

    plt.title(name)
    plt.savefig(name+'.png')
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


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))




def get_user_yes_no(prompt):
    response = raw_input(prompt)
    response = response.capitalize()
    while response != "Y" and response != "N":
        response = raw_input(prompt)
        response = response.capitalize()
    return response == "Y"

if __name__ == '__main__':
    try:
        #file_='../../maps/map_cropped.pgm'
        #image = read_pgm(file_, byteorder='<')
        #plt.imshow(image, plt.cm.gray)
        #plt.show()


        if True:
            # Robot 1 detects robot 2, robot 2's weight gets updated
            filename1 = "saved_data/particle_cloud_2_pre.txt"
            filename2 = "saved_data/particle_cloud_1_pre.txt"
            #filename = raw_input("Filename: ")
            particle_data = np.loadtxt(filename1) # robot 1
            required_data = np.loadtxt(filename2) # robot 2
            trans_data = translate_data(particle_data, [0.154284026244, 3.09919748583, 1.84971691093])

        else:
            pass
            # Get the particle data
            # particlecloud = get_particlecloud()
            # Convert data from particlecloud to particle data numpy array
            #particle_data = particles_from_particlecloud(particlecloud)

        # Plot the particle data
        plot_particle_data(particle_data, "Detecting Robot's Particle Filter")
        plot_particle_data(required_data, "Detected Robot's Particle Filter")
        plot_particle_data(trans_data, "Detecting Robot's Translated Particle Filter")
        # Fit and plot kde density estimation
        plot_density_estimation(trans_data, name="Density Estimation of Translated Particles")
        plot_density_estimation(required_data, name="Density Estimation of Detected Particles")
        new_density = reweight_data(trans_data, required_data)
        plot_particle_data(new_density, name='Updated Particle Filter')

    except rospy.ROSInterruptException:
        pass
