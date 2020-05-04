import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import math
import time
from copy import copy


def plot_particle_data(data1, data2=None, name=None):
    if data2 is None:
        # Plot the data
        fig, ax = plt.subplots()
        # Get list of x,y directions (from yaw)
        arrow_x = [math.cos(yaw) for yaw in data1[:, 2]]
        arrow_y = [math.sin(yaw) for yaw in data1[:, 2]]
        im = ax.scatter(data1[:, 0], data1[:, 1], c=data1[:, 3])
        im = ax.quiver(data1[:, 0], data1[:, 1], arrow_x, arrow_y, data1[:, 3])
    else:
        # Plot the data
        fig, (ax1,ax2) = plt.subplots(2)
        # Get list of x,y directions (from yaw)
        arrow_x = [math.cos(yaw) for yaw in data1[:, 2]]
        arrow_y = [math.sin(yaw) for yaw in data1[:, 2]]
        im = ax1.scatter(data1[:, 0], data1[:, 1], c=data1[:, 3])
        im = ax1.quiver(data1[:, 0], data1[:, 1], arrow_x, arrow_y, data1[:, 3])
        if name is not None:
            plt.title(name)
        # Get list of x,y directions (from yaw)
        arrow_x = [math.cos(yaw) for yaw in data2[:, 2]]
        arrow_y = [math.sin(yaw) for yaw in data2[:, 2]]
        im = ax2.scatter(data2[:, 0], data2[:, 1], c=data2[:, 3])
        im = ax2.quiver(data2[:, 0], data2[:, 1], arrow_x, arrow_y, data2[:, 3])


    

    fig.subplots_adjust(right=0.8)
    cbar_ax = fig.add_axes([0.85, 0.15, 0.05, 0.7])
    fig.colorbar(im, cax=cbar_ax)
    
    plt.show()


def translate_data(x, translate):
    """
    Translates the positions of x by translate and then rotates the yaw by angle
    x: [[x,y,yaw,weight],...]
    translate: [x_offset, y_offset, yaw_offset, weight_offset]
    """
    t = copy(x)
    translate = np.array(translate)

    for data in t:
        # Translate position along arrow
        theta = data[2] - math.pi/2
        r = np.array(((np.cos(theta), -np.sin(theta)), (np.sin(theta), np.cos(theta))))
        data[0:2] += r.dot([translate[0], translate[1]])
        # Rotate yaw
        data[2] += translate[2]
        # Translate weight
        data[3] += translate[3]
    # Convert to numpy array
    t_data = np.asarray(t)
    return t_data


def generate_circle_data(rad=1, n=100):
    """
    Generates an array like [[x,y,yaw,weight],...] where the positions (x,y) are in a cirlce
    of radius rad and the yaw makes the normal directed out of the circle
    """
    circle_data = np.zeros((n, 4))
    for i, data in enumerate(circle_data):
        angle = i/n * 2 * math.pi
        data[0:2] = [rad * math.cos(angle), rad * math.sin(angle)]
        data[2] = angle
    return circle_data


def generate_line_data(width=1, yaw=math.pi/2, n=100):
    """
    Generates an array like [[x,y,yaw,weight],...] where the positions (x,y) are in a line
    of width and the yaw pointing straight up
    """
    width_data = np.zeros((n, 4))
    for i, data in enumerate(width_data):
        x = (i/n * width) - width/2
        data[0:2] = [x, 0]
        data[2] = yaw
    return width_data


if __name__ == "__main__":
    # Circle test
    plot_particle_data(generate_circle_data(), translate_data(generate_circle_data(), [0, 1, math.pi, 0]), "circle data : [0,1,pi,0]")
    plot_particle_data(generate_circle_data(), translate_data(generate_circle_data(), [1, 0, math.pi, 0]), "circle data : [1,0,pi,0]")
    # Line test
    plot_particle_data(generate_line_data(), translate_data(generate_line_data(), [0, 1, math.pi, 0]), "line data : [0,1,pi,0]")
    plot_particle_data(generate_line_data(), translate_data(generate_line_data(), [1, 0, math.pi, 0]), "line data : [1,0,pi,0]")
    plot_particle_data(generate_line_data(yaw=0), translate_data(generate_line_data(yaw=0), [0, 1, math.pi, 0]), "line data : [0,1,pi,0]")
    plot_particle_data(generate_line_data(yaw=0), translate_data(generate_line_data(yaw=0), [1, 0, math.pi, 0]), "line data : [1,0,pi,0]")
    plot_particle_data(generate_line_data(yaw=0), translate_data(generate_line_data(yaw=0), [0, -1, math.pi, 0]), "line data : [0,-1,pi,0]")
    plot_particle_data(generate_line_data(yaw=0), translate_data(generate_line_data(yaw=0), [-1, 0, math.pi, 0]), "line data : [-1,0,pi,0]")
    