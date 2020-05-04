import math
from scipy.stats import multivariate_normal
import numpy as np
from copy import copy
from sklearn.neighbors import KernelDensity
import matplotlib.pyplot as plt
from operator import itemgetter 

"""
GENERAL
"""

def silverman_rule(num_samples, num_features):
    """
    Uses silverman's rule to estimate an appropriate bin size to use in a kde
    returns: bin width
    """
    return (num_samples * (num_features + 2) / 4.)**(-1. / (num_features + 4))


def update_weights(particles_n, particles_m, kernel='gaussian', min_prob=1e-9):
    """
    Multiplies the particle filter n onto particle m, updating particle m's weights
    returns: updated particle filter m, (max,min), or none is max prob is < min_prob
    """
    #plot_density_estimation(particles_n)
    #plot_density_estimation(particles_m)

    # Fit the density estimation kernels
    samples_n = particles_n[:, 0:3] # Sample data should be of shape (num samples, features)
    kde_n = KernelDensity(kernel=kernel, bandwidth=silverman_rule(*samples_n.shape)).fit(samples_n)
    samples_m = particles_m[:, 0:3] # Sample data should be of shape (num samples, features)
    kde_m = KernelDensity(kernel=kernel, bandwidth=silverman_rule(*samples_m.shape)).fit(samples_m)
    # Update the weights by multiplying the kdes together
    new_density = particles_m
    positions = particles_m[:, 0:3]
    new_density[:, 3] = np.exp(kde_m.score_samples(positions)) * np.exp(kde_n.score_samples(positions))
    if max(new_density[:, 3]) > min_prob:
        if (min(new_density[:, 3]) < min_prob):
            new_density[:, 3] += min_prob
        new_density = normalize_weights(new_density)
        return new_density
    else:
        return None


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
    return np.array(t)


def add_uniform_data(x, n=10, min_x=-5, max_x=5, min_y=-5, max_y=5, min_t=0, max_t=math.pi*2, weight='avg', origin='avg'):
    """
    Adds a set of random, uniformly distributed particles to the particle data x
    x: [[x,y,yaw,weight],...]
    n: number of particles to add
    min_x: minimum x position
    max_x: maximum x position
    min_y: minimum y position
    max_y: maximum y position
    min_t: minimum yaw
    max_t: maximum yaw
    weight: method to select weight ['avg', 'max', 'min'] or number to set the weights to
    origin: where to offset the position from ['avg', 'origin']
    returns: x + uniform data
    """
    t = np.asarray(copy(x))

    # Set weight function
    weight_function = None
    if weight == 'avg':
        weight_function = np.average
    elif weight == 'max':
        weight_function = max
    elif weight == 'min':
        weight_function = min
    elif str(weight).isdigit:
        weight_function = lambda x : float(weight)
    else:
        raise ValueError("weight must be 'avg', 'max', or 'min'")

    # Set origin function
    origin_function = None
    if origin == 'avg':
        origin_function = lambda x : np.average(x, axis=0)
    elif origin == 'origin':
        origin_function = lambda x : [0,0]
    else:
        raise ValueError("origin must be 'avg' or 'origin'")

    # Get origin
    origin_pos = origin_function(t[:,0:2])

    # Get data to append
    xs = np.random.uniform(min_x + origin_pos[0], max_x + origin_pos[0], n)
    ys = np.random.uniform(min_y + origin_pos[1], max_y + origin_pos[1], n)
    ts = np.random.uniform(min_t, max_t, n)
    w = weight_function(t[:,3])
    ws = np.random.uniform(w, w, n)

    # append to array
    u = np.stack((xs, ys, ts, ws), axis=-1)
    t = np.concatenate((t,u))

    return t

def get_best_particles(x, n):
    """
    Returns the n particles from x with the heighest probabilities
    x: [[x,y,yaw,weight],...]
    n: Number of particles to grab
    returns: sorted_x[:n]
    """
    # Sort the particles from high to low probability
    s_x = sorted(x, key = itemgetter(3), reverse=True) 
    return np.array(s_x[:n])


def normalize_weights(x):
    """
    Normalizes the weight of the particles in x
    returns: normalized x
    """
    xc = copy(x)
    xc[:, 3] /= np.sum(xc[:, 3])
    return xc



def plot_density_estimation(particle_data, kernel='gaussian'):
    # Fit the kernel
    samples = particle_data[:, 0:2] # Sample data should be of shape (num samples, features)
    weights = particle_data[:, 3] # Weight data should be of shape (num samples, )
    # Normalize the weights
    weights = np.array(weights, np.float)
    weights /= np.sum(weights)

    #print "bin width", silverman_rule(*particle_data[:,0:2].shape)
    pdf = KernelDensity(kernel=kernel, bandwidth=silverman_rule(*particle_data[:,0:2].shape)).fit(particle_data[:, 0:2])

    # Evaluate the kde on a grid
    xmin, xmax = min(particle_data[:, 0]), max(particle_data[:, 0])
    print(xmin, xmax)
    ymin, ymax = min(particle_data[:, 1]), max(particle_data[:, 1])
    print(ymin, ymax)
    x = np.linspace(xmin, xmax, 100)
    y = np.linspace(ymin, ymax, 100)
    xx, yy = np.meshgrid(x, y)

    zz = pdf.score_samples(np.vstack([xx.ravel(), yy.ravel()]).T)
    #print "Grid Size: ", len(xx), "x", len(yy)
    
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


"""
ROS
"""
def particles_from_particlecloud(particlecloud):
    import tf.transformations
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
    import rospy
    from pose_conversions import pose2d_rad_to_pose, numpy_to_pose2d
    from std_msgs.msg import String, Header
    from amcl_msgs.msg import WeightedParticlecloud
    """
    Turns GetParticlecloudResponse into a list of particle data
    [[x,y,yaw,weight], ...]
    """
    weights = particles[:, 3]
    h = Header(stamp=rospy.Time.now())
    poses = [pose2d_rad_to_pose(numpy_to_pose2d(particle[0:3])) for particle in particles]
    return WeightedParticlecloud(header=h, poses=poses, weights=weights)


if __name__ == "__main__":
    print(add_uniform_data(np.zeros((10, 4))))
    print(get_best_particles(add_uniform_data(translate_data(np.zeros((10, 4)), [0, 0, 0, 1]), weight=0), 12))