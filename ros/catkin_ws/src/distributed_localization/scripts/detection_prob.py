import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from scipy.stats import multivariate_normal
from scipy.stats.mvn import mvnun

X_VARIANCE = 0.1 # meters^2
Y_VARIANCE = 0.1 # meters^2
THETA_VARIANCE = 0.01  # radians^2
X_DOMAIN = 10 # meters
Y_DOMAIN = 10 # meters
THETA_DOMAIN = math.pi # radians


def np_bivariate_normal_pdf(domain, mean, variance):
  X = np.arange(-domain+mean, domain+mean, variance)
  Y = np.arange(-domain+mean, domain+mean, variance)
  X, Y = np.meshgrid(X, Y)
  R = np.sqrt(X**2 + Y**2)
  Z = ((1. / np.sqrt(2 * np.pi)) * np.exp(-.5 * R ** 2))
  print(X.shape, Y.shape, Z.shape, sum(sum(Z)))
  return X + mean, Y + mean, Z
 

def sci_bivariate_pdf(domain, mean, variance):
    cov = [[X_VARIANCE, 0, 0], [0, Y_VARIANCE, 0], [0, 0, THETA_VARIANCE]]
    rv = multivariate_normal([mean, mean, mean], cov)
    X = np.arange(-domain+mean, domain+mean, variance)
    Y = np.arange(-domain + mean, domain + mean, variance)
    T = np.arange(-THETA_DOMAIN, THETA_DOMAIN, THETA_VARIANCE)
    X, Y, T = np.meshgrid(X, Y, T)
    #X, Y, T = np.mgrid[*(-domain + mean:domain + mean:variance, -domain + mean:domain + mean:variance, -THETA_DOMAIN:THETA_DOMAIN:THETA_VARIANCE)]
    pos = np.stack((X, Y, T), axis=-1)
    print(X.shape, Y.shape, T.shape, pos.shape)
    Z = rv.pdf(pos)
    print(X.shape, Y.shape, Z.shape, sum(sum(sum(Z))))
    return X + mean, Y + mean, Z


def plt_plot_bivariate_normal_pdf(x, y, z, name='Graphy'):
  fig = plt.figure(figsize=(12, 6))
  ax = fig.gca(projection='3d')
  ax.plot_surface(x, y, z, 
                  cmap=cm.coolwarm,
                  linewidth=0, 
                  antialiased=True)
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_zlabel('z')
  plt.show()


class DetectionProb:
    def __init__(self, domains, means, variances):
        if not(np.array(domains).shape == np.array(means).shape and np.array(domains).shape == np.array(variances).shape):
            raise ValueError("domains, means, and variances must have the same shape")
        
        # Create model
        cov=[]
        for i, variance in enumerate(variances):
            z = np.zeros(len(variances))
            z[i] = variance
            cov.append(z)
        print(cov)
        self.rv = multivariate_normal(means, cov)

    def eval(self, x):
        return self.rv.pdf(x)

if __name__ == "__main__":
    #plt_plot_bivariate_normal_pdf(*np_bivariate_normal_pdf(X_DOMAIN, 0, X_VARIANCE), name="Robot Detection")
    #plt_plot_bivariate_normal_pdf(*sci_bivariate_pdf(X_DOMAIN, 0, X_VARIANCE), name="Robot Detection")

    det_prob = DetectionProb([X_DOMAIN, Y_DOMAIN, THETA_DOMAIN], [0,0,0], [X_VARIANCE, Y_VARIANCE, THETA_VARIANCE])
    