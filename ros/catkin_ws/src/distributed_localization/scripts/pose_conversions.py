from geometry_msgs.msg import PoseArray, Pose2D, PoseWithCovarianceStamped, Pose, Quaternion
import tf.transformations
import math
import numpy as np

def pose_to_pose2d_rad(pose):
    """
    Turns pose into pose2d with radian yaw
    """
    # get x,y
    x = pose.position.x
    y = pose.position.y
    # get yaw
    # euler_from_quaternion -> (roll, pitch, yaw)
    yaw = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
    # append data to list
    return Pose2D(x,y,yaw)


def pose_to_pose2d_degree(pose):
    """
    Turns pose into pose2d with degree yaw
    """
    # get x,y
    x = pose.position.x
    y = pose.position.y
    # get yaw
    # euler_from_quaternion -> (roll, pitch, yaw)
    yaw = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]/math.pi * 180.0
    # append data to list
    return Pose2D(x, y, yaw)
    
def pose2d_rad_to_pose(pose):
    """
    Turns pose into pose2d with radian yaw
    """
    p = Pose()
    p.position.x = pose.x
    p.position.y = pose.y
    p.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose.theta))
    return p


def pose2d_rad_to_pose2d_degree(pose):
    """
    Turns pose2d with radian yaw to pose2d with degree yaw
    """
    return Pose2D(pose.x, pose.y, pose.theta / math.pi * 180.0)
    

def pose2d_degree_to_pose2d_rad(pose):
    """
    Turns pose2d with degree yaw to pose2d with radian yaw
    """
    return Pose2D(pose.x, pose.y, pose.theta / 180.0 * math.pi)
    
def pose2d_to_numpy(pose):
    """
    Turns a pose2d into a numpy array [x,y,theta]
    """
    return np.array([pose.x, pose.y, pose.theta])

def numpy_to_pose2d(array):
    return Pose2D(x=array[0], y=array[1], theta=array[2])