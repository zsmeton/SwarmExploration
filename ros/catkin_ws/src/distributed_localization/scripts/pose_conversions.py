from geometry_msgs.msg import PoseArray, Pose2D, PoseWithCovarianceStamped
import tf.transformations
import math

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
    return Pose2D(x,y,yaw)


def pose2d_rad_to_pose2d_degree(pose):
    """
    Turns pose2d with radian yaw to pose2d with degree yaw
    """
    return Pose2D(pose.x, pose.y, pose.theta / math.pi * 180.0)
    

def pose2d_degree_to_pose2d_rad(pose):
    """
    Turns pose2d with degree yaw to pose2d with radian yaw
    """
    return Pose2D(pose.x, pose.y, pose.theta/math.pi * 180.0)