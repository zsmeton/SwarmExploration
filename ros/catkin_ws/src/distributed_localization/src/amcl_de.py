#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from amcl_msgs.msg import *
from amcl.srv import *

def get_particle_cloud():
    rospy.wait_for_service('get_particlecloud')
    try:
        get_particles = rospy.ServiceProxy('get_particlecloud', GetParticlecloud)
        resp1 = get_particles()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        print(get_particle_cloud())
        talker()
    except rospy.ROSInterruptException:
        pass
