#!/usr/bin/env python  
import roslib
from tf.transformations import quaternion_from_euler
roslib.load_manifest('optibot_startup')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('base_to_laser')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    print("ready")
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0,0,3.141592),
                         rospy.Time.now(),
                         "/laser",
                         "/base_link")
        rate.sleep()
