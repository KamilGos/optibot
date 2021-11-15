#!/usr/bin/env python  
import roslib
roslib.load_manifest('optibot_startup')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('map_to_camera')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    print("ready")
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/camera_odom_frame",
                         "/map")
        rate.sleep()
