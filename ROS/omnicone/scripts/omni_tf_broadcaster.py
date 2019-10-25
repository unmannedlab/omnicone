#!/usr/bin/env python

import roslib
import rospy

import tf
import turtlesim.msg

def Omni_TF_run():
    while(not rospy.is_shutdown()):
        br = tf.TransformBroadcaster()
        br.sendTransform((0,0,0),(0,0,1,0),
                         rospy.Time.now(),
                         "local_origin_ned",
                         "fcu")

if __name__ == '__main__':
    rospy.init_node('omni_tf_broadcaster')
    Omni_TF_run()
