#!/usr/bin/env python3
#
#   posread.py
#
#   Test reading the position.
#
import rospy
import tf
import numpy as np

from tf2_msgs.msg       import TFMessage


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('posread')
    rospy.loginfo("Starting the position (transform) reading test...")

    # Set up the listener.
    listener = tf.TransformListener()
    
    # Wait until TF has collected enough information at startup.
    rospy.sleep(2)


    # Grab a single time.
    rospy.loginfo("Grabbing a transform...")
    (trans,rot) = listener.lookupTransform('map', 'base_footprint',
                                           rospy.Time(0))
    rospy.loginfo("Got a transform...")
    
    # Report.
    print(trans)
    print(rot)
