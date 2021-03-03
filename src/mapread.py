#!/usr/bin/env python3
#
#   readmap.py
#
#   Test reading the map.
#
import rospy
import numpy as np

from nav_msgs.msg       import OccupancyGrid


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('readmap')
    rospy.loginfo("Starting the map reading test...")

    # Grab a single instance of the /map topic.
    rospy.loginfo("Waiting for a map...")
    map = rospy.wait_for_message('/map', OccupancyGrid)
    rospy.loginfo("Got a map...")

    # Report.
    print("width:  ", map.info.width)
    print("height: ", map.info.height)

    width = map.info.width

    print("Contents: -1 = unknown, 0..100 = probability of wall")
    print("map[160,100]: (unknown) ", map.data[160*width + 100])
    print("map[160,292]: (wall)    ", map.data[160*width + 292])
    print("map[160,300]: (free)    ", map.data[160*width + 300])

    print(len(map.data))
    
