#!/usr/bin/env python3
#
#   readmap.py
#
#   Test reading the map.
#
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from nav_msgs.msg       import OccupancyGrid

def showgrid(map):

    M = map.info.width
    N = map.info.height

    # Close the old figure.
    plt.close()

    fig = plt.figure()
    ax = plt.axes()

    # turn off axis labels
    ax.axis("off")

    for m in range(M+1):
        ax.axhline(m, lw=1, color='b', zorder = 1)
    for n in range(N+1):
        ax.axvline(n, lw=1, color='b', zorder = 1)

    color = np.ones((M,N,3))
    for m in range(M):
        for n in range(N):
            if map.data[m*width + n] == -1:
                color[m,n,0:3] = np.array([1.0, 1.0, 0])  #Yellow
            else:
                # Shades of pink/purple/blue. Yellow means impossible
                p = 1-(map.data[m*width + n])/100
                #rlevel = (1.0 - p)
                #glevel = (1.0 - p)
                #blevel = (1.0 - p) #if p > 0 else 0
                #color[m,n,0:3] = np.array([rlevel, glevel, blevel])
                color[m,n,0:3] = np.array([p, p, p]) #white if free space, black if wall

    # Draw the boxes
    ax.imshow(color, aspect='equal', interpolation='none', extent=[0, N, 0, M], zorder=0)

    # Force the figure to pop up
    plt.pause(0.001)


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

    showgrid(map)
    input('Hit return to continue')

    
