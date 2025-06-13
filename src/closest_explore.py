#!/usr/bin/env python3
#
#   set_goal.py
#
#   Test setting the goal.
#
import rospy
import threading
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf
from tf2_msgs.msg import TFMessage
import copy

#map = rospy.wait_for_message('/map', OccupancyGrid)

def set_goal(pub, goal, x, y, theta):

	goal.header.seq = 1.0
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"

	goal.pose.position.x = x
	goal.pose.position.y = y
	goal.pose.position.z = 0.0

	goal.pose.orientation.x = 0.0
	goal.pose.orientation.y = 0.0
	goal.pose.orientation.z = np.sin(theta/2)
	goal.pose.orientation.w = np.cos(theta/2)

	pub.publish(goal)

def map_callback(data):
	global map
	map = data
	print("subscribed")

def check_wall(m, n, radius):
	'''
	Check if there is a wall in the vicinity of the point
	'''
	global map
	M = map.info.width
	THRESHOLD = 19.6
	for i in range(-radius, radius + 1):
		for j in range(-radius, radius + 1):
			if map.data[(m+i)*M + n+j] > THRESHOLD:
				return False
	return True

#
#  Main Code
#
def main():
	THRESHOLD = 19.6

	# Prepare the node.
	rospy.init_node('closest_explore')

	# Create a publisher to send the joint values (joint_states).
	pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
	goal = PoseStamped()
	listener = tf.TransformListener()
	global map
	map = rospy.wait_for_message('/map', OccupancyGrid)
	sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
	old_data = 0



	while True:

		# Wait until connected.  You don't have to wait, but the first
		# messages might go out before the connection and hence be lost.
		rospy.sleep(3)

		min_distance = 100000
		min_goal = (0, 0)
		min_index = (0, 0)
		
		(trans,rot) = listener.lookupTransform('map', 'base_footprint',
										   rospy.Time(0))
		pos_x = trans[0]
		pos_y = trans[1]
		print("position:", pos_x, ",", pos_y)

		M = map.info.width
		N = map.info.height
		res = map.info.resolution
		origin_x = map.info.origin.position.x
		origin_y = map.info.origin.position.y

		print("Dimensions:", M, ",", N)
		print("origin:", origin_x, ",", origin_y)

		print(np.array_equal(map.data, old_data))
		old_data = copy.deepcopy(map.data)

		for m in range(M):
			for n in range(N):
				if map.data[m*M + n] == -1:
					if map.data[(m-1)*M + n] < THRESHOLD or \
					   map.data[(m+1)*M + n] < THRESHOLD or \
					   map.data[m*M + n + 1] < THRESHOLD or \
					   map.data[m*M + n - 1] < THRESHOLD:
						if True:
							x = origin_x + n * res
							y = origin_y + m * res
							distance = ((pos_x - x)**2 + (pos_y - y)**2)**(1/2)
							if distance < min_distance:
								min_distance = distance
								min_goal = (x, y)
								min_index = (m, n)
		print("min_index", min_index)

		print("min_goal:", min_goal)				   
		set_goal(pub, goal, *min_goal, 0)

if __name__ == "__main__":
	main()
	



