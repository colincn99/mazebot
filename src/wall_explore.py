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
import random

THRESHOLD = 19.6
COST_COEF = 0

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

def costmap_callback(data):
	global costmap
	costmap = data

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


	# Prepare the node.
	rospy.init_node('closest_explore')

	# Create a publisher to send the joint values (joint_states).
	pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
	goal = PoseStamped()
	listener = tf.TransformListener()
	global map
	global global_costmap
	map = rospy.wait_for_message('/map', OccupancyGrid)
	costmap = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid)
	sub_map = rospy.Subscriber('/map', OccupancyGrid, map_callback)
	sub_costmap = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costmap_callback)
	old_data = 0

	#rospy.sleep(1)
	#set_goal(pub, goal, -2.5, 1, 0)



	while True:

		# Wait until connected.  You don't have to wait, but the first
		# messages might go out before the connection and hence be lost.
		rospy.sleep(2)

		min_cost = 100000
		min_goal = (0, 0)
		min_index = (0, 0)
		min_direction = [0, 0]
		
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
		print("Costmap Dimensions:", costmap.info.width, ",", costmap.info.height)
		print("origin:", origin_x, ",", origin_y)

		print(np.array_equal(map.data, old_data))
		old_data = copy.deepcopy(map.data)

		for m in range(3,M-3):
			for n in range(3,N-3):
				if map.data[m*M + n] < THRESHOLD and map.data[m*M + n] != -1:
					direction = [0, 0] #from free to unknown (y,x)
					frontier = False

					if map.data[(m-1)*M + n] == -1: #down
						frontier = True
						direction[0] = direction[0] - 1
					if map.data[(m+1)*M + n] == -1: #up
						frontier = True
						direction[0] = direction[0] + 1
					if map.data[m*M + n + 1] == -1: #right
						frontier = True
						direction[1] = direction[1] + 1
					if map.data[m*M + n - 1] == -1: #left
						frontier = True
						direction[1] = direction[1] - 1

					if frontier:
						if check_wall(m,n,3):
						
							x = origin_x + n * res
							y = origin_y + m * res
							distance = ((pos_x - x)**2 + (pos_y - y)**2)**(1/2)
							cost = distance + COST_COEF * costmap.data[m*M + n]
							if cost < min_cost:
								min_cost = cost
								min_goal = (x, y)
								min_index = (m, n)
								min_direction = direction
		
		print("min_index", min_index)
		print("min_goal:", min_goal)
		theta = 0
		if min_direction!= [0, 0]:
			theta = np.arctan2(*min_direction)
		print("direction:", min_direction)
		print("theta:", theta)		   
		set_goal(pub, goal, *min_goal, theta + random.uniform(-np.pi/4, np.pi/4))

if __name__ == "__main__":
	main()
	



