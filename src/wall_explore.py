#!/usr/bin/env python3
#
#   set_goal.py
#
#   Test setting the goal.
#
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf
from tf2_msgs.msg import TFMessage
import random

THRESHOLD = 19.6
WALL_COEF = -0.002 #negative attacts to walls, positive repels from walls
STUCK_THRESHOLD = 0.2

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

	pos_history = []

	while True:

		# Wait until connected.  You don't have to wait, but the first
		# messages might go out before the connection and hence be lost.
		rospy.sleep(4)

		min_cost = 100000
		min_goal = (0, 0)
		min_index = (0, 0)
		min_direction = [0, 0]
		any_goal = False
		
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
		
		pos_history.append((pos_x, pos_y))
		if len(pos_history) > 4:
			pos_history.pop(0)
			if all(((pos[0] - pos_history[0][0])**2 + (pos[1] - pos_history[0][1])**2)**(1/2) \
				     < STUCK_THRESHOLD for pos in pos_history):
				print("robot stuck")
				set_goal(pub, goal, pos_x + random.uniform(-1,1), pos_y + random.uniform(-1,1), random.uniform(0, 2*np.pi))
				continue


		for m in range(3,M-3):
			for n in range(3,N-3):
				if map.data[m*M + n] < THRESHOLD and map.data[m*M + n] != -1: #if free
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
							any_goal = True
							x = origin_x + n * res
							y = origin_y + m * res
							distance = ((pos_x - x)**2 + (pos_y - y)**2)**(1/2)
							cost = distance + WALL_COEF * costmap.data[m*M + n]
							if cost < min_cost:
								min_cost = cost
								min_goal = (x, y)
								min_index = (m, n)
								min_direction = direction
		
		if not any_goal:
			print('No more to explore')
			break

		#print("min_index", min_index)
		print("position goal:", min_goal)
		theta = 0
		if min_direction!= [0, 0]:
			theta = np.arctan2(*min_direction) + np.pi/8*random.uniform(-1,1)
		#print("direction:", min_direction)
		print("theta:", theta)		   
		set_goal(pub, goal, *min_goal, theta)

if __name__ == "__main__":
	main()
	



