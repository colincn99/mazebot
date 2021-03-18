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

#
#  Main Code
#
def main():
	# Prepare the node.
	rospy.init_node('set_goal')

	# Create a publisher to send the joint values (joint_states).
	pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
	goal = PoseStamped()

	while True:

		# Wait until connected.  You don't have to wait, but the first
		# messages might go out before the connection and hence be lost.
		rospy.sleep(0.25)

		user_input = input("Enter x_position, y_position, orientation\n")
		if user_input == "exit":
			break
		try:
			x, y, theta = user_input.split(",")
			x = float(x)
			y = float(y)
			theta = float(theta)
		except:
			print("incorrect format")
			continue
		set_goal(pub, goal, x, y, theta)

if __name__ == "__main__":
	main()
	



