#!/usr/bin/env python

import os
import numpy as np
from math import cos, sin

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates

import tf
import rospy

class true_state_pub:
	def __init__(self):
		rospy.init_node('true_state_pub', log_level=rospy.DEBUG)

		self.rear_pose_pub = rospy.Publisher('/racebot/true_state/rear_pose', PoseStamped, queue_size = 1)
		self.center_pose_pub = rospy.Publisher('/racebot/true_state/center_pose', PoseStamped, queue_size = 1)
		self.vel_pub = rospy.Publisher('/racebot/true_state/velocity', TwistStamped, queue_size = 1)
		self.true_odom_pub = rospy.Publisher('/racebot/true_state/odom', Odometry, queue_size = 1)

		rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size = 1)

		rospy.spin()

	def model_cb(self,data):
		try:
			vehicle_model_index = data.name.index("racebot")
		except:
			return
		vehicle_position = data.pose[vehicle_model_index]
		vehicle_velocity = data.twist[vehicle_model_index]
		orientation = vehicle_position.orientation
		(_, _, yaw) = tf.transformations.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
		time_stamp = rospy.Time.now()

		# vehicle center position
		center_pose = PoseStamped()
		center_pose.header.frame_id = 'world'
		center_pose.header.stamp = time_stamp
		center_pose.pose.position = vehicle_position.position
		center_pose.pose.orientation = vehicle_position.orientation
		self.center_pose_pub.publish(center_pose)

		# vehicle rear axle position
		rear_pose = PoseStamped()
		rear_pose.header.frame_id = 'world'
		rear_pose.header.stamp = time_stamp
		center_x = vehicle_position.position.x
		center_y = vehicle_position.position.y
		rear_x = center_x - cos(yaw) * 0.13
		rear_y = center_y - sin(yaw) * 0.13
		rear_pose.pose.position.x = rear_x
		rear_pose.pose.position.y = rear_y
		rear_pose.pose.orientation = vehicle_position.orientation
		self.rear_pose_pub.publish(rear_pose)

		# vehicle velocity
		velocity = TwistStamped()
		velocity.header.frame_id = ''
		velocity.header.stamp = time_stamp
		velocity.twist.linear = vehicle_velocity.linear
		velocity.twist.angular = vehicle_velocity.angular
		self.vel_pub.publish(velocity)

		# true odom
		true_odom = Odometry()
		true_odom.header.frame_id = 'world'
		true_odom.header.stamp = time_stamp
		true_odom.pose.pose = rear_pose.pose
		true_odom.twist.twist = velocity.twist
		self.true_odom_pub.publish(true_odom)

if __name__ == "__main__":
    
	try:
		true_state_pub()
	except:
		rospy.logwarn("cannot start vehicle pose and velocity updater updater")