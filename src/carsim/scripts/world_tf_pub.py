#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class world_tf_pub():
	def __init__(self):
		rospy.init_node('world_tf_pub')

		self.last_published_timestamp = rospy.Time.from_sec(0)

		rospy.Subscriber('/racebot/true_state/center_pose', PoseStamped, self.pose_cb, queue_size = 1000)

		rospy.spin()

	def pose_cb(self, msg):
		pose = msg.pose.position
		orientation = msg.pose.orientation
		br = tf.TransformBroadcaster()
		if self.last_published_timestamp != rospy.Time.now():
			self.last_published_timestamp = rospy.Time.now()
			br.sendTransform((pose.x, pose.y, pose.z),
							(orientation.x, orientation.y, orientation.z, orientation.w),
							self.last_published_timestamp,
							'base_footprint', 'world')

if __name__ == "__main__":
	try:
		world_tf_pub()
	except:
		rospy.logwarn("cannot start transform publisher")