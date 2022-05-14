#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class transform_publisher():
	def __init__(self):
		rospy.init_node('transform_publisher')

		rospy.Subscriber('/smart/center_pose', PoseStamped, self.pose_cb, queue_size = 1)

		rospy.spin()

	def pose_cb(self, msg):
		pose = msg.pose.position
		orientation = msg.pose.orientation
		# 创建一个世界坐标系
		br = tf.TransformBroadcaster()
		br.sendTransform((pose.x, pose.y, pose.z),
							(orientation.x, orientation.y, orientation.z, orientation.w), #小车base_link的位置
							rospy.Time.now(),
							'base_link', 'world')  # 全局在world里面，world就是我们创建的内容


if __name__ == "__main__":
	try:
		transform_publisher()
	except:
		rospy.logwarn("cannot start transform publisher")