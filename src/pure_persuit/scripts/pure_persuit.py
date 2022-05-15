#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist

from styx_msgs.msg import Lane, Waypoint

from gazebo_msgs.msg import ModelStates

import tf
import rospy

# 每次HORIZON，就是在PurePersuit的选取的目标点与当前点的距离
HORIZON = 6.0

class PurePersuit:
	def __init__(self):
		# 初始化节点
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)
		# 读取后轮的位置
		rospy.Subscriber('/smart/rear_pose', PoseStamped, self.pose_cb, queue_size = 1)
		# 读取小车的速度，但这个没啥用
		rospy.Subscriber('/smart/velocity', TwistStamped, self.vel_cb, queue_size = 1)
		# 读取局部路径点
		#rospy.Subscriber('/final_waypoints', Lane, self.lane_cb, queue_size = 1)
		rospy.Subscriber('/ekf_final_waypoints', Lane, self.lane_cb, queue_size = 1)

		# 命令发送器
		self.twist_pub = rospy.Publisher('/smart/cmd_vel', Twist, queue_size = 1)

		self.currentPose = None
		self.currentVelocity = None
		self.currentWaypoints = None

		self.loop()

	def loop(self):
		rate = rospy.Rate(20)
		rospy.logwarn("pure persuit starts")
		while not rospy.is_shutdown():
			# 如果当前速度有了，当前位置有了，局部路径有了，那么我们就开始计算追踪算法
			if self.currentPose and self.currentVelocity and self.currentWaypoints:
				twistCommand = self.calculateTwistCommand()
				self.twist_pub.publish(twistCommand)
			rate.sleep()

	# 把后轮轴中心的位置作为当前点的位置
	def pose_cb(self,data):
		self.currentPose = data

	def vel_cb(self,data):
		self.currentVelocity = data

	def lane_cb(self,data):
		self.currentWaypoints = data

	def calculateTwistCommand(self):
		# 距离累加
		lad = 0.0 #look ahead distance accumulator
		# 目标点索引
		targetIndex = len(self.currentWaypoints.waypoints) - 1
		for i in range(len(self.currentWaypoints.waypoints)):
			# 如果不是最后一个点
			# 在局部路径里面找到一个点，使得这个点与当前点的距离恰好基本等于6,比6大一点点，如果没法比HORIZON打，那就是局部路径中的最后一个点
			# 也就是吧局部路径分成好几份，每份大小是6m，但是最后一份应该比6小
			# 但是局部路径也是实时更新的，所有只有到末尾的时候，才会出现最后一份比6小的情况
			if((i+1) < len(self.currentWaypoints.waypoints)):
				this_x = self.currentWaypoints.waypoints[i].pose.pose.position.x
				this_y = self.currentWaypoints.waypoints[i].pose.pose.position.y
				next_x = self.currentWaypoints.waypoints[i+1].pose.pose.position.x
				next_y = self.currentWaypoints.waypoints[i+1].pose.pose.position.y
				lad = lad + math.hypot(next_x - this_x, next_y - this_y)
				# 
				if(lad > HORIZON):
					targetIndex = i+1
					break

		# 找到目标点
		targetWaypoint = self.currentWaypoints.waypoints[targetIndex]
		# 找到目标点的速度
		targetSpeed = self.currentWaypoints.waypoints[0].twist.twist.linear.x
	    
		# HORIZON目标点的位置
		targetX = targetWaypoint.pose.pose.position.x
		targetY = targetWaypoint.pose.pose.position.y		
		# 当前点的位置
		currentX = self.currentPose.pose.position.x
		currentY = self.currentPose.pose.position.y
		#get vehicle yaw angle
		# 当前点幅角的四元数
		quanternion = (self.currentPose.pose.orientation.x, self.currentPose.pose.orientation.y, self.currentPose.pose.orientation.z, self.currentPose.pose.orientation.w)
		# 转成欧拉角
		euler = tf.transformations.euler_from_quaternion(quanternion)
		# 把z后的yaw提炼出来
		yaw = euler[2]
		#get angle difference
		# 算出alpha,差值减去当前yaw就是相对幅角
		alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw
		# 算出两者之间距离
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
		if(l > 0.5):
			# 如果两者之间的距离比0.5大，那么根据我们的要求计算 转角
			theta = math.atan(2 * 1.868 * math.sin(alpha) / l)
			# #get twist command
			twistCmd = Twist()
			twistCmd.linear.x = targetSpeed  # 这个速度应该和当前点的速度是一样的
			twistCmd.angular.z = theta  # 设置转角
		else:
			# 小于0.5就停下来
			twistCmd = Twist()
			twistCmd.linear.x = 0
			twistCmd.angular.z = 0

		return twistCmd

# 我们不断的实时更新局部路径，根据这个局部路径，我们实时的截出6m进行规划
# 到最后不满6m了，那么我们只能减小距离
# 最后停下，然后每个过程我们都采用6m进行速度和角度规划，但这个6m一直在变，因此角度和速度也会跟着变

if __name__ == '__main__':
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start motion control node.')

