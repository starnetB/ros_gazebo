#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion, PoseStamped

from styx_msgs.msg import Lane, Waypoint

from nav_msgs.msg import Path

import tf
import rospy

CSV_HEADER = ['x', 'y', 'yaw']
MAX_DECEL = 1.0


class WaypointLoader(object):

    def __init__(self):
        # 初始化节点
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        # 发布base_waypoint路径点
        # waypoint
        # geometry_msgs/PoseStamped pose  位置与旋转量
        # geometry_msgs/TwistStamped twist
        # Lane waypoints[]
        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1, latch=True)
        # 和Lane差不多，但这个是rviz需要预订的信号
        self.pub_path = rospy.Publisher('/base_path', Path, queue_size=1, latch=True)
        
        # 从参数服务器里面获取额定速度，这个速度在launch文件里已经设定了
        self.velocity = self.kmph2mps(rospy.get_param('~velocity'))
        # 路径的path在哪里
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    # 这个函数里面获取路标点
    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            waypoints, base_path = self.load_waypoints(path)
            self.publish(waypoints, base_path)# 同时发布path与waypoints
            rospy.loginfo('Waypoint Loded')
        else:
            rospy.logerr('%s is not a file', path)

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def load_waypoints(self, fname):
        # waypoints列表
        waypoints = []
        # rviz路径
        base_path = Path()
        # rviz确定world
        base_path.header.frame_id = 'world'
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            # 读出csv文件
            for wp in reader:
                # 读出一个waypoint点
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                # 位置的副角，也就是汽车的朝向
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                # 设定额定速度
                p.twist.twist.linear.x = float(self.velocity)
                p.forward = True
                waypoints.append(p)
                # 位置戳
                path_element = PoseStamped()
                path_element.pose.position.x = p.pose.pose.position.x
                path_element.pose.position.y = p.pose.pose.position.y
                # rviz专用
                base_path.poses.append(path_element)

                
        waypoints = self.decelerate(waypoints)
        return waypoints,base_path

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:  #反过来，根据你的当前位置和目标点之距离慢慢衰减速度
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints, base_path):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.pub.publish(lane)
        self.pub_path.publish(base_path)


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
