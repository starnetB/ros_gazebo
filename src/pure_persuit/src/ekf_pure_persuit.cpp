#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <styx_msgs/Lane.h>
#include <styx_msgs/Waypoint.h>

class EKF_PurePersuit{
public:
    EKF_PurePersuit():poseflag(false),laneflag(false){}
    ~EKF_PurePersuit(){}

    void ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void ekf_lane_cb(const styx_msgs::Lane::ConstPtr& lane);

private:
    geometry_msgs::PoseStamped pose;
    bool poseflag;

    styx_msgs::Lane lane;
    bool laneflag;
};

void EKF_PurePersuit::ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    this->pose.header=pose->header;
    this->pose.pose.orientation.w=pose->pose.orientation.w;
    this->pose.pose.orientation.x=pose->pose.orientation.x;
    this->pose.pose.orientation.y=pose->pose.orientation.y;
    this->pose.pose.orientation.z=pose->pose.orientation.z;
    this->pose.pose.position.x=pose->pose.position.x;
    this->pose.pose.position.y=pose->pose.position.y;
    this->pose.pose.position.z=pose->pose.position.z;
    this->poseflag=true; 
}

void  EKF_PurePersuit::ekf_lane_cb(const styx_msgs::Lane::ConstPtr& lane)
{
    this->lane.header=lane->header;
    this->lane.waypoints=lane->waypoints;
    this->lane.
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"ekf_pure_persuit");

    ros::NodeHandle nh;

    EKF_PurePersuit ekf_pure;
    ros::Subscriber ekf_rear_pose_sub=nh.subscribe("/smart/rear_pose",1,&EKF_PurePersuit::ekf_pose_cb,&ekf_pure);

    ros::Subscriber ekf_waypoints_sub=nh.subscribe("/ekf_final_waypoints",1,&EKF_PurePersuit::ekf_lane_cb,&ekf_pure);

    ros::Publisher ekf_twist_pub=nh.advertise<geometry_msgs::Twist>("/smart/cmd_vel",1);



}