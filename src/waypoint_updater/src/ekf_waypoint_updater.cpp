#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "styx_msgs/Lane.h"
#include "styx_msgs/Waypoint.h"
#include "nav_msgs/Path.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


class EKF_WayPointUpdater
{
public: 
    EKF_WayPointUpdater():poseflag(false),laneflag(false),kdtreeflag(false){}
    ~EKF_WayPointUpdater(){}
    
    void ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void ekf_waypoints_cb(const styx_msgs::Lane::ConstPtr& lane);

    bool get_poseflag();
    bool get_laneflag();
    bool get_kdtreeflag();

    int get_closest_waypoint_idx();
    void publish_waypoint(ros::Publisher& final_waypoints_pub,ros::Publisher final_path_pub,int idx);

private:
    geometry_msgs::PoseStamped pose;
    bool poseflag;

    styx_msgs::Lane lane;
    bool laneflag;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    bool kdtreeflag;


};

bool EKF_WayPointUpdater::get_poseflag()
{
    return this->poseflag;
}

bool EKF_WayPointUpdater::get_kdtreeflag()
{
    return this->kdtreeflag;
}

bool EKF_WayPointUpdater::get_laneflag()
{
    return this->laneflag;
}

void EKF_WayPointUpdater::ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
   
    this->pose.header.frame_id=pose->header.frame_id;
    this->pose.pose.orientation.w=pose->pose.orientation.w;
    this->pose.pose.orientation.x=pose->pose.orientation.x;
    this->pose.pose.orientation.y=pose->pose.orientation.y;
    this->pose.pose.orientation.z=pose->pose.orientation.z;
    this->pose.pose.position.x=pose->pose.position.x;
    this->pose.pose.position.y=pose->pose.position.y;
    this->pose.pose.position.z=pose->pose.position.z;
    this->poseflag=true;
}

void EKF_WayPointUpdater::ekf_waypoints_cb(const styx_msgs::Lane::ConstPtr& ekf_lane)
{
    if(!laneflag){
        this->lane.header=ekf_lane->header;
        this->lane.waypoints=ekf_lane->waypoints;
        this->laneflag=true;
    }
    if(!this->kdtreeflag){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


        //Generate pointcloud data;
        cloud->width=ekf_lane->waypoints.size();
        cloud->height=1;
        cloud->points.resize(cloud->width*cloud->height);

        for(std::size_t i=0;i<cloud->points.size();++i)
        {
            cloud->points[i].x=ekf_lane->waypoints[i].pose.pose.position.x;
            cloud->points[i].y=ekf_lane->waypoints[i].pose.pose.position.y;
            cloud->points[i].z=0;
        }

        kdtree.setInputCloud(cloud);
        kdtreeflag=true;
    }
}

int EKF_WayPointUpdater::get_closest_waypoint_idx()
{
    int K=1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquareDistance(K);

    pcl::PointXYZ searchPoint;

    searchPoint.x=this->pose.pose.position.x;
    searchPoint.y=this->pose.pose.position.y;
    searchPoint.z=0;

    if(!(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquareDistance)>0))
    {
        ROS_INFO("kdtree build wrong！\n");
    }

    return pointIdxNKNSearch[0];
}

void EKF_WayPointUpdater::publish_waypoint(ros::Publisher& final_waypoints_pub,ros::Publisher final_path_pub,int idx)
{   
    styx_msgs::Lane lane;
    lane.header=this->lane.header;
    for(std::size_t i=0;i<30;i++)
    {
        lane.waypoints.push_back(this->lane.waypoints[idx+i]);
    }

    nav_msgs::Path path;
    path.header.frame_id="/world";

    for(std::size_t i=0;i<lane.waypoints.size();i++)
    {
        geometry_msgs::PoseStamped path_element;
        path_element.pose.position.x=lane.waypoints[i].pose.pose.position.x;
        path_element.pose.position.y=lane.waypoints[i].pose.pose.position.y;
        path.poses.push_back(path_element);
    }

    final_waypoints_pub.publish(lane);
    final_path_pub.publish(path);

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"ekf_local_waypoint");

    ros::NodeHandle nh;

    EKF_WayPointUpdater ekf_waypointupdater;

    ros::Subscriber rear_pose=nh.subscribe("/smart/rear_pose",1,&EKF_WayPointUpdater::ekf_pose_cb,&ekf_waypointupdater);
    ros::Subscriber waypoints=nh.subscribe("/ekf_base_waypoints",1,&EKF_WayPointUpdater::ekf_waypoints_cb,&ekf_waypointupdater);

    ros::Publisher final_waypoints_pub=nh.advertise<styx_msgs::Lane>("ekf_final_waypoints",1);
    ros::Publisher final_path_pub=nh.advertise<nav_msgs::Path>("ekf_final_path",1);

    ros::Rate rate(20);

    int count=0;
    while (ros::ok())
    {
        if(ekf_waypointupdater.get_laneflag() &&ekf_waypointupdater.get_kdtreeflag() && ekf_waypointupdater.get_poseflag())
        {
            int idx=ekf_waypointupdater.get_closest_waypoint_idx();
            ekf_waypointupdater.publish_waypoint(final_waypoints_pub,final_path_pub,idx);
        }
        ros::spinOnce();
        rate.sleep();
        ++count;
    }

    return 0;
}