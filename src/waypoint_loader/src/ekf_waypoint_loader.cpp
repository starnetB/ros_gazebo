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

class Ekf_WaypointLoader{
public:
    Ekf_WaypointLoader(int argc,char **argv){
        ros::init(argc,argv,"ekf_waypoint_loader");
   
        ros::NodeHandle n;
     
        pub=n.advertise<styx_msgs::Lane>("/ekf_base_waypoints",1);
        pub_path=n.advertise<nav_msgs::Path>("/ekf_base_path",1);
      
        base_path=new nav_msgs::Path();

        ekf_new_waypoint_loader(n,"/ekf_waypoint_loader/ekf_path");
        

        ros::Rate loop_rate(20);
        
        int count=0;
        while (ros::ok())
        {
            ekf_publisher();
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }
        
        
    }

    ~Ekf_WaypointLoader(){
        delete base_path;
    }

    bool check_files(std::ifstream& outfile);
    
    bool ekf_new_waypoint_loader(ros::NodeHandle& nh,const std::string& ekf_path_param);
    
    bool ekf_loader_waypoint(std::string& ekf_file_path);

    std::vector<float> Quaternion_from_yaw(float yaw);

    bool decelerate();

    float kmph2mps(float velocity);

    void ekf_publisher();

private:
    ros::Publisher pub;
    ros::Publisher pub_path;

    nav_msgs::Path *base_path;

    std::vector<styx_msgs::Waypoint> waypoints;

    std::vector<float> velocity_l;
    std::vector<float> timeStamps_l;

    
};

std::vector<float> Ekf_WaypointLoader::Quaternion_from_yaw(float yaw)
{
    Eigen::AngleAxisf yawAngle(yaw,Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf quaternion;
    quaternion=yawAngle;
    std::vector<float> Rvector;
    Rvector.push_back(quaternion.x());
    Rvector.push_back(quaternion.y());
    Rvector.push_back(quaternion.z());
    Rvector.push_back(quaternion.w());
    return Rvector;
}

void Ekf_WaypointLoader::ekf_publisher()
{
    styx_msgs::Lane Lane;
    Lane.header.frame_id="/world";
    Lane.header.stamp=ros::Time::now();
    Lane.waypoints=waypoints;

    this->pub.publish(Lane);
    this->pub_path.publish(*base_path);
}

bool Ekf_WaypointLoader::check_files(std::ifstream& infile)
{
    if(!infile.is_open()){
        return false;
    }else{
        return true;
    }
}

float Ekf_WaypointLoader::kmph2mps(float velocity)
{
    return velocity*1000/(3600);
}

bool Ekf_WaypointLoader::ekf_new_waypoint_loader( ros::NodeHandle& nh,const std::string& ekf_path_param)
{
   
    std::string ekf_file_path;
    ROS_INFO("%s",ekf_path_param.c_str());
    if(nh.getParam(ekf_path_param,ekf_file_path))
    {
        ROS_INFO("%s",ekf_file_path.c_str());
        if(ekf_loader_waypoint(ekf_file_path))
        {
            ROS_INFO("load ekf data");
           
        }else{
            ROS_INFO("failed to open the ekf data file!");
            return false;
        }
    }   
    return true;
}

bool Ekf_WaypointLoader::decelerate()
{
    return true;
}

//padding velocity_l
//padding timeStamps_l
//padding  waypoints
bool Ekf_WaypointLoader::ekf_loader_waypoint(std::string& ekf_file_path)
{
    base_path->header.frame_id="world";
    std::ifstream infile;
    infile.open(ekf_file_path.c_str(),std::ifstream::in);

    if(!check_files(infile))
    {
        std::cerr<<"Cannot open input file!,infile"<<std::endl;
        return false;
    }


    std::string Line;
    float ox=0,oy=0;
    while(getline(infile,Line))
    {
        std::istringstream row(Line);
        float x,y,vx,vy,v,timestamp,yaw;
        styx_msgs::Waypoint p;
        row>>x;
        row>>y;
        row>>vx;
        row>>vy;
        
        v=sqrt(vx*vx+vy*vy);
        row>>timestamp;
        yaw=atan2(vy,vx);
        auto q=Quaternion_from_yaw(yaw);
        p.pose.pose.position.x=x;
        p.pose.pose.position.y=y;
        p.pose.pose.orientation.x=q[0];
        p.pose.pose.orientation.y=q[1];
        p.pose.pose.orientation.z=q[2];
        p.pose.pose.orientation.w=q[3];
        p.twist.twist.linear.x=v;
        p.forward=true;
        waypoints.push_back(p);

        geometry_msgs::PoseStamped path_element;
        path_element.pose.position.x=p.pose.pose.position.x;
        path_element.pose.position.y=p.pose.pose.position.y;
        //rviz
        base_path->poses.push_back(path_element);

        ox=x;
        oy=y;
    }
    infile.close();

    return true;
}


int main(int argc,char **argv)
{
    Ekf_WaypointLoader(argc,argv);
}