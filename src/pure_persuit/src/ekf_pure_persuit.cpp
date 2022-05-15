#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <styx_msgs/Lane.h>
#include <styx_msgs/Waypoint.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class EKF_PurePersuit{
public:
    EKF_PurePersuit():poseflag(false),laneflag(false){
        this->horizon=6.0;
    }
    ~EKF_PurePersuit(){}

    void ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void ekf_lane_cb(const styx_msgs::Lane::ConstPtr& lane);

    bool get_laneflag();
    bool get_poseflag();

    void set_horizon(double horizon);
    double get_horizon();
    geometry_msgs::Twist calcuateTwistCommand();
private:
    geometry_msgs::PoseStamped pose;
    bool poseflag;

    styx_msgs::Lane lane;
    bool laneflag;

    double horizon;

    void T_hpi_hpi(double& alhpa);
};

void EKF_PurePersuit::T_hpi_hpi(double& alpha)
{
    double pi=3.1415926;
    double h_pi=pi/2;
    if(alpha>h_pi)
    {
        alpha=alpha-pi;
        this->T_hpi_hpi(alpha);
    }
    if(alpha<-h_pi)
    {
       alpha=alpha+pi;
        this->T_hpi_hpi(alpha);
    }
}

double EKF_PurePersuit::get_horizon()
{
    return this->horizon;
}

void EKF_PurePersuit::set_horizon(double horizon)
{
    this->horizon=horizon;
}

geometry_msgs::Twist EKF_PurePersuit::calcuateTwistCommand()
{
    //double lad=sqrt(pose.pose.position.x*pose.pose.orientation.x+pose.pose.position.y*pose.pose.position.y);
    double lad=0.0;
    int targetIndex=this->lane.waypoints.size()-1;
    for(std::size_t i=0;i<lane.waypoints.size()-1;i++)
    {
        double current_x=this->lane.waypoints[i].pose.pose.position.x;
        double current_y=this->lane.waypoints[i].pose.pose.position.y;
        double next_x=this->lane.waypoints[i+1].pose.pose.position.x;
        double next_y=this->lane.waypoints[i+1].pose.pose.position.y;
        lad=lad+sqrt((next_x-current_x)*(next_x-current_x)+(next_y-current_y)*(next_y-current_y));
        //std::cout<<lad<<std::endl;
        if(lad>this->horizon)
        {
            targetIndex=i+1;
            break;
        }
    }

    //找到目标点
    styx_msgs::Waypoint targetWaypoint=this->lane.waypoints[targetIndex];
    double targetSpeed=this->lane.waypoints[targetIndex].twist.twist.linear.x;
    //double targetSpeed=1;
    //目标点
    double targetX=targetWaypoint.pose.pose.position.x;
    double targetY=targetWaypoint.pose.pose.position.y;

    //当前点
    double currentX=this->pose.pose.position.x;
    double currentY=this->pose.pose.position.y;


    Eigen::Quaterniond qua((double)this->pose.pose.orientation.w,
                           (double)this->pose.pose.orientation.x,
                           (double)this->pose.pose.orientation.y,
                           (double)this->pose.pose.orientation.z);

    Eigen::Vector3d eulr=qua.matrix().eulerAngles(2,1,0);

    double yaw=eulr[0];
   
    //alpha
    double alpha=atan2(targetY-currentY,targetX-currentX)-yaw;
 
    this->T_hpi_hpi(alpha);
    double l=sqrt((targetY-currentY)*(targetY-currentY)+(targetX-currentX)*(targetX-currentX));

    geometry_msgs::Twist twist;
   
    if(l>2.5)
    {   //std::cout<<"alpha:"<<alpha<<std::endl;
        double theta=atan(2*1.868*sin(alpha)/l);
        
        twist.linear.x=targetSpeed;
        twist.angular.z=theta;
    }else{

        twist.linear.x=0;
        twist.angular.z=0;
    }
    return twist;
}
bool EKF_PurePersuit::get_laneflag()
{
    return this->laneflag;
}

bool EKF_PurePersuit::get_poseflag()
{
    return this->poseflag;
}
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
    this->laneflag=true;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"ekf_pure_persuit");

    ros::NodeHandle nh;

    EKF_PurePersuit ekf_pure;

    ekf_pure.set_horizon(6.0);

    ros::Subscriber ekf_rear_pose_sub=nh.subscribe("/smart/rear_pose",1,&EKF_PurePersuit::ekf_pose_cb,&ekf_pure);

    ros::Subscriber ekf_waypoints_sub=nh.subscribe("/ekf_final_waypoints",1,&EKF_PurePersuit::ekf_lane_cb,&ekf_pure);

    ros::Publisher ekf_twist_pub=nh.advertise<geometry_msgs::Twist>("/smart/cmd_vel",1);

    ros::Rate rate(20);

    while(ros::ok())
    {
        if(ekf_pure.get_laneflag() && ekf_pure.get_poseflag())
        {
            auto cmd=ekf_pure.calcuateTwistCommand();
            ekf_twist_pub.publish(cmd); 
        }
        ros::spinOnce();
        rate.sleep();
    }

}