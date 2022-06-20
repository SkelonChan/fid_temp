#include <ros/ros.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <nav_msgs/Odometry.h>//gps_odom
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>//fix
#include <geometry_msgs/TwistStamped.h>//vel
#include <geometry_msgs/QuaternionStamped.h>//heading

double x_enu = 0;
double y_enu = 0;
double z_enu = 0;
double q_x,q_y,q_z,q_w;
double ang_x,ang_y,ang_z;
double line_x,line_y,line_z;
GeographicLib::LocalCartesian geo_converter;

nav_msgs::Odometry gps_odo;
bool first = true;

class gps2enu
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_heading;
    ros::Subscriber sub_fix;
    ros::Publisher pub_gps;
public:
    gps2enu(/* args */);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &p);
    void heading_cb(const geometry_msgs::QuaternionStamped::ConstPtr &p);
    void fix_cb(const sensor_msgs::NavSatFix::ConstPtr &p);
    ~gps2enu();
};

gps2enu::gps2enu(/* args */)
{
    sub_vel = _nh.subscribe<geometry_msgs::TwistStamped>("/vel",2,&gps2enu::vel_cb,this);
    sub_heading = _nh.subscribe<geometry_msgs::QuaternionStamped>("/heading",2,&gps2enu::heading_cb,this);
    sub_fix = _nh.subscribe<sensor_msgs::NavSatFix>("/fix",2,&gps2enu::fix_cb,this);
    pub_gps = _nh.advertise<nav_msgs::Odometry>("/gps_odom",5);
}

void gps2enu::fix_cb(const sensor_msgs::NavSatFix::ConstPtr &p)
{
    if (first)
    {
        geo_converter.Reset(p->latitude,p->longitude,p->altitude);//重置原点   
        first = false;
    }
    gps_odo.pose.covariance[0] =  p->position_covariance[0];
    gps_odo.pose.covariance[7] =  p->position_covariance[4];
    gps_odo.pose.covariance[14] =  p->position_covariance[8];

    geo_converter.Forward(p->latitude,p->longitude,p->altitude,x_enu,y_enu,z_enu);//使用geographiclib转换经纬高至ENU    
}

void gps2enu::heading_cb(const geometry_msgs::QuaternionStamped::ConstPtr &p)
{
    q_x = p->quaternion.x;
    q_y = p->quaternion.y;
    q_z = p->quaternion.z;
    q_w = p->quaternion.w;
    // gps_odo.pose.covariance[0] =  p->quaternion;

}

void gps2enu::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &p)
{
    ang_x = p->twist.angular.x;
    ang_y = p->twist.angular.y;
    ang_z = p->twist.angular.z;

    line_x = p->twist.linear.x;
    line_y = p->twist.linear.y;
    line_z = p->twist.linear.z;

    gps_odo.pose.pose.position.x = x_enu;
    gps_odo.pose.pose.position.y = y_enu;
    gps_odo.pose.pose.position.z = z_enu;
    gps_odo.pose.pose.orientation.x = q_x;
    gps_odo.pose.pose.orientation.y = q_y;
    gps_odo.pose.pose.orientation.z = q_z;
    gps_odo.pose.pose.orientation.w = q_w;
    gps_odo.twist.twist.angular.x = ang_x;
    gps_odo.twist.twist.angular.y = ang_y;
    gps_odo.twist.twist.angular.z = ang_z;
    gps_odo.twist.twist.linear.x = line_x;
    gps_odo.twist.twist.linear.y = line_y;
    gps_odo.twist.twist.linear.z = line_z;
    gps_odo.header.frame_id = "origin";
    gps_odo.header.stamp = p->header.stamp;
    gps_odo.child_frame_id = "gps_odom";

    gps_odo.pose.covariance[21] = 0.01;
    gps_odo.pose.covariance[28] = 0.01;
    gps_odo.pose.covariance[35] = 0.01;
    gps_odo.twist.covariance[0] = 0.05;
    gps_odo.twist.covariance[7] = 0.05;
    gps_odo.twist.covariance[14] = 0.05;
    gps_odo.twist.covariance[21] = 0.05;
    gps_odo.twist.covariance[28] = 0.05;
    gps_odo.twist.covariance[35] = 0.05;

    pub_gps.publish(gps_odo);
}

gps2enu::~gps2enu()
{
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "gps2enu_node");

    gps2enu gps2odo;
    ros::spin();
    return 0;
}
