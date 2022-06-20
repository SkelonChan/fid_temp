#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <transf/encoder.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
#include <Eigen/Dense>

double yaw = 0;
double xt = 0;
double yt = 0;

double theta = 0;

class enc2odom
{
    private:
        ros::NodeHandle _nh;
        ros::Subscriber enc_sub;
        ros::Subscriber ser_sub;
        ros::Publisher odo_pub;
        nav_msgs::Odometry odo;
    public:
        enc2odom();
        ~enc2odom();
        void enc_sub_cb(const transf::encoder::ConstPtr &p);
        void ser_sub_cb(const std_msgs::Int16::ConstPtr &p);

};

enc2odom::enc2odom()
{
    enc_sub = _nh.subscribe<transf::encoder>("/ros_encoder",3,&enc2odom::enc_sub_cb,this);
    ser_sub = _nh.subscribe<std_msgs::Int16>("/servo",3,&enc2odom::ser_sub_cb,this);
    odo_pub = _nh.advertise<nav_msgs::Odometry>("/odom",3);

}

enc2odom::~enc2odom()
{
}
void enc2odom::enc_sub_cb(const transf::encoder::ConstPtr &p)
{
    yaw = yaw + p->vel * tan(theta)/0.62 * 0.05;
    xt = xt + p->vel * cos(yaw) * 0.05;
    yt = yt + p->vel * sin(yaw) * 0.05;
    
    odo.twist.twist.linear.x = p->vel * cos(yaw);
    odo.twist.twist.linear.y = p->vel * sin(yaw);
    odo.twist.twist.angular.z = p->vel * tan(theta)/0.62;
    odo.pose.pose.position.x = xt;
    odo.pose.pose.position.y = yt;

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,yaw);
    odo.pose.pose.orientation.x = qtn.getX();
    odo.pose.pose.orientation.y = qtn.getY();
    odo.pose.pose.orientation.z = qtn.getZ();
    odo.pose.pose.orientation.w = qtn.getW();

    odo.pose.covariance[0] = 0.1;//x
    odo.pose.covariance[7] = 0.1;//y
    odo.pose.covariance[14] = 0.1; //z
    odo.pose.covariance[21] = 0.1; //r
    odo.pose.covariance[28] = 0.1; //p
    odo.pose.covariance[35] = 0.1; //theta

    odo.twist.covariance[0] = 0.2;
    odo.twist.covariance[7] = 0.2;
    odo.twist.covariance[14] = 0.2;
    odo.twist.covariance[21] = 0.2;
    odo.twist.covariance[28] = 0.2;
    odo.twist.covariance[35] = 0.2;


    odo.header.frame_id = "origin";
    odo.header.stamp = ros::Time::now();
    odo.child_frame_id = "odom_vehicle_base";
    odo_pub.publish(odo);

    static tf2_ros::TransformBroadcaster bro;
    geometry_msgs::TransformStamped tfs;

    tfs.header.frame_id = "origin";
    tfs.header.stamp = ros::Time::now();

    tfs.child_frame_id = "odom_vehicle_base";
    
    tfs.transform.translation.x = xt;
    tfs.transform.translation.y = yt;
    tfs.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置

    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();


    //  5-3.广播器发布数据ca
    bro.sendTransform(tfs);
}

void enc2odom::ser_sub_cb(const std_msgs::Int16::ConstPtr &p)
{
    theta = -(-(p->data) + 90)*3.14159 /180;
}




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "wheel_odom_trans");

    enc2odom demo1;

    ros::spin();
    return 0;
}
