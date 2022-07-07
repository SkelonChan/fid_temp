#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "vehicle_info_msg/vehicle_info_msg.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

geometry_msgs::PoseWithCovarianceStamped sub_msg;
vehicle_info_msg::vehicle_info_msg vehicle_msg;


struct last_info
{
    double last_time = 0;
    double last_accx = 0;
    double last_accy = 0;
    double last_x = 0;
    double last_y = 0;
    double last_z = 0;
    double last_vx = 0;
    double last_vy = 0;
    double last_vz = 0;

    double last_yaw = 0;
    double last_yaw_rate = 0;
    double last_yaw_rate_acc = 0;
};
    
    bool isfirsttime = true;
    last_info last_state;
    double tt = 0.03;//假定一个初始时间间隔，后续根据时间辍更新

class calibrated2vehicel
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    calibrated2vehicel(/* args */);
    ~calibrated2vehicel();
    void sub_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &p);
    static Eigen::Vector3d qua2rpy(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void msg_deal();
};

calibrated2vehicel::calibrated2vehicel(/* args */)
{
    sub = _nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/calibrated_msg",3,&calibrated2vehicel::sub_cb,this);
    pub = _nh.advertise<vehicle_info_msg::vehicle_info_msg>("/vehicle_info",3);
}

calibrated2vehicel::~calibrated2vehicel()
{
}

void calibrated2vehicel::sub_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &p)
{
    sub_msg.header = p->header;
    sub_msg.pose = p->pose;
    msg_deal();

    pub.publish(vehicle_msg);

}
//对得到的信息进行处理，得到vx vy yaw_rate
void calibrated2vehicel::msg_deal()
{
    //采取的是上一时刻帧中的速度的推算，与本时刻帧速度权重各区1/2的策略()
    if (isfirsttime == false)
        tt = ros::Time::now().toSec() - last_state.last_time;
    double now_vx = ((sub_msg.pose.pose.position.x - last_state.last_x) / tt + (last_state.last_vx + last_state.last_accx * tt))/2;
    double now_vy = ((sub_msg.pose.pose.position.y - last_state.last_y) / tt + (last_state.last_vy + last_state.last_accy * tt))/2;
    // double now_vz = (sub_msg.pose.pose.position.z - last_state.last_z) / tt;
    Eigen::Vector3d RPY = calibrated2vehicel::qua2rpy(sub_msg); 
    double yaw = RPY(2);
    double yaw_rate = ((yaw - last_state.last_yaw) / tt + (last_state.last_yaw_rate + last_state.last_yaw_rate_acc * tt))/ 2;

    vehicle_msg.phi = yaw;
    vehicle_msg.phi_dot = yaw_rate;
    vehicle_msg.x_dot = now_vx;
    vehicle_msg.y_dot = now_vy;
    vehicle_msg.y_pos = sub_msg.pose.pose.position.x;
    vehicle_msg.x_pos = sub_msg.pose.pose.position.y;

    isfirsttime = false;
    //信息更新
    last_state.last_x = sub_msg.pose.pose.position.x;
    last_state.last_y = sub_msg.pose.pose.position.y;
    last_state.last_yaw = yaw;
    last_state.last_accx = (now_vx - last_state.last_vx) / tt;
    last_state.last_accy = (now_vy - last_state.last_vy) / tt;
    last_state.last_vx = now_vx;
    last_state.last_vy = now_vy;
    last_state.last_yaw_rate_acc = (yaw_rate - last_state.last_yaw_rate) / tt;
    last_state.last_yaw_rate = yaw_rate;
    last_state.last_time = ros::Time::now().toSec();
}

Eigen::Vector3d calibrated2vehicel::qua2rpy(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    Eigen::Quaterniond qua(msg.pose.pose.orientation.w,
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z);
    Eigen::Vector3d RPY = qua.matrix().eulerAngles(0,1,2);//xyz，即RPY
    return RPY;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv,"calibrated2vehicle");

    calibrated2vehicel vehicel;
    ros::spin();
    return 0;

}