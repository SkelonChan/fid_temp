#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <Eigen/Core>


//对GPS VO的频率进行预差分

//预保存订阅信息
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odo_msg;
nav_msgs::Odometry gps_msg;
nav_msgs::Odometry vo_msg;
nav_msgs::Odometry gps_msg_new;
nav_msgs::Odometry vo_msg_new;
double last_time_imu;
double t_gap;
sensor_msgs::Imu last_imu_msg;

struct ang_vel{
 double x_angular_vel;
 double y_angular_vel;
 double z_angular_vel;
};

ang_vel last_imu_ang_vel;


class time_syn
{
private:
    ros::NodeHandle _nh;
    ros::Publisher pub_gps;
    ros::Publisher pub_imu;
    ros::Publisher pub_odo;
    ros::Publisher pub_vo;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_vo;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_odo;
public:
    time_syn(/* args */);
    ~time_syn();
    void gps_cb(const nav_msgs::Odometry::ConstPtr &p){msgtrans(gps_msg,p);last_time_imu = p->header.stamp.toSec();};
    void vo_cb(const nav_msgs::Odometry::ConstPtr &p){msgtrans(vo_msg,p);};
    void odo_cb(const nav_msgs::Odometry::ConstPtr &p){msgtrans(odo_msg,p);};
    void imu_cb(const sensor_msgs::Imu::ConstPtr &p);
    void msgtrans(nav_msgs::Odometry &msg,const nav_msgs::Odometry::ConstPtr &tp);
    void msgtrans2(sensor_msgs::Imu &msg,const sensor_msgs::Imu::ConstPtr &ts);
    Eigen::Quaterniond rpy2qua(const Eigen::Vector3d &p);
    Eigen::Vector3d qua2rpy(const nav_msgs::Odometry &msg);

};

time_syn::time_syn()
{
    sub_gps = _nh.subscribe<nav_msgs::Odometry>("/gps_odom",3,&time_syn::gps_cb,this);
    sub_odo = _nh.subscribe<nav_msgs::Odometry>("/wheel_odom",3,&time_syn::odo_cb,this);
    sub_vo = _nh.subscribe<nav_msgs::Odometry>("/zed2/zed_node/odom",3,&time_syn::vo_cb,this);
    sub_imu = _nh.subscribe<sensor_msgs::Imu>("/zed2/zed_node/imu/data",3,&time_syn::imu_cb,this);
    pub_gps = _nh.advertise<nav_msgs::Odometry>("/syn_gps",5);
    pub_odo = _nh.advertise<nav_msgs::Odometry>("/syn_odo",5);
    pub_vo = _nh.advertise<nav_msgs::Odometry>("/syn_vo",5);
}

time_syn::~time_syn()
{
}

void time_syn::msgtrans(nav_msgs::Odometry &msg,const nav_msgs::Odometry::ConstPtr &tp)
{
    //将接收到的里程计信息保存到新的全局信息文件中
    msg.child_frame_id = tp->child_frame_id;
    msg.header = tp->header;
    msg.pose = tp->pose;
    msg.twist = tp->twist;
}
void time_syn::msgtrans2(sensor_msgs::Imu &msg,const sensor_msgs::Imu::ConstPtr &ts)
{
    //将接收到的IMU信息保存到新的全局信息文件中
    msg.angular_velocity = ts->angular_velocity;
    msg.angular_velocity_covariance = ts->angular_velocity_covariance;
    msg.header = ts->header;
    msg.linear_acceleration = ts->linear_acceleration;
    msg.linear_acceleration_covariance = ts->linear_acceleration_covariance;
    msg.orientation = ts->orientation;
    msg.orientation_covariance = ts->orientation_covariance;
}

void time_syn::imu_cb(const sensor_msgs::Imu::ConstPtr &p)
{
    msgtrans2(imu_msg,p);
    if (gps_msg.header.stamp.toSec() < p->header.stamp.toSec())
    {
        //使用运动学方程外推，根据imu的频率得到对应时间辍下的gps/vo值
        t_gap = p->header.stamp.toSec() - last_time_imu;
        gps_msg.header.stamp = p->header.stamp;
        gps_msg.pose.pose.position.x = gps_msg.pose.pose.position.x + gps_msg.twist.twist.linear.x * t_gap + 0.5 * p->linear_acceleration.x * pow(t_gap,2);
        gps_msg.pose.pose.position.y = gps_msg.pose.pose.position.y + gps_msg.twist.twist.linear.y * t_gap + 0.5 * p->linear_acceleration.y * pow(t_gap,2);
        gps_msg.pose.pose.position.z = gps_msg.pose.pose.position.z + gps_msg.twist.twist.linear.y * t_gap + 0.5 * p->linear_acceleration.y * pow(t_gap,2);
        gps_msg.twist.twist.linear.x = gps_msg.twist.twist.linear.x + p->linear_acceleration.x * t_gap;
        gps_msg.twist.twist.linear.y = gps_msg.twist.twist.linear.y + p->linear_acceleration.y * t_gap;
        gps_msg.twist.twist.linear.z = gps_msg.twist.twist.linear.z + p->linear_acceleration.z * t_gap;
        gps_msg.twist.twist.angular.x = gps_msg.twist.twist.angular.x + p->angular_velocity.x * t_gap;
        gps_msg.twist.twist.angular.y = gps_msg.twist.twist.angular.y + p->angular_velocity.y * t_gap;
        gps_msg.twist.twist.angular.z = gps_msg.twist.twist.angular.z + p->angular_velocity.z * t_gap;
        
        //对GPS得到的四元数进行更新
        //Eigen::Vector3d imu_rpy = qua2rpy(*p);
        Eigen::Vector3d gps_rpy = qua2rpy(gps_msg);

        
        ang_vel ang_acc;
        ang_acc.x_angular_vel = (last_imu_ang_vel.x_angular_vel - p->angular_velocity.x)/ t_gap;
        ang_acc.y_angular_vel = (last_imu_ang_vel.y_angular_vel - p->angular_velocity.y)/ t_gap;
        ang_acc.z_angular_vel = (last_imu_ang_vel.z_angular_vel - p->angular_velocity.z)/ t_gap;
        

        gps_rpy(0) = gps_rpy(0) + p->angular_velocity.x * t_gap + 0.5 * ang_acc.x_angular_vel * pow(t_gap,2);
        gps_rpy(1) = gps_rpy(1) + p->angular_velocity.y * t_gap + 0.5 * ang_acc.y_angular_vel * pow(t_gap,2);
        gps_rpy(2) = gps_rpy(2) + p->angular_velocity.z * t_gap + 0.5 * ang_acc.z_angular_vel * pow(t_gap,2);

        Eigen::Quaterniond gps_qua_new = rpy2qua(gps_rpy);
        gps_msg.pose.pose.orientation.w = gps_qua_new.coeffs()(0);//有Eigen下的quaterniond转为单独的数，正则表达式 提取非零系数
        gps_msg.pose.pose.orientation.x = gps_qua_new.coeffs()(1);
        gps_msg.pose.pose.orientation.y = gps_qua_new.coeffs()(2);
        gps_msg.pose.pose.orientation.z = gps_qua_new.coeffs()(3);
        

//TO DO 
//对于初始imu时间 last_ang_vel的选取再次确认
//完成对VO的预插值处理
//信息的发布
//calibration中  得到两个坐标系之间的相对RPY角度 的信息的使用
//重新对本时间同步部分的梳理， 对在线位姿相对标定的重新梳理

        //更新last的信息
        last_imu_ang_vel.x_angular_vel = p->angular_velocity.x;
        last_imu_ang_vel.y_angular_vel = p->angular_velocity.y;
        last_imu_ang_vel.z_angular_vel = p->angular_velocity.z;

        last_time_imu = p->header.stamp.toSec();
    }
    
}



//对四元数的下一帧率计算
//四元数转RPY,gps，imu ：： qua----->RPY
//记录imu上一帧的角速度信息，与本帧的角速度信息，得到角加速度信息
//计算得到本帧对应的RPY角
//RPY转四元数

Eigen::Quaterniond time_syn::rpy2qua(const Eigen::Vector3d &p)
{
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(p(0),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(p(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(p(2),Eigen::Vector3d::UnitZ()));
        
    Eigen::Quaterniond quater = yawAngle*pitchAngle*rollAngle;
    return quater;
}

//四元数转欧拉角
// Eigen::Vector3d qua2rpy(const sensor_msgs::Imu &msg)
// {
//     Eigen::Quaterniond qua(msg.orientation.w,
//                            msg.orientation.x,
//                            msg.orientation.y,
//                            msg.orientation.z);
//     Eigen::Vector3d RPY = qua.matrix().eulerAngles(0,1,2);//xyz，即RPY
//     return RPY;  
// }

Eigen::Vector3d time_syn::qua2rpy(const nav_msgs::Odometry &msg)
{
    Eigen::Quaterniond qua(msg.pose.pose.orientation.w,
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z);
    Eigen::Vector3d RPY = qua.matrix().eulerAngles(0,1,2);//xyz，即RPY
    return RPY;
}
//初始化last信息
void initialize(){
    last_imu_ang_vel.x_angular_vel = 0;
    last_imu_ang_vel.y_angular_vel = 0;
    last_imu_ang_vel.z_angular_vel = 0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "time_synchronization");
    initialize();




    ros::spin();
}