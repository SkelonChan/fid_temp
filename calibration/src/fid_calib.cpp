#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "calibration/fid_queue.h"
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;


nav_msgs::Odometry odo_msg;
nav_msgs::Odometry gps_msg;
nav_msgs::Odometry vo_msg;
sensor_msgs::Imu imu_msg;


Queue<Eigen::Matrix4f> *gps_queue_homo = new Queue<Eigen::Matrix4f>();
Queue<Eigen::Matrix4f> *vo_queue_homo = new Queue<Eigen::Matrix4f>();
Queue<Eigen::Matrix4f> *odo_queue_homo = new Queue<Eigen::Matrix4f>();
// nav_msgs::Odometry w[];

class fid_calib
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber gps_sub;
    ros::Subscriber vo_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odo_sub;
    ros::Publisher cali_gps_pub;
    ros::Publisher cali_vo_pub;
    ros::Publisher cali_imu_pub;
    ros::Publisher cali_odo_pub;
public:
    fid_calib( );
    ~fid_calib();
    void gps_sub_cb(const nav_msgs::Odometry::ConstPtr &p){ROS_INFO("GET THE gps_msg");msgtrans(gps_msg,p);}
    void vo_sub_cb(const nav_msgs::Odometry::ConstPtr &p){ROS_INFO("GET THE vo_msg");msgtrans(vo_msg,p);}
    void imu_sub_cb(const sensor_msgs::Imu::ConstPtr &s){ROS_INFO("GET THE imu_msg");msgtrans2(imu_msg,s);}
    void odo_sub_cb(const nav_msgs::Odometry::ConstPtr &p);
    void msgtrans(nav_msgs::Odometry &msg,const nav_msgs::Odometry::ConstPtr &tp);
    void msgtrans2(sensor_msgs::Imu &msg,const sensor_msgs::Imu::ConstPtr &ts);
    void cacluate_homo();
    Eigen::Matrix4f qua2rota(const nav_msgs::Odometry &tmp_msg);
    Eigen::Matrix4f cacluate_trans(const Eigen::Matrix4f &t1,const Eigen::Matrix4f &t2,
                                   const Eigen::Matrix4f &t3, const Eigen::Matrix4f &t4);
    Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3f &R);

};

fid_calib::fid_calib()
{
    gps_sub = _nh.subscribe<nav_msgs::Odometry>("/gps_odom",3,&fid_calib::gps_sub_cb,this);
    vo_sub = _nh.subscribe<nav_msgs::Odometry>("/zed2/zed_node/odom",3,&fid_calib::vo_sub_cb,this);
    imu_sub = _nh.subscribe<sensor_msgs::Imu>("/zed2/zed_node/imu/data",3,&fid_calib::imu_sub_cb,this);
    odo_sub = _nh.subscribe<nav_msgs::Odometry>("wheel_odom",3,&fid_calib::odo_sub_cb,this);
    cali_gps_pub = _nh.advertise<nav_msgs::Odometry>("/cali_gps_odom",3);
    cali_vo_pub = _nh.advertise<nav_msgs::Odometry>("/cali_vo_odom",3);
    cali_imu_pub = _nh.advertise<sensor_msgs::Imu>("/cali_imu_odom",3);
    cali_odo_pub = _nh.advertise<nav_msgs::Odometry>("/cali_odo_odom",3);
}

fid_calib::~fid_calib()
{
}


void fid_calib::msgtrans(nav_msgs::Odometry &msg,const nav_msgs::Odometry::ConstPtr &tp)
{
    //??????????????????????????????????????????????????????????????????
    msg.child_frame_id = tp->child_frame_id;
    msg.header = tp->header;
    msg.pose = tp->pose;
    msg.twist = tp->twist;
}
void fid_calib::msgtrans2(sensor_msgs::Imu &msg,const sensor_msgs::Imu::ConstPtr &ts)
{
    //???????????????IMU??????????????????????????????????????????
    msg.angular_velocity = ts->angular_velocity;
    msg.angular_velocity_covariance = ts->angular_velocity_covariance;
    msg.header = ts->header;
    msg.linear_acceleration = ts->linear_acceleration;
    msg.linear_acceleration_covariance = ts->linear_acceleration_covariance;
    msg.orientation = ts->orientation;
    msg.orientation_covariance = ts->orientation_covariance;
}


void fid_calib::odo_sub_cb(const nav_msgs::Odometry::ConstPtr &p)
{
    ROS_INFO("GET THE wheel_odom_msg");
    msgtrans(odo_msg,p);//??????????????????????????????????????????
    cacluate_homo();//?????????????????????????????????????????????
    //????????????????????????????????? TO DO 
    if(gps_queue_homo->isFull() && odo_queue_homo->isFull() && vo_queue_homo->isFull())//??????????????????????????????
    {
        //?????????????????????????????????????????????????????????????????????
        Eigen::Matrix4f gps2odo1 = cacluate_trans(odo_trans_array[0],odo_trans_array[1],gps_trans_array[0],gps_trans_array[1]);
        Eigen::Matrix4f gps2odo2 = cacluate_trans(odo_trans_array[1],odo_trans_array[2],gps_trans_array[1],gps_trans_array[2]);
        Eigen::Matrix4f gps2odo3 = cacluate_trans(odo_trans_array[0],odo_trans_array[2],gps_trans_array[0],gps_trans_array[2]);
        Eigen::Matrix4f vo2odo1 = cacluate_trans(odo_trans_array[0],odo_trans_array[1],vo_trans_array[0],vo_trans_array[1]);
        Eigen::Matrix4f vo2odo2 = cacluate_trans(odo_trans_array[1],odo_trans_array[2],vo_trans_array[1],vo_trans_array[2]);
        Eigen::Matrix4f vo2odo3 = cacluate_trans(odo_trans_array[0],odo_trans_array[2],vo_trans_array[0],vo_trans_array[2]);
        //?????????????????????????????????
        // gps2odo1.eulerAngles(2,1,0); // ZYX?????????yaw,pitch,roll
        //????????????????????????????????????RPY??????
        Eigen::Vector3d gps2odo_rpy1 = rotationMatrixToEulerAngles(gps2odo1.block<3,3>(0,0)); //RPY XYZ
        Eigen::Vector3d gps2odo_rpy2 = rotationMatrixToEulerAngles(gps2odo2.block<3,3>(0,0)); //RPY XYZ
        Eigen::Vector3d gps2odo_rpy3 = rotationMatrixToEulerAngles(gps2odo3.block<3,3>(0,0)); //RPY XYZ
        Eigen::Vector3d vo2odo_rpy1 = rotationMatrixToEulerAngles(vo2odo1.block<3,3>(0,0)); //RPY XYZ
        Eigen::Vector3d vo2odo_rpy2 = rotationMatrixToEulerAngles(vo2odo2.block<3,3>(0,0)); //RPY XYZ
        Eigen::Vector3d vo2odo_rpy3 = rotationMatrixToEulerAngles(vo2odo3.block<3,3>(0,0)); //RPY XYZ

        //???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        //???????????????gps->odo??????????????????????????????????????????????????????????????????odo?????????????????????vo??????
        //gps_trans_odo???vo_trans_odo???gps????????????????????????vo???????????????odo????????????
        Eigen::Matrix4f gps_trans_odo = (gps2odo1+gps2odo2+gps2odo3)/3 * gps_queue_homo->getrear();
        Eigen::Matrix4f vo_trans_odo = (vo2odo1+vo2odo2+vo2odo3)/3 * vo_queue_homo->getrear();

        //
        Eigen::Vector3f gps_linear,vo_linear;
        Eigen::Vector3f gps_angular,vo_angular;
        gps_linear << gps_msg.twist.twist.linear.x,gps_msg.twist.twist.linear.y,gps_msg.twist.twist.linear.z;
        vo_linear << vo_msg.twist.twist.linear.x,vo_msg.twist.twist.linear.y,vo_msg.twist.twist.linear.z;
        gps_angular << gps_msg.twist.twist.angular.x,gps_msg.twist.twist.angular.y,gps_msg.twist.twist.angular.z;
        vo_angular << vo_msg.twist.twist.angular.x,vo_msg.twist.twist.angular.y,vo_msg.twist.twist.angular.z;

        //??????????????????????????????odo???????????? v'= R* v   w' = R * w
        Eigen::Vector3f gps_linear_new = gps_trans_odo.block<3,3>(0,0) * gps_linear;
        Eigen::Vector3f gps_angular_new = gps_trans_odo.block<3,3>(0,0) * gps_angular;
        Eigen::Vector3f vo_linear_new = vo_trans_odo.block<3,3>(0,0) * vo_linear;
        Eigen::Vector3f vo_angular_new = vo_trans_odo.block<3,3>(0,0) * vo_angular;


        Eigen::Quaternionf gps_trans_odo_quater(gps_trans_odo.block<3,3>(0,0));
        Eigen::Quaternionf vo_trans_odo_quater(vo_trans_odo.block<3,3>(0,0));
        
        //???Eigen?????????????????? wxyz
        gps_msg.pose.pose.orientation.w = gps_trans_odo_quater.coeffs()(0,0);//???w,x,y,z)
        gps_msg.pose.pose.orientation.x = gps_trans_odo_quater.coeffs()(0,1);//???w,x,y,z)
        gps_msg.pose.pose.orientation.y = gps_trans_odo_quater.coeffs()(0,2);//???w,x,y,z)
        gps_msg.pose.pose.orientation.z = gps_trans_odo_quater.coeffs()(0,3);//???w,x,y,z)
        gps_msg.pose.pose.position.x = gps_trans_odo(0,3);
        gps_msg.pose.pose.position.y = gps_trans_odo(1,3);
        gps_msg.pose.pose.position.z = gps_trans_odo(2,3);
        gps_msg.twist.twist.angular.x = gps_angular_new(0);
        gps_msg.twist.twist.angular.y = gps_angular_new(1);
        gps_msg.twist.twist.angular.z = gps_angular_new(2);
        gps_msg.twist.twist.linear.x = gps_linear_new(0);
        gps_msg.twist.twist.linear.y = gps_linear_new(1);
        gps_msg.twist.twist.linear.z = gps_linear_new(2);
        gps_msg.header.frame_id = "origin";
        gps_msg.child_frame_id = "gps_odom";

        vo_msg.pose.pose.orientation.w = vo_trans_odo_quater.coeffs()(0);//???w,x,y,z)
        vo_msg.pose.pose.orientation.x = vo_trans_odo_quater.coeffs()(1);//???w,x,y,z)
        vo_msg.pose.pose.orientation.y = vo_trans_odo_quater.coeffs()(2);//???w,x,y,z)
        vo_msg.pose.pose.orientation.z = vo_trans_odo_quater.coeffs()(3);//???w,x,y,z)
        vo_msg.pose.pose.position.x = vo_trans_odo(0,3);
        vo_msg.pose.pose.position.y = vo_trans_odo(1,3);
        vo_msg.pose.pose.position.z = vo_trans_odo(2,3);
        vo_msg.twist.twist.angular.x = vo_angular_new(0);
        vo_msg.twist.twist.angular.y = vo_angular_new(1);
        vo_msg.twist.twist.angular.z = vo_angular_new(2);
        vo_msg.twist.twist.linear.x = vo_linear_new(0);
        vo_msg.twist.twist.linear.y = vo_linear_new(1);
        vo_msg.twist.twist.linear.z = vo_linear_new(2);
        vo_msg.header.frame_id = "origin";
        vo_msg.child_frame_id = "vo_odom";

        imu_msg.header.frame_id = "origin";

        cali_gps_pub.publish(gps_msg);
        cali_vo_pub.publish(vo_msg);
        cali_imu_pub.publish(imu_msg);
        cali_odo_pub.publish(odo_msg);
        
    }
}

bool isRotationMatirx(Eigen::Matrix3f R)
{
    double err=1e-6;
    Eigen::Matrix3f shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3f I=Eigen::Matrix3f::Identity();
    return (shouldIdenity - I).norm() < err;
}

Eigen::Vector3d fid_calib::rotationMatrixToEulerAngles(const Eigen::Matrix3f &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}







void fid_calib::cacluate_homo()
{
    //???????????????????????????????????????,????????????????????????
    gps_queue_homo->insert_1(qua2rota(gps_msg));
    vo_queue_homo->insert_1(qua2rota(vo_msg));
    odo_queue_homo->insert_1(qua2rota(odo_msg));
    //imu??????????????? sensor_msgs/Imu????????????????????????????????????????????????????????????
    //?????????????????????????????????????????????   
}


//odo_0 = tt1 * odo_1
//gps_0 = tt2 * gps_1
//odo_0 = tx * gps_0
//????????????????????????????????????????????????????????????????????????????????????????????????????????????

/*
imu_base <--------------ITib-------------imu2
|                                          |
|                                          |
|                                          |  
|tx                                        |T
|                                          |
|                                          |
|                                          |
|                                          |
odo_base<---------------OTob-------------odo2
*/
Eigen::Matrix4f fid_calib::cacluate_trans(const Eigen::Matrix4f &t1, const Eigen::Matrix4f &t2,
                                          const Eigen::Matrix4f &t3, const Eigen::Matrix4f &t4)
{
    // Eigen::Matrix4f tt1 = t1 * t2.inverse();
    // Eigen::Matrix4f tt2 = t3 * t4.inverse();
    // Eigen::Matrix4f tx = tt1 * t2 *(tt2 * t4).inverse();

    Eigen::Matrix4f tx = t2 * t1.inverse() * t4 * t2.inverse() * t3 * t4.inverse();
    return tx;
    //tx???t3??????????????????t1?????????????????????t1 = tx * t3//t2 = tx * t4;
}




//???????????????????????????
Eigen::Matrix4f fid_calib::qua2rota(const nav_msgs::Odometry &tmp_msg)
{
    Eigen::Matrix3f tmp_rotation;
    Eigen::Quaternionf tmp_orien((float)tmp_msg.pose.pose.orientation.w,
                                (float)tmp_msg.pose.pose.orientation.x,
                                (float)tmp_msg.pose.pose.orientation.y,
                                (float)tmp_msg.pose.pose.orientation.z);
    tmp_rotation = tmp_orien.toRotationMatrix();
    Eigen::Matrix4f tmp_homo = Eigen::Matrix4f::Identity();
    tmp_homo.block<3,3>(0,0) = tmp_rotation;//?????????0???0????????????????????????3*3?????????
    tmp_homo(0,3) = tmp_msg.pose.pose.position.x;
    tmp_homo(1,3) = tmp_msg.pose.pose.position.y;
    tmp_homo(2,3) = tmp_msg.pose.pose.position.z;
    return tmp_homo;
}





int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fid_calibration_node");

    fid_calib fid;
    ros::spin();
    return 0;
}
