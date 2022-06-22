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
//定义消息格式生成的转化矩阵类型的数组，以保存接收到的历史信息
Eigen::Matrix4f gps_homo[3];
Eigen::Matrix4f vo_homo[3];
Eigen::Matrix4f odo_homo[3];

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
    void gps_sub_cb(const nav_msgs::Odometry::ConstPtr &p){msgtrans(gps_msg,p);}
    void vo_sub_cb(const nav_msgs::Odometry::ConstPtr &p){msgtrans(vo_msg,p);}
    void imu_sub_cb(const sensor_msgs::Imu::ConstPtr &s){msgtrans2(imu_msg,s);}
    void odo_sub_cb(const nav_msgs::Odometry::ConstPtr &p);
    void msgtrans(nav_msgs::Odometry &msg,const nav_msgs::Odometry::ConstPtr &tp);
    void msgtrans2(sensor_msgs::Imu &msg,const sensor_msgs::Imu::ConstPtr &ts);
    void cacluate_homo();
    Eigen::Matrix4f qua2rota(const nav_msgs::Odometry &tmp_msg);
    Eigen::Matrix4f cacluate_trans(const Eigen::Matrix4f &t1,const Eigen::Matrix4f &t2,
                                   const Eigen::Matrix4f &t3, const Eigen::Matrix4f &t4);
    Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3f &R);
    Eigen::Matrix3f trans2orien(const Eigen::Matrix4f &t1);

};

fid_calib::fid_calib()
{
    gps_sub = _nh.subscribe<nav_msgs::Odometry>("gps_odom",3,&fid_calib::gps_sub_cb,this);
    vo_sub = _nh.subscribe<nav_msgs::Odometry>("vo_odom",3,&fid_calib::vo_sub_cb,this);
    imu_sub = _nh.subscribe<sensor_msgs::Imu>("imu_odom",3,&fid_calib::imu_sub_cb,this);
    odo_sub = _nh.subscribe<nav_msgs::Odometry>("odo_odom",3,&fid_calib::odo_sub_cb,this);
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
    msg.child_frame_id = tp->child_frame_id;
    msg.header = tp->header;
    msg.pose = tp->pose;
    msg.twist = tp->twist;
}
void fid_calib::msgtrans2(sensor_msgs::Imu &msg,const sensor_msgs::Imu::ConstPtr &ts)
{
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
    msgtrans(odo_msg,p);//把信息提取到对应的全局变量中
    cacluate_homo();//计算当前坐标位姿的齐次变换矩阵
    //是否一定是同步帧的信息 TO DO 
    if(gps_queue_homo->isFull() && odo_queue_homo->isFull() && vo_queue_homo->isFull())//如果链表满足三个元素
    {
        //对由链表读入到数组中的数据进行计算相对变换矩阵
        Eigen::Matrix4f odo2gps1 = cacluate_trans(gps_trans_array[0],gps_trans_array[1],odo_trans_array[0],odo_trans_array[1]);
        Eigen::Matrix4f odo2gps2 = cacluate_trans(gps_trans_array[1],gps_trans_array[2],odo_trans_array[1],odo_trans_array[2]);
        Eigen::Matrix4f odo2gps3 = cacluate_trans(gps_trans_array[0],gps_trans_array[2],odo_trans_array[0],odo_trans_array[2]);
        Eigen::Matrix4f odo2vo1 = cacluate_trans(vo_trans_array[0],vo_trans_array[1],odo_trans_array[0],odo_trans_array[1]);
        Eigen::Matrix4f odo2vo2 = cacluate_trans(vo_trans_array[1],vo_trans_array[2],odo_trans_array[1],odo_trans_array[2]);
        Eigen::Matrix4f odo2vo3 = cacluate_trans(vo_trans_array[0],vo_trans_array[2],odo_trans_array[0],odo_trans_array[2]);
        //需要把旋转矩阵提取出来
        // odo2gps1.eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll
        Eigen::Vector3d odo2gps_rpy1 = rotationMatrixToEulerAngles(trans2orien(odo2gps1)); //RPY XYZ
        Eigen::Vector3d odo2gps_rpy2 = rotationMatrixToEulerAngles(trans2orien(odo2gps2)); //RPY XYZ
        Eigen::Vector3d odo2gps_rpy3 = rotationMatrixToEulerAngles(trans2orien(odo2gps3)); //RPY XYZ
        Eigen::Vector3d odo2vo_rpy1 = rotationMatrixToEulerAngles(trans2orien(odo2vo1)); //RPY XYZ
        Eigen::Vector3d odo2vo_rpy2 = rotationMatrixToEulerAngles(trans2orien(odo2vo2)); //RPY XYZ
        Eigen::Vector3d odo2vo_rpy3 = rotationMatrixToEulerAngles(trans2orien(odo2vo3)); //RPY XYZ

        //trans2orien(odo2gps_rpy1,odo2gps1);
        // eulerAngle1 = bokcc.eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll
        // odo2gps1.block<3,3>(0,0);


    }
}

Eigen::Matrix3f fid_calib::trans2orien(const Eigen::Matrix4f &t1)
{
    Eigen::Matrix3f ts;
    ts << t1(0,0),t1(0,1),t1(0,2),
          t1(1,0),t1(1,1),t1(1,2),
          t1(2,0),t1(2,1),t1(2,2);
    return ts;
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
    //生成各坐标系的齐次变换矩阵,并插入到链表当中
    gps_queue_homo->insert_1(qua2rota(gps_msg));
    vo_queue_homo->insert_1(qua2rota(vo_msg));
    odo_queue_homo->insert_1(qua2rota(odo_msg));
    //imu的信息格式 sensor_msgs/Imu中没有位置信息，暂时不对其进行位置确认，
    //只通过其对融合信息进行积分处理   
}


//odo_0 = tt1 * odo_1
//gps_0 = tt2 * gps_1
//odo_0 = tx * gps_0
//通过计算两个传感器采集到的两个点的信息，计算两个传感器之间的齐次变换矩阵
Eigen::Matrix4f fid_calib::cacluate_trans(const Eigen::Matrix4f &t1, const Eigen::Matrix4f &t2,
                                          const Eigen::Matrix4f &t3, const Eigen::Matrix4f &t4)
{
    Eigen::Matrix4f tt1 = t1 * t2.inverse();
    Eigen::Matrix4f tt2 = t3 * t4.inverse();
    Eigen::Matrix4f tx = tt1 * t2 *(tt2 * t4).inverse();
    return tx;
    //tx为t3经过变换后到t1的变换矩阵，即t1 = tx * t3//t2 = tx * t4;
}





Eigen::Matrix4f fid_calib::qua2rota(const nav_msgs::Odometry &tmp_msg)
{
    Eigen::Matrix3f tmp_rotation;
    Eigen::Quaternionf tmp_orien((float)tmp_msg.pose.pose.orientation.x,
                                (float)tmp_msg.pose.pose.orientation.y,
                                (float)tmp_msg.pose.pose.orientation.z,
                                (float)tmp_msg.pose.pose.orientation.w);
    tmp_rotation = tmp_orien.toRotationMatrix();
    Eigen::Matrix4f tmp_homo = Eigen::Matrix4f::Identity();
    tmp_homo.block<3,3>(0,0) = tmp_rotation;//对从（0，0）开始插入大小为3*3的矩阵
    tmp_homo(0,3) = tmp_msg.pose.pose.position.x;
    tmp_homo(1,3) = tmp_msg.pose.pose.position.y;
    tmp_homo(2,3) = tmp_msg.pose.pose.position.z;
    return tmp_homo;
}



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fid_calibration_node");


    ros::spin();
    return 0;
}
