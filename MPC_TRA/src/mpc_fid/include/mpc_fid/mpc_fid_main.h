#ifndef _mpc_fid_main_
#define _mpc_fid_main_

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <vehicle_info_msg/gp_msg.h>
#include <vehicle_info_msg/vehicle_control_msg.h>
#include <vehicle_info_msg/vehicle_info_msg.h>
#include <vehicle_info_msg/GPRdata.h>



class mpc_follow
{
    private:
    ros::NodeHandle _nh;
    ros::Subscriber _vehicle_info_sub;
    ros::Subscriber _gp_com_sub;
    ros::Publisher _vehicle_control_pub;

    const double vel_param_mass = 30;
    const double vel_param_g = 9.8;
    const double vel_param_lf = 0.28;
    const double vel_param_lr = 0.34;
    const double vel_param_l = 0.62;
    const double Iz = 170;//Fake!
    const double ccf = 66900;
    const double ccr = 62700;
    const double clf = 66900;
    const double clr = 62700;
    const double ts = 0.03;//[s]
    const double T = 0.03;//
    double y_dot;
    double x_dot;
    double phi;
    double phi_dot;
    double x_pos;
    double y_pos;
    double delta_f;
    double mean_vx;
    double mean_vy;
    double mean_yaw_rate;
    const int Np = 20;//predict horizon
    const int Nc = 5;//control horizon
    const int Ny = 2;//output number
    const int Nx = 6;//states number
    const int Nu = 1; //control number
    const int Row = 1000; //松弛因子

    Eigen::Matrix<double,6,6> a;
    Eigen::Matrix<double,1,6> b;
    public:
        mpc_follow();
        ~mpc_follow();
        void vehicle_info_cb(const vehicle_info_msg::vehicle_info_msg::ConstPtr &p);
        void gp_com_cb(const vehicle_info_msg::GPRdata::ConstPtr &p);
    
    
};

mpc_follow::mpc_follow()
{
    if(add_gp)
    _gp_com_sub = _nh.subscribe<vehicle_info_msg::GPRdata>("/gp_com",1,&mpc_follow::gp_com_cb,this);
    _vehicle_info_sub = _nh.subscribe<vehicle_info_msg::vehicle_info_msg>("/vehicle_info",1,&mpc_follow::vehicle_info_cb,this);
    
     _vehicle_control_pub = _nh.advertise<vehicle_info_msg::vehicle_control_msg>("/vehicle_control",1);

}

void mpc_follow::vehicle_info_cb(const vehicle_info_msg::vehicle_info_msg::ConstPtr &p)
{
    y_dot = p->y_dot;
    x_dot = p->x_dot;
    phi = p->phi;
    phi_dot = p->phi_dot;
    x_pos = p->x_pos;
    y_pos = p->y_pos;
    delta_f = 0;///this need to update every time.run the callback_function
}

void mpc_follow::gp_com_cb(const vehicle_info_msg::GPRdata::ConstPtr &p)
{
    mean_vx = p->mean_vx;
    mean_vy = p->mean_vy;
    mean_yaw_rate = p->mean_yaw_rate;
}


#endif