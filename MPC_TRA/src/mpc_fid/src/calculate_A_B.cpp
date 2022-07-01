#include "mpc_fid/model_param.h"
#include <Eigen/Dense>
#include <Eigen/Jacobi>
void calculate_A_B(class vehicle_info *info,class vehicle_param *p,double sf,double sr)
{
    // sf sr 为车辆滑移率
    int mass = p->mass;//vehilce mass
    double g = p->g; //gravity
    double lf = p->lf;//vehicle lenght of the front
    double lr = p->lr; //vehicle length 
    double l = p->l; //wheellbase
    double Iz = p->Iz;// internal in z xais
    double ccf = p->ccf;//vehicle tire param
    double ccr = p->ccr;
    double clf = p->clf;
    double clr = p->clr;
    double ts = p->ts;//resmaple time
    
    double y_dot = info->y_dot;
    double x_dot = info->x_dot;
    double phi = info->phi;
    double phi_dot = info->phi_dot;
    double x_pos = info->x_pos;
    double y_pos = info->y_pos;
    double delta_f = info->delta_f;

    //车辆动力学模型
    double dy_dot = -x_dot* phi_dot+(ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+ccr*(lr*phi_dot-y_dot)/x_dot)/mass;
    double dx_dot = y_dot*phi_dot+2*(clf*sf+clf*sr-ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/mass;
    double dphi_dot = (2*lf*ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*ccr*(lr*phi_dot-y_dot)/x_dot)/Iz;
    double Y_dot = x_dot*sin(phi)+y_dot*cos(phi);
    double X_dot = x_dot*cos(phi)-y_dot*sin(phi);
    // dphi_dot = dphi_dot;

    //雅可比矩阵求解
    Eigen::Matrix<double, 6, 1> Dynamics_func;
    Eigen::Matrix<double, 1, 6> state_vector;
    Dynamics_func << dy_dot,dx_dot,phi_dot,dphi_dot,Y_dot,X_dot;
    state_vector << y_dot,x_dot,phi,phi_dot,y_pos,x_pos;
    Eigen::Matrix<double,1,1> control_input;
    control_input << delta_f;
    
    //对矩阵的jacobian函数实现为找到方式，暂时弃坑
}
