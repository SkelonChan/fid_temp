#ifndef _vehicle_param_
#define _vehicle_param_
// #include<eigen3/Eigen/Core>
#include <Eigen/Dense>


class vehicle_param{
 public:
    int mass;//vehilce mass
    double g; //gravity
    double lf;//vehicle lenght of the front
    double lr; //vehicle length 
    double l; //wheellbase
    double Iz;// internal in z xais
    double ccf;//vehicle tire param
    double ccr;
    double clf;
    double clr;
    double ts;//resmaple time
vehicle_param(int _mass,double _g,double _lf,double _lr,double _l ,double _Iz,double _ccf,double _ccr,double _clf,double _clr,double _ts){
    mass = _mass;g = _g;lf=_lf;lr=_lr;l=_l;Iz=_Iz;ccf=_ccf;ccr=_ccr;clf=_clf;clr=_clr;ts=_ts;
}
};
class vehicle_info{
    public:
        double y_dot;
        double x_dot;
        double phi;
        double phi_dot;
        double x_pos;
        double y_pos;
        double delta_f;
    vehicle_info(double _y_dot,double _x_dot,double _phi,double _phi_dot,double _x_pos,double _y_pos,double _delta_f){
        y_dot=_y_dot;x_dot=_y_dot;phi=_phi;phi_dot=_phi_dot;x_pos=_x_pos;y_pos=_y_pos;delta_f=_delta_f;
    }
};

class MPC_param{
    public:
    int NP; //predict horizon
    int NC; //control horizon
    int Nx; //number of the state elements
    int Nu; //number of the control elementl; 
    MPC_param(int _NP,int _NC, int _Nx,int _Nu){
        NP=_NP;NC=_NC;Nx=_Nx;Nu=_Nu;
    }
};



#endif