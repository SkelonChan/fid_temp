#include "mpc_fid/mpc_fid_main.h"
#include <Eigen/Dense>
#include <model_param.h>

//a b 矩阵为动力学模型中车辆参数得到的矩阵，与车辆的参数及滑移率等密切相关
Eigen::Matrix<double, 6,6> a;
Eigen::Matrix<double, 1,6> b;

vehicle_param vehicle_param1
// vehicle_param1.vehicle_param()
