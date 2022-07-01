
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>
#include <ros/ros.h>

// 1/2X'HX + f'X
// H = [1,-1,1;
    //  -1,2,-2;
    //  1,-2,4]
// f = [2,-3,1]'
// 0<=X <= 1
// sum(Xi)=1/2

/*
// H = [1,-1,1;
   //  -1,2,-2;
   //  1,-2,4]

f = [2;-3;1];
lb = zeros(3,1)
ub = ones(size(lb))
Aeq = one(1,3)
beq = 1/2

*/



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "osqp_demo");


  Eigen::SparseMatrix<double> hessian;
  hessian.resize(3,3);
  hessian.insert(0,0) = 1;
  hessian.insert(1,0) = -1;
  hessian.insert(2,0) = 1;
  hessian.insert(0,1) = -1;
  hessian.insert(1,1) = 2;
  hessian.insert(2,1) = -2;
  hessian.insert(0,2) = 1;
  hessian.insert(1,2) = -2;
  hessian.insert(2,2) = 4;
 // std::cout << "hessian:" << std::endl << hessian << std::endl;

  Eigen::Vector3d gradient;
  gradient.resize(3);
  gradient << 2, -3, 1;

 // std::cout << "gradient:" << std::endl << gradient << std::endl;

  Eigen::SparseMatrix<double> linearMatrix;
  linearMatrix.resize(4,3);
  linearMatrix.insert(0,0) = 1;
  linearMatrix.insert(1,0) = 0;
  linearMatrix.insert(2,0) = 0;
  linearMatrix.insert(3,0) = 1;

  linearMatrix.insert(0,1) = 0;
  linearMatrix.insert(1,1) = 1;
  linearMatrix.insert(2,1) = 0;
  linearMatrix.insert(3,1) = 1;

  linearMatrix.insert(0,2) = 0;
  linearMatrix.insert(1,2) = 0;
  linearMatrix.insert(2,2) = 1;
  linearMatrix.insert(3,2) = 1;
 // std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

  Eigen::Vector4d lowerBound;
  lowerBound.resize(4);
  lowerBound << 0, 0, 0, 0.5;
 // std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

  Eigen::Vector4d upperBound;
  upperBound.resize(4);
  upperBound << 1, 1, 1, 0.5;
//  std::cout << "upperBound:" << std::endl << upperBound << std::endl;

  int NumberOfVariables = 3; //A矩阵的列数
  int NumberOfConstraints = 4; //A矩阵的行数

  OsqpEigen::Solver solver;

  // settings
  //solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  //矩阵A为m*n矩阵
  solver.data()->setNumberOfVariables(NumberOfVariables);     //设置A矩阵的列数，即n
  solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m

  if (!solver.data()->setHessianMatrix(hessian))
    return 1; //设置P矩阵
  if (!solver.data()->setGradient(gradient))
    return 1; //设置q or f矩阵。当没有时设置为全0向量
  if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
    return 1; //设置线性约束的A矩阵
  if (!solver.data()->setLowerBound(lowerBound))
    return 1; //设置下边界
  if (!solver.data()->setUpperBound(upperBound))
    return 1; //设置上边界

  // instantiate the solver
  if (!solver.initSolver())
    return 1;

  Eigen::VectorXd QPSolution;

  clock_t time_start = clock();
  clock_t time_end = clock();
  time_start = clock();
  // solve the QP problem
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    return 1;

  // get the controller input

  QPSolution = solver.getSolution();
  time_end = clock();
  std::cout << "time use:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << std::endl;

  std::cout << "QPSolution:" << std::endl
            << QPSolution << std::endl;
            
    return 0;
}
