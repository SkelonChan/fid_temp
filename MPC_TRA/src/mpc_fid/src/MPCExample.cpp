
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include "osqp/osqp.h"
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
  Eigen::VectorXd QPSolution;
  int NumberOfVariables = 3; //A矩阵的列数
  int NumberOfConstraints = 4; //A矩阵的行数
  Eigen::SparseMatrix<double> hessian;
  Eigen::Vector3d gradient;
  Eigen::SparseMatrix<double> linearMatrix;
  Eigen::Vector4d lowerBound;
  Eigen::Vector4d upperBound;

void H2hessian(const Eigen::Matrix<double,3,3> &H,Eigen::SparseMatrix<double> &hessianMatrix)
{
    hessianMatrix.resize(3,3);
    for (int i = 0; i < 3;i++)
    {
        for(int j = 0; j < 3; j++)
        {
            hessianMatrix.insert(i,j) = H(i,j);
        }
    }
}
void set_param(){

  Eigen::Matrix<double,3,3> h;
  h << 1,-1,1,-1,2,-2,1,-2,4;
    //   Eigen::Matrix<double,5+1,5+1> ff;
    // ff << 1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6;


    // H2hessian(ff,hessian);
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
  // H2hessian(h,hessian);
 // std::cout << "hessian:" << std::endl << hessian << std::endl;


  gradient.resize(3);
  gradient << 2, -3, 1;

 // std::cout << "gradient:" << std::endl << gradient << std::endl;


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


  lowerBound.resize(4);
  lowerBound << 0, 0, 0, 0.5;
 // std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;


  upperBound.resize(4);
  upperBound << 1, 1, 1, 0.5;
//  std::cout << "upperBound:" << std::endl << upperBound << std::endl;


}

void solve_osqp(){

  OsqpEigen::Solver solver;

  // settings
  //solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  //矩阵A为m*n矩阵
  solver.data()->setNumberOfVariables(NumberOfVariables);     //设置A矩阵的列数，即n
  solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
  solver.data()->setHessianMatrix(hessian);
  solver.data()->setGradient(gradient);
  solver.data()->setLinearConstraintsMatrix(linearMatrix);
  solver.data()->setLowerBound(lowerBound);
  solver.data()->setUpperBound(upperBound);


  // instantiate the solver
  solver.initSolver();

    // solve the QP problem
  solver.solveProblem();
  QPSolution = solver.getSolution();
}

int main(int argc, char * argv[])
{
 ros::init(argc, argv, "osqp_demo");

  set_param();
  solve_osqp();


  std::cout << "QPSolution:" << std::endl
            << QPSolution << std::endl;
            
    return 0;
}
