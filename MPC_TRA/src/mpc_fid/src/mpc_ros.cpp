#include "interpfun.h"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>
//#include <mpc_fid/Array.h>
#include <vehicle_info_msg/gp_msg.h>
#include <vehicle_info_msg/vehicle_control_msg.h>
#include <vehicle_info_msg/vehicle_info_msg.h>
#include <vehicle_info_msg/GPRdata.h>
#include <vehicle_info_msg/ref_path.h>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <Eigen/Jacobi>
#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>

using namespace Eigen;
using namespace std;

vehicle_info_msg::vehicle_control_msg cmd;//初始化要发送的位置信息格式

int U = 0;//设置全局变量保存delta_f

bool add_gp = false;//判断是否添加gp
double ref_speed = 0;

const int Np = 20;//predict horizon
const int Nc = 5;//control horizon
const int Ny = 2;//output number
const int Nx = 6;//states number
const int Nu = 1; //control number
const int Row = 1000; //松弛因子

//reference info，写死了
double t_ref[423];
double l_ref[423];
double y_ref[423];
double vx_ref[423];
double vy_ref[423];
double yaw_ref[423];
double yaw_rate_ref[423];	

Eigen::SparseMatrix<double> hessian;
Eigen::VectorXd gradient;
Eigen::SparseMatrix<double> linearMatrix;
Eigen::VectorXd lowerBound;
Eigen::VectorXd upperBound;

char *method = (char*)"spl";
double val;
Interp th;

// 功能：将filename 中的数据（共cols列）读取到_vector中，_vector可视为二维数组
int read_scanf(const string &filename, const int &cols, vector<double *> &_vector)
{
	FILE *fp = fopen(filename.c_str(), "r");
	bool flag = true;
	int i = 0;
	if (!fp) 
	{ 
		cout << "File open error!\n"; 
		return 0; 
	}
 
	while (flag)
	{
		double *rowArray = new double[cols]; //new一个double类型的动态数组
 
		for (i = 0; i < cols; i++) //读取数据，存在_vector[cols]中
		{
			if (EOF == fscanf(fp,"%lf", &rowArray[i]))
			{ 
				flag = false; 
				break; 
			}
		}
		if (cols == i) //将txt文本文件中的一行数据存入rowArray中，并将rowArray存入vector中
			_vector.push_back(rowArray);
	}
	fclose(fp);
	return 1;
}

void setgradient(const Eigen::Matrix<double,Nu*Nc+1,1> &f,Eigen::VectorXd &gradient){
    gradient = Eigen::VectorXd::Zero(Nu*Nc+1,1);
    for (int i = 0;i < Nu*Nc+1;i++)
    {
        gradient(i,0) = f(i,0);
    }
     
 }
void setlowerbound(const Eigen::Matrix<double,Nc+1,1> &lb,Eigen::VectorXd &lowerbound){
    lowerbound = Eigen::MatrixXd::Zero(2*Nc+Nc+1,1);
    for (int i =0;i<2*Nc;i++)
    {
        lowerbound(i,0) = -OsqpEigen::INFTY;
    }
    for(int i = 2*Nc;i<2*Nc+Nc+1;i++)
    {
        lowerbound(i,0) = lb(i-2*Nc,0);
    }
}
void setupperbound(const Eigen::Matrix<double,Nc+1,1> &ub,const Eigen::Matrix<double,2*Nc,1> b_cons,Eigen::VectorXd &upperbound){
    upperbound = Eigen::MatrixXd::Zero(2*Nc+Nc+1,1);
    for (int i =0;i<2*Nc;i++)
    {
        upperbound(i,0) = b_cons(i,0);
    }
    for(int i = 2*Nc;i<2*Nc+Nc+1;i++)
    {
        upperbound(i,0) = ub(i-2*Nc,0);
    }
}

Eigen::Matrix<double,Nx+Nu,Nx+Nu> powerA(Eigen::Matrix<double,Nx+Nu,Nx+Nu> a,int n)
{
    //很多不完善，没有考虑，仅作做幂次方的矩阵,n大于1
    Eigen::Matrix<double,Nx+Nu,Nx+Nu> b;
    b << a;
    for (int i = 1 ;i<n;i++)
    {
        b = b*a;
    }
    return b;
}

//从得到的H矩阵一次向稀疏的海塞矩阵赋值，不能直接赋值H = hessian
void H2hessian(const Eigen::Matrix<double,Nu*Nc+1,Nu*Nc+1> &H,Eigen::SparseMatrix<double> &hessianMatrix)
{
    hessianMatrix.resize(Nu*Nc+1,Nu*Nc+1);
    for (int i = 0; i < Nu*Nc+1;i++)
    {
        for(int j = 0; j < Nu*Nc+1; j++)
        {
            hessianMatrix.insert(i,j) = H(i,j);
        }
    }
}

//从得到的A_cons向线性矩阵进行填充上半部分为A_cons,下半部分为eye(nc+1)
void setlinearMatrix(const Eigen::Matrix<double,2*Nu*Nc,Nu*Nc+1> &A_cons,Eigen::SparseMatrix<double> &linearMatrix)
{
    linearMatrix.resize(2*Nu*Nc+Nc+1,Nu*Nc+1);
    //对linearMatrix上半部分A_cons赋值
    for(int i = 0;i< 2*Nu*Nc;i++)
    {
        for(int j =0;j<Nu*Nc+1;j++)
        {
            linearMatrix.insert(i,j) = A_cons(i,j);
        }
    }
    //对linearMatrix下半部分eye(Nc+1)赋值
    Eigen::Matrix<double,Nc+1,Nc+1> eye_6 = Eigen::Matrix<double,Nc+1,Nc+1>::Identity();
    for(int i=2*Nu*Nc;i< 2*Nu*Nc+Nc+1;i++)
    {
        for(int j = 0; j< Nu*Nc+1;j++)
        {
            linearMatrix.insert(i,j) = eye_6(i-2*Nu*Nc,j);
        }
    }
}
//设置初始的x初值x0 = [0.01;0.01;0.01;0.01;0.01;0.01]
//Eigen::Matrix<double,Nc+1,1> x0 = 0.01 * Eigen::Matrix<double,Nc+1,1>::Ones();

void read_ref_data()
{
    //ros中读取文件位置，绝对地址
    string file ="/home/skelon/demo/MPC_ros/src/mpc_fid/src/425.txt";
	//txt文件中有7列
	int columns = 7;
	vector<double *> output_vector;
	if (!read_scanf(file, columns, output_vector))
	{
		cout << "open file faild!!" << endl;
	}
	//output_vector可视为二维数组;输出数组元素：
	int rows = output_vector.size();

	for (int i = 0; i < 423; i++)
	{
		t_ref[i] = output_vector[i][0];
		l_ref[i] = output_vector[i][1];
		// y_ref[i] = output_vector[i][2];
		vx_ref[i] = output_vector[i][3];
		vy_ref[i] = output_vector[i][4];
		yaw_ref[i] = output_vector[i][5];
		// yaw_rate_ref[i] = output_vector[i][6];
	}
}



class mpc_follow
{
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _vehicle_info_sub;//实时传入车辆的定位，速度加速度，phi,phi_rate等信息
        ros::Subscriber _gp_com_sub;//
    //    ros::Subscriber _ref_path_sub;//参考路径信息，可以写死
        ros::Publisher _vehicle_control_pub;

        const double mass = 30.24;//kg
        const double g = 9.8;
        const double lf = 0.28;//质心到前轮轴长度 m
        const double lr = 0.34;//质心到后轮轴长度m
        const double l = 0.62;//前后轮长度 m
        const double Iz = 1.7465;//kgm^
	const double wid = 0.47;//左右轮距m
	const double dd = 0.23;//轮胎直径 m
	const double ll = 1; //车体长度 m
	const double ww = 0.55;//车体宽度m
	const double hh = 0.4;//车体高度 m
	const double ccf = 669;//Fake!
        const double ccr = 600;//Fake!
        const double clf = 669;//Fake!
        const double clr = 600;//Fake!
        //const double ts = 0.03;//[s]
        const double T = 0.03;//

        const double sf = -0.34;//fake data
        const double sr = 0.04;//fake data 车辆前后轮的滑移率

        double y_dot;
        double x_dot;
        double phi;
        double phi_dot;
        double x_pos;
        double y_pos;
        double delta_f;
        //gp_com_info
        double mean_vx;
        double mean_vy;
        double mean_yaw_rate;

        Matrix<double,Nx,Nx> a;

        Matrix<double,Nx,Nu> b;
    public:
        mpc_follow();
        ~mpc_follow();
        void vehicle_info_cb(const vehicle_info_msg::vehicle_info_msg::ConstPtr &p);
        void gp_com_cb(const vehicle_info_msg::GPRdata::ConstPtr &p);
        //void ref_path_cb(const vehicle_info_msg::ref_path::ConstPtr &p);
        void mpc_core();
    
};

mpc_follow::mpc_follow()
{
    void read_ref_data();
 //   _ref_path_sub = _nh.subscribe<vehicle_info_msg::ref_path>("/ref_path",1,&mpc_follow::ref_path_cb,this);
    if(add_gp)
        _gp_com_sub = _nh.subscribe<vehicle_info_msg::GPRdata>("/gp_com",1,&mpc_follow::gp_com_cb,this);
    _vehicle_info_sub = _nh.subscribe<vehicle_info_msg::vehicle_info_msg>("/vehicle_info",1,&mpc_follow::vehicle_info_cb,this);
    _vehicle_control_pub = _nh.advertise<vehicle_info_msg::vehicle_control_msg>("/vehicle_control",1);

}
mpc_follow::~mpc_follow(){

}

// void mpc_follow::ref_path_cb(const vehicle_info_msg::ref_path::ConstPtr &p)
// {
//     vx_ref = p->vx_ref;
//     vy_ref = p->vy_ref;
//     phi_ref = p->phi_ref;
//     phi_rate_ref = p->phi_rate_ref;
//     x_ref = p->x_ref;
//     y_ref = p->y_ref;

// }

void mpc_follow::vehicle_info_cb(const vehicle_info_msg::vehicle_info_msg::ConstPtr &p)
{
    y_dot = p->y_dot;
    x_dot = p->x_dot;
    phi = p->phi;
    phi_dot = p->phi_dot;
    x_pos = p->x_pos;
    y_pos = p->y_pos;
    //delta_f = 0;///this need to update every time.run the callback_function
    if (!add_gp)
    {
        mean_vx = 0;
        mean_vy = 0;
        mean_yaw_rate = 0;
    }
    mpc_core();//此函数计算出前轮的转角信息
    cmd.steer_angle = U * 180 /3.141;
    // cmd.vx = 3;//先使用固定速度
    cmd.vx = ref_speed;
    //发送数据
    _vehicle_control_pub.publish(cmd);


}

void mpc_follow::gp_com_cb(const vehicle_info_msg::GPRdata::ConstPtr &p)
{
    mean_vx = p->mean_vx;
    mean_vy = p->mean_vy;
    mean_yaw_rate = p->mean_yaw_rate;
}

//以下部分为mpc_core的实现部分
//*********************************************************************************************************************
void mpc_follow::mpc_core()
{
    a << 1,1,1,1,1,1,
         1,1,1,1,1,1,
         1,1,1,1,1,1,
         1,1,1,1,1,1,
         1,1,1,1,1,1,
         1,1,1,1,1,1;
    b << 1,1,1,1,1,1; //TO DO
    //A B矩阵是状态空间方程的系数矩阵
    //A B 矩阵准确将需要每次都要重新计算
    //设置权重矩阵Q
    Eigen::Matrix<double,Np*Ny,Np*Ny> Q = Eigen::Matrix<double,Np*Ny,Np*Ny> ::Zero();
    for(int i = 0;i < Np*Ny; i++){
        for (int j = 0; j < Np*Ny; j++)
        {
            if (i==j){
                Q(i,j) = 3000;
                if (i % 2 != 0 )
                    Q(i,j) = 5000;
            }

        }    
    }

    //设置权重矩阵R
    Eigen::Matrix<double,Nu*Nc,Nu*Nc> R = Eigen::Matrix<double,Nu*Nc,Nu*Nc>::Identity();
    R = 5000 * R;

    delta_f = U;//将上一步计算的delta_f值传给U，第一次则传初值
    //定义kesi向量
    Eigen::Matrix<double,Nx+Nu,1> kesi;
    kesi(0) = y_dot;
    kesi(1) = x_dot;
    kesi(2) = phi;
    kesi(3) = phi_dot;
    kesi(4) = y_pos;
    kesi(5) = x_pos;
    kesi(6) = delta_f;//此处的delta_f应该是上一步长的输出量

    Eigen::Matrix<double,Nx,1> state_k1;
    state_k1(0) = y_dot + T * (-x_dot*phi_dot+2*(ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+ccr*(lr*phi_dot-y_dot)/x_dot)/mass) - mean_vy;
    state_k1(1) = x_dot + T *(y_dot*phi_dot+2*(clf*sf+clr*sr+ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/mass) - mean_vx;
    state_k1(2) = phi + T * phi_dot;
    state_k1(3) = phi_dot + T * ((2*lf*ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*ccr*(lr*phi_dot-y_dot)/x_dot)/Iz) - mean_yaw_rate;
    state_k1(4) = y_pos + T * (x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(5) = x_pos + T * (x_dot*cos(phi)-y_dot*sin(phi));   

    Eigen::Matrix<double,Nx,Nu> d_k = state_k1 - a * kesi.topLeftCorner(6,1) - b * kesi.row(6);

    Eigen::Matrix<double,Nx+Nu,1> d_piao_k  = Eigen::Matrix<double,Nx+Nu,1>::Zero();
    d_piao_k.topLeftCorner(6,1) = d_k;
    //d_piao_k.row(6) << 0;

    //set A   TO DO 此处对A矩阵的赋值还可以用bolck进行重写，避免新建矩阵和多余的赋值加速操作
    Eigen::Matrix<double,Nx+Nu,Nx+Nu> A;
    Eigen::Matrix<double,Nx,Nx+Nu> A1;
    A1 << a,b;
    Eigen::Matrix<double,Nu,Nx+Nu> A2;
    Eigen::Matrix<double,Nu,Nx> A2_sub1 = Eigen::Matrix<double,Nu,6>::Zero();
    Eigen::Matrix<double,Nu,Nu> A2_sub2 = Eigen::Matrix<double,Nu,Nu>::Identity();
    A2 << A2_sub1,A2_sub2;
    A << A1,
         A2;

    //set B
    Eigen::Matrix<double,Nx+Nu,Nu> B;
    Eigen::Matrix<double,Nu,Nu> B1 = Eigen::Matrix<double,Nu,Nu>::Identity();
    B << b,B1;

    //set C  输出矩阵
    Eigen::Matrix<double,Ny,Nu+Nx> C = Eigen::Matrix<double,Ny,Nu+Nx>::Zero();
    C(0,2) = 1;
    C(1,4) = 1;
    Eigen::Matrix<double,Np*(Nx+Nu),1> PHI;
    Eigen::Matrix<double,Np*Ny,Nu+Nx> PSI;
    Eigen::Matrix<double,Np*Ny,Np*(Nx+Nu)> GAMMA;
    Eigen::Matrix<double,Np*Ny,Nc*Nu> THETA;//GAMMA &THETA 可以直接定义为零矩阵，只对上三角赋值


    for (int p = 0;p < Np; p++){
        PHI.block<Nx+Nu,1>(p*(Nx+Nu),0) = d_piao_k;
        PSI.block<Ny,Nx+Nu>(p*(Ny),0) = C * powerA(A,p+1);
        for(int q=0;q<Np;q++){
            if(q<=p)//下三角矩阵
                GAMMA.block<Ny,Nx+Nu>(p*Ny,q*(Nx+Nu)) = C * powerA(A,p-q);//自定义A的幂次方矩阵
            else//上三角矩阵
                GAMMA.block<Ny,Nx+Nu>(p*Ny,q*(Nx+Nu)) = Eigen::Matrix<double,Ny,Nx+Nu>::Zero();
        }
        for(int k = 0; k< Nc;k++){
            if(k<=p)
                THETA.block<Ny,Nu>(p*(Ny),k*(Nu)) = C * powerA(A,p-k) * B;
            else
                THETA.block<Ny,Nu>(p*(Ny),k*(Nu)) = Eigen::Matrix<double,Ny,Nu>::Zero();
        }
    }


    //set H matrix
    Eigen::Matrix<double,(Nu*Nc+1),(Nu*Nc+1)> H;
    H.block<Nu*Nc,Nu*Nc>(0,0) = 2 *(THETA.adjoint() * Q * THETA + R);
    H.block<Nu*Nc,1>(0,Nu*Nc) = Eigen::Matrix<double,Nu*Nc,1>::Zero();
    H.block<1,Nu*Nc>(Nu*Nc,0) = Eigen::Matrix<double,1,Nu*Nc>::Zero();
    H.block<1,1>(Nu*Nc,Nu*Nc)  <<  2 * Row;

    H = (H.adjoint() + H )/2;
    //对车辆未来时域内的状态进行预测
    Eigen::Matrix<double,Ny*Np,1> error_1;
    Eigen::Matrix<double,Ny*Np,1> Yita_ref;
    Eigen::Matrix<double,Ny*Np,1> X_predict;

    for (int p = 0; p < Np; p++)
    {

        double X_DOT = x_dot * cos(phi) - y_dot * sin(phi);
        X_predict(p,0) = x_pos + X_DOT * p * T;
        if (X_predict(p,0) > t_ref[423])//x_predict 大于规划的时间序列
        {
            Yita_ref(2*p,0) = 0;
            Yita_ref(2*p+1,0) = 0;
        }
    //此处需要填入当前时刻向后推Np步的参考轨迹中phi_ref, y_pos_ref
        else
        {
            th.interp_onePoint(l_ref,y_ref,423,X_predict(p,0),&val,method);
            double y_pos_ref,phi_ref;
            y_pos_ref = val;
            th.interp_onePoint(l_ref,yaw_ref,423,X_predict(p,0),&val,method);
            phi_ref = val;
            Yita_ref(2*p,0) = phi_ref;
            Yita_ref(2*p+1,0) = y_pos_ref;            
        }
        th.interp_onePoint(l_ref,vx_ref,423,X_predict(p,0),&val,method);
        ref_speed = val;
    }

    error_1 = Yita_ref - PSI*kesi - GAMMA * PHI;
    Eigen::Matrix<double,1,Nu*Nc+1> g_cell;
    g_cell.block<1,Nu*Nc>(0,0) =   2 * error_1.adjoint() * Q * THETA;
    g_cell.block<1,1>(0,Nu*Nc) << 0;

    Eigen::Matrix<double,Nu*Nc+1,1> f = g_cell.adjoint();


    //约束生成域：
    //---------------***********************
    Eigen::Matrix<double,Nc,Nc> A_t = Eigen::Matrix<double,Nc,Nc>::Zero();
    for(int p = 0;p<Nc; p++){
        for(int q = 0;q<Nc;q++){
            if(q<=p)
                A_t(p,q) = 1;
        }
    }

    Eigen::Matrix<double,Nu*Nc,Nu*Nc> A_I = Eigen::kroneckerProduct(A_t,Eigen::Matrix<double,Nu,Nu>::Identity());

    Eigen::Matrix<double,Nc,1> Ut = Eigen::kroneckerProduct(Eigen::Matrix<double,Nc,1>::Ones(),U*Eigen::Matrix<double,1,1>::Ones());

    double umin = -0.3744;
    double umax = 0.3744;
    Eigen::Matrix<double,Nc,1> Umin = Eigen::kroneckerProduct(Eigen::Matrix<double,Nc,1>::Ones(),umin * Eigen::Matrix<double,1,1>::Ones());
    Eigen::Matrix<double,Nc,1> Umax = Eigen::kroneckerProduct(Eigen::Matrix<double,Nc,1>::Ones(),umax * Eigen::Matrix<double,1,1>::Ones());

    //输出量进行约束
    Eigen::Matrix<double,2,1> ycmax;
    ycmax << 1,4;
    Eigen::Matrix<double,2,1> ycmin;
    ycmin << -1,-1;

    Eigen::Matrix<double,2*Np,1> Ycmax = Eigen::kroneckerProduct(Eigen::Matrix<double,Np,1>::Ones(),ycmax);
    Eigen::Matrix<double,2*Np,1> Ycmin = Eigen::kroneckerProduct(Eigen::Matrix<double,Np,1>::Ones(),ycmin);


    //quadprog A matrix
    //不等式约束中X的系数矩阵 A_cons
    Eigen::Matrix<double,2*Nu*Nc,Nu*Nc+1> A_cons;
    A_cons.block<Nu*Nc,Nu*Nc>(0,0) = A_I;
    A_cons.block<Nu*Nc,Nu*Nc>(Nu*Nc,0) = -A_I;
    A_cons.block<Nu*Nc,1>(0,Nu*Nc) = Eigen::Matrix<double,Nu*Nc,1>::Zero();
    A_cons.block<Nu*Nc,1>(Nu*Nc,Nu*Nc) = Eigen::Matrix<double,Nu*Nc,1>::Zero();
    
    //quadprog B matrix
    //不等式约束中的边界
    Eigen::Matrix<double,2*Nc,1> b_cons;
    b_cons.block<Nc,1>(0,0) = Umax - Ut;
    b_cons.block<Nc,1>(Nc,0) = -Umin + Ut;

    //the limited of control increment
    double M = 10;
    double delta_umin = -0.248;//前轮偏角变化量的约束下限
    double delta_umax = 0.248;//前轮偏角变化量的约束上限
    Eigen::Matrix<double,Nc,1> delta_Umax = Eigen::kroneckerProduct(Eigen::Matrix<double,Nc,1>::Ones(),delta_umax * Eigen::Matrix<double,1,1>::Ones());
    Eigen::Matrix<double,Nc,1> delta_Umin = Eigen::kroneckerProduct(Eigen::Matrix<double,Nc,1>::Ones(),delta_umin * Eigen::Matrix<double,1,1>::Ones());
    Eigen::Matrix<double,Nc+1,1> lb;
    Eigen::Matrix<double,Nc+1,1> ub;
    lb.block<Nc,1>(0,0) = delta_Umin;
    lb.block<1,1>(Nc,0) << 0;
    ub.block<Nc,1>(0,0) = delta_Umax;
    ub.block<1,1>(Nc,0) << 0;
    //约束部分测试通过
    //---*******************************************************

    //现在得到二次规划的各个矩阵，
    //  hessian ---H
    //  gradient -- f
    //  不等式约束中X的系数矩阵A_cons,
    //  不等式约束中边界约束值 b_cons
    //  x的下界 lb
    //  x的上界 ub

    setgradient(f,gradient);
    setlowerbound(lb,lowerBound);
    setupperbound(ub,b_cons,upperBound);

    H2hessian(H,hessian);
    //从得到的A_cons向线性矩阵进行填充上半部分为A_cons,下半部分为eye(nc+1)
    setlinearMatrix(A_cons,linearMatrix);


    //c++ Solver 中的求解部分
    OsqpEigen::Solver solver;
    //setting
    solver.settings()->setWarmStart(true);
    //将线性矩阵的维度传出，线性矩阵的行数即为总的约束维度，线性矩阵的列数即为变量个数
    int NumberOfVariables = linearMatrix.cols();
    int NumberOfConstraints = linearMatrix.rows();

    //向slover中传递待求解量的参数信息
    solver.data()->setNumberOfConstraints(NumberOfConstraints);
    solver.data()->setNumberOfVariables(NumberOfVariables);

    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);
    solver.initSolver();
    Eigen::VectorXd QPSolution;


    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
          cout << "solve the qp problems failed!" <<endl;  
          QPSolution = Eigen::Vector2d::Zero();
        }
    else
        QPSolution = solver.getSolution();

    if (isnan(QPSolution(0,0)))
        QPSolution(0,0) = 0;
    std::cout << "QPSolution is "<< std::endl << QPSolution << std::endl;
    
    U = QPSolution(0,0);


}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mpc_ros");

    ros::param::param<bool>("if_add_gp",add_gp,false);
    mpc_follow model1;
    ros::spin();
    return 0;
}
