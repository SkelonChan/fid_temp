#include "interpfun.h"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace std;
using namespace Eigen;
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
			//输出rowArray存入的数据
			//cout << rowArray[0] << " " << rowArray[1] << " " << rowArray[2] << " " << rowArray[3] << endl;
		}
		if (cols == i) //将txt文本文件中的一行数据存入rowArray中，并将rowArray存入vector中
			_vector.push_back(rowArray);
	}
	fclose(fp);
	return 1;
}




int main()
{

    string file ="425.txt";
	//txt文件中有7列
	int columns = 7;
	vector<double *> output_vector;
	if (!read_scanf(file, columns, output_vector))
	{
		return 0;
	}
	double t[423];
	double l[423];
	double y[423];
	double vx[423];
	double vy[423];
	double yaw[423];
	double yaw_rate[423];	
	//output_vector可视为二维数组;输出数组元素：
	int rows = output_vector.size();

	for (int i = 0; i < 423; i++)
	{
		t[i] = output_vector[i][0];
		l[i] = output_vector[i][1];
		y[i] = output_vector[i][2];
		vx[i] = output_vector[i][3];
		vy[i] = output_vector[i][4];
		yaw[i] = output_vector[i][5];
		yaw_rate[i] = output_vector[i][6];
	}

char *method = (char*)"spl";
double val;
Interp th;
th.interp_onePoint(l,y,output_vector.size()-1,91,&val,method);
cout<<"t=424,"<<"z = "<<val<<endl;


 return 0;

}
