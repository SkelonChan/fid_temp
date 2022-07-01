#ifndef INTERPFUN_H
#define INTERPFUN_H
#include"math.h"
#include<stdio.h>
#include <string.h> 
#include <iostream>
#ifndef SafeDeleteVec
#define SafeDeleteVec(X) { if((X)) delete (X); (X)=NULL;}
#endif

using namespace std;
class Interp
{
public:

char *strMethod1;
char *strMethod2;
char *strMethod3;

public:
Interp();
~Interp();
/*****以下两个函数为插值函数**/
void interp_onePoint(double *x,double *y,int n,double t,double *fval,char *method);//返回单个点的值
void interp_multiPoint(double *x,double *y,int n,double *t,double *fval,int m,char *method);//返回多个点的值,点的个数为m

 

double interp2_onePoint(double *x,double *y,double *z,int m,int n,double a,double b,char *method);//matlab interp2 实现 2D面数据插值
void interp2_multiPoint(double *x,double *y,double *z,int m,int n,double *a,double *b,int asize,int bsize,double *fval,char *method);//matlab interp2 实现 2D面向量插值

private:
//拉格朗日线性插值
double lgr(double *x,double *y,int n,double t);
//拉格朗日一元三点线性插值
double lg3(double *x,double *y,int n,double t);
//平滑插值 三次多项式插值
void spl(double *x,double *y,int n,int k,double t,double *s);

int findIndex(double *x,int t,int n);

};
#endif

Interp::Interp()
{
strMethod1 =(char*)"lgr";
strMethod2 = (char*)"lg3";
strMethod3 = (char*)"spl";
}

Interp::~Interp()
{

}

double Interp::lgr(double *x,double *y,int n,double t)
{
int i,j,k,m;
double z,s;
z= 0.0f;
if(n<1)
return(z);
if(n==1)
{
z=y[0];
return(z);
}

if(n==2)
{
z=(y[0]*(t-x[1]) - y[1]*(t-x[0]))/(x[0]-x[1]);
return(z);

}
if(n>1 && t>x[n-1]){
z= y[n-1];
return(z);
}
if(n>1 && t<x[0]){
z= y[0];
return(z);
}

i =0;
while((x[i] <t) && (i<n)) {
i=i+1;
}

k = i-4;
if(k<0) 
k=0;
m = i+3;
if(m>n-1) 
m = n-1;
for( i = k;i<=m;i++)
{
s=1.0;
for( j =k;j<=m;j++){
if(j!=i) 
s= s*(t-x[j])/(x[i]-x[j]);
}
z = z+s*y[i];
}
return(z);

}

//拉格朗日一元三点插值
double Interp::lg3(double *x,double *y,int n,double t)
{
int i,j,k,m;
double z,s;
z= 0.0;
if(n<1) return (z);
if(n ==1) {z= y[0];return(z);}
if(n==2)
{
z = (y[0]*(t-x[1]) - y[1]*(t - x[0]))/(x[0] - x[1]);
return(z);
}
if(n>0&&t<x[0]){
z = y[0];
return(z);
}
if(n>0 && t>x[n-1]){
z = y[n-1];
return(z);
}

if(t<=x[1]) 
{
k =0;
m=2;
}
else if(t>=x[n-2])
{
k = n-3;
m = n-1;
}
else 
{
k =1;
m =n;
while ((m-k) !=1)
{
i = (k+m)/2;
if(t<x[i-1])
m =i;
else
k =i;
}
k = k-1;
m = m-1;
if(fabs(t-x[k]) <fabs(t- x[m]))
k = k-1;
else
m = m+1;
}
z = 0.0;
for(i =k;i<=m;i++)
{
s= 1.0;
for(j = k;j<=m;j++)
{
if(j!=i)
s = s*(t-x[j])/(x[i] - x[j]);
}
z = z+ s*y[i];

}
return (z);

}

//平滑插值 三次多项式插值
void Interp::spl(double *x,double *y,int n,int k,double t,double *s)
{
int kk, m,lc;

double u[5], p,q;

s[4] = 0.0f;
s[0] = 0.0f;
s[1] = 0.0f;
s[2] = 0.0f;
s[3] = 0.0f;

if(n<1)
return;
if(n == 1)
{
s[0] = y[0];
s[4] = y[0];
return;
}
if(n == 2)
{
s[0] = y[0];
s[1] = (y[1] - y[0])/(x[1] - x[0]);
if(k<0)
s[4] = (y[0]*(t - x[1]) - y[1]*(t - x[0]))/(x[0] - x[1]);
return;
}

if(k<0 && n>0 && t<x[0] )
{

s[4] = y[0];
return;
}

if(k<0 && n>0 && t>x[n-1] )
{

s[4] = y[n-1];
return;
}

if(k<0)
{

if(t<=x[1]) 
kk =0;
else if(t>=x[n-1])
kk = n-2;
else
{

kk=1;
m = n;
while (((kk -m) !=1) && ((kk - m) != -1))
{
lc=(kk +m)/2;
if(t<x[lc-1]) 
m = lc;
else
kk =lc;

}
kk = kk -1;
}
}
else
kk = k;

if( kk>n-1)
kk = n-2;

u[2] = (y[kk +1] - y[kk])/(x[kk +1] - x[kk]);
if (n == 3)
{
if(kk == 0)
{
u[3] = (y[2] - y[1])/(x[2]-x[1]);
u[4] = 2.0*u[3] - u[2];
u[1] = 2.0*u[2] - u[3];
u[0] = 2.0*u[1] - u[2];
}
else
{
u[1] = (y[1] - y[0])/(x[1] - x[0]);
u[0] = 2.0*u[1] - u[2];
u[3] = 2.0*u[2] - u[1];
u[4] = 2.0*u[3] - u[2];
}

}
else
{
if(kk <=1)
{
u[3] = (y[kk+2] - y[kk +1])/(x[kk+2]-x[kk+1]);
if(kk ==1)
{
u[1] = (y[1] -y[0])/(x[1] - x[0]);
u[0] = 2.0*u[1] - u[2];
if(n == 4)
u[4] = 2.0*u[3] - u[2];
else
u[4] = (y[4]-y[3])/(x[4] - x[3]);
}
else
{
u[1] = 2.0*u[2] - u[3];
u[0] = 2.0*u[1] - u[2];
u[4] = (y[3]-y[2])/(x[3] - x[2]);
}

}
else if(kk >=(n-3))
{
u[1] = (y[kk]-y[kk-1])/(x[kk] - x[kk-1]);
if(kk == (n-3))
{
u[3] = (y[n-1] - y[n-2])/(x[n-1] - x[n-2]);
u[4] = 2.0*u[3] - u[2];
if( n== 4)
u[0] = 2.0*u[1] - u[2];
else 
u[0] = (y[kk-1] - y[kk-2])/(x[kk-1] - x[kk-2]);

}
else
{
u[3] = 2.0*u[2] - u[1];
u[4] = 2.0*u[3] - u[2];
u[0] = (y[kk -1] - y[kk-2])/(x[kk-1] - x[kk -2]);
}

}
else
{
u[1] = (y[kk] - y[kk -1])/(x[kk] - x[kk-1]);
u[0] = (y[kk-1] - y[kk -2])/(x[kk-1] - x[kk-2]);
u[3] = (y[kk+2] - y[kk +1])/(x[kk+2] - x[kk +1]);
u[4] = (y[kk+3] - y[kk +2])/(x[kk+3] - x[kk +2]);
}

}

s[0] = fabs(u[3] - u[2]);
s[1] = fabs(u[0] - u[1]);
if( (s[0] + 1.0 == 1.0) && (s[1] + 1.0 == 1.0))
p = (u[1] + u[2])/2.0;
else
p = (s[0]*u[1] + s[1]*u[2])/(s[0] + s[1]);
s[0] = fabs(u[3] - u[4]);
s[1] = fabs(u[2] - u[1]);
if((s[0] + 1.0 == 1.0) && (s[1] + 1.0 == 1.0))
q = (u[2]+u[3])/2.0;
else 
q= (s[0]*u[2] + s[1]*u[3])/(s[0] + s[1]);

s[0] = y[kk];
s[1] = p;
s[3] = x[kk +1] - x[kk];
s[2] = (3.0*u[2] - 2.0*p - q)/s[3];
s[3] = (p + q - 2.0*u[2])/(s[3]*s[3]);
if (k<0)
{
p = t-x[kk];
s[4] = s[0] + s[1]*p + s[2]*p*p + s[3]*p*p*p;

}
return;

}

double Interp::interp2_onePoint(double *x,double *y,double *z,int m,int n,double a,double b,char *method)
{

//find a，b的位置
int tempi,tempj;
double w1,w2,w;
double tempx[2] = {0,0};
double tempz[2] = {0,0};
/*if(a>x[m-1] || b>y[n-1] || a<x[0] || b<y[0])
{
w = 0.0f;

}
else
{*/

tempi = findIndex(x,a,m);
tempj = findIndex(y,b,n);
if(tempi<0)
tempi =0;
if(tempi>m-2)
tempi = m-2;
if(tempj<0)
tempj = 0;
if(tempj>n-2)
tempj = n-2;

tempx[0] = x[tempi];
tempx[1] = x[tempi+1];
tempz[0] = z[tempj*m + tempi];
tempz[1] = z[tempj*m + tempi + 1];

interp_onePoint(tempx,tempz,2,a,&w1,method);
//update tempz
tempz[0] = z[(tempj+1)*m + tempi];
tempz[1] = z[(tempj+1)*m + tempi + 1];
interp_onePoint(tempx,tempz,2,a,&w2,method);

//update tempx and tempz
tempx[0] = y[tempj];
tempx[1] = y[tempj+1];

tempz[0] = w1;
tempz[1] = w2;

interp_onePoint(tempx,tempz,2,b,&w,method);
//}
return(w);
}

void Interp::interp2_multiPoint(double *x,double *y,double *z,int m,int n,double *a,double *b,int asize,int bsize,double *fval,char *method)
{
//grid nets
for(int i=0;i<asize;i++)
{
for(int j = 0;j<bsize;j++)
*(fval+j*asize+i) = interp2_onePoint(x,y,z,m,n,a[i],b[j],method);
}

}

/*
void Interp::interp2_multiPoint(double *x,double *y,double *z,int m,int n,double *gridx,double *gridy,int asize,int bsize,double *fval,char *method)
{
//grid nets
for(int i=0;i<asize;i++)
{
for(int j = 0;j<bsize;j++)
*(fval+j*asize+i) = interp2_onePoint(x,y,z,m,n,gridx[j*asize + i],gridx[j*asize + i],method);
}

}*/

int Interp::findIndex(double *x,int t,int n)
{
int kk,m,lc;
/*if(t<x[0])
return( 0);
if(t>=x[n-1])
return(n-1);*/

if(t<=x[1]) 
kk =0;
else if(t>=x[n-1])
kk = n-2;
else
{

kk=1;
m = n;
while (((kk -m) !=1) && ((kk - m) != -1))
{
lc=(kk +m)/2;
if(t<x[lc-1]) 
m = lc;
else
kk =lc;

}
kk = kk -1;
}

return(kk);

}

void Interp::interp_onePoint(double *x,double *y,int n,double t,double *fval,char *method)
{

if(nullptr == method)
*fval = lgr(x,y,n,t);
else if(strcmp(method,strMethod1) == 0)
*fval = lgr(x,y,n,t);
else if(strcmp(method,strMethod2) == 0)
*fval = lg3(x,y,n,t);
else if(strcmp(method,strMethod3) == 0)
{
static double s[5];
spl(x,y,n,-1,t,s);
*fval = s[4];
}
else
*fval = lgr(x,y,n,t);
}

void Interp::interp_multiPoint(double *x,double *y,int n,double *t,double *fval,int m,char *method)
{
if(t == NULL)
return;
double tempVal = 0.0;;
for(int k = 0;k<m;k++){

interp_onePoint(x,y,n,t[k],&tempVal,method);
fval[k] = tempVal;
}

}

