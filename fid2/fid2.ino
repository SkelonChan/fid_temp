#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>


ros::NodeHandle nh;

int pos = 0;    // 角度存储变量
Servo myservo;
Servo mymotor;
const int MOTOR = 5;//定义舵机接口数字接口9
const int SERVO = 10;
int motor_speed = 1500;
//int motor_speed = 1500;
int servo_angle = 90;


//servo 是作映射，0-180 映射单周期高电平 0.5ms-2.5ms
//电调的控制周期为20ms,高电平 1ms反向最大，1.5终点0位 2ms正转最大
//1500中点，1000反向最大，2000正向最大
//正向启动阈值约在1525附近，反向在1475附近，1475--1525区间，齿轮响动异常


void motor_cb(std_msgs::Int16 input)
  {
    motor_speed = input.data;
  }
void servo_cb(std_msgs::Int16 servo_input)
  {
    servo_angle = servo_input.data;
  }

    ros::Subscriber<std_msgs::Int16> submotor("/motor",&motor_cb);
    ros::Subscriber<std_msgs::Int16> subservo("/servo",&servo_cb);


void runcommand(){
    myservo.write(servo_angle);
    mymotor.writeMicroseconds(motor_speed);
    //setmotor(motor_speed);
}

void setup()
{
    nh.initNode();
    nh.subscribe(submotor);
    nh.subscribe(subservo);
    //pinMode(MOTOR,OUTPUT);//设定输出接口
    myservo.attach(SERVO);
    mymotor.attach(MOTOR);
}

void loop()
{
runcommand();
nh.spinOnce();
}
