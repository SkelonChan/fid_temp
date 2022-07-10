#include <ros.h>
#include <std_msgs/Int16.h>
#include <router2ros/router2ros.h>

ros::NodeHandle nh;
router2ros::router2ros value;
const int motor_rec = 2;// 电机  中断口0
const int servo_rec = 3;//舵机 由遥控器接收到信息 中断口1

volatile int pwm_value = 0; 
//Arduino docs 强调在有并发执行的线程（如中断）情况下，变量声明必许使用volatile
volatile int prev_time = 0;
volatile int prev_time1 = 0;
volatile int pwm_value1 = 0; 


ros::Publisher pub_base_control("base_value",&value);


void setup()
{
    nh.initNode();
    nh.advertise(pub_base_control);
    pinMode(motor_rec,INPUT);
    pinMode(servo_rec,INPUT);
    Serial.begin(57600);
    attachInterrupt(1, risingCallback, RISING); //启用上升中断
    attachInterrupt(0, risingCallback1, RISING); //启用上升中断
}

void loop()
{
  pub_base_control.publish(&value);
  delay(10);
  nh.spinOnce();
}

void risingCallback() {
  attachInterrupt(1, fallingCallback, FALLING);//启用下降中断
  prev_time = micros(); //开始计时
}
 
void fallingCallback() {
  attachInterrupt(1, risingCallback, RISING);
  pwm_value = micros()-prev_time;//低电平持续时间
  //Serial.println(pwm_value);
  value.motor_value = pwm_value - 12;
}

void risingCallback1() {
  attachInterrupt(0, fallingCallback1, FALLING);//启用下降中断
  prev_time1 = micros(); //开始计时
}
 
void fallingCallback1() {
  attachInterrupt(0, risingCallback1, RISING);
  pwm_value1 = micros()-prev_time1;//低电平持续时间
  //Serial.println(pwm_value1+1000);
  value.servo_value = pwm_value1 - 24;

}
