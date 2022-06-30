#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
#include <router2ros/router2ros.h>

ros::NodeHandle nh;
router2ros::router2ros value;
int pos = 0;    // 角度存储变量
Servo myservo;
Servo mymotor;
const int MOTOR = 5;//定义舵机接口数字接口9
const int SERVO = 10;
int motor_speed = 1500;
//int motor_speed = 1500;
int servo_angle = 90;

const int motor_rec = 18;// 电机  中断口5
const int servo_rec = 19;//舵机 由遥控器接收到信息 中断口4

volatile int pwm_value = 0; 
//Arduino docs 强调在有并发执行的线程（如中断）情况下，变量声明必许使用volatile
volatile int prev_time = 0;
volatile int prev_time1 = 0;
volatile int pwm_value1 = 0; 


ros::Publisher pub_base_control("base_value",&value);



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
    nh.advertise(pub_base_control);
    pinMode(motor_rec,INPUT);
    pinMode(servo_rec,INPUT);
    Serial.begin(57600);
    nh.subscribe(submotor);
    nh.subscribe(subservo);
    //pinMode(MOTOR,OUTPUT);//设定输出接口
    myservo.attach(SERVO);
    mymotor.attach(MOTOR);
    attachInterrupt(5, risingCallback, RISING); //启用上升中断
    attachInterrupt(4, risingCallback1, RISING); //启用上升中断
}

void loop()
{
  pub_base_control.publish(&value);
  runcommand();
  nh.spinOnce();
}

void risingCallback() {
  attachInterrupt(5, fallingCallback, FALLING);//启用下降中断
  prev_time = micros(); //开始计时
}
 
void fallingCallback() {
  attachInterrupt(5, risingCallback, RISING);
  pwm_value = micros()-prev_time;//低电平持续时间
  //Serial.println(pwm_value);
  value.motor_value = pwm_value - 12;
}

void risingCallback1() {
  attachInterrupt(4, fallingCallback1, FALLING);//启用下降中断
  prev_time1 = micros(); //开始计时
}
 
void fallingCallback1() {
  attachInterrupt(4, risingCallback1, RISING);
  pwm_value1 = micros()-prev_time1;//低电平持续时间
  //Serial.println(pwm_value1+1000);
  value.servo_value = pwm_value1 + 8;

}
