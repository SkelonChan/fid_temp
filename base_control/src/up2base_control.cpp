#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "router2ros/router2ros.h" //自定义的baseremote信息


std_msgs::Int16 servo_msg;
std_msgs::Int16 motor_msg;
int pwm_limit;
int motor_pwm,servo_num;
bool error_flag = false;

class up2base_control
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber motor_sub;
    ros::Subscriber servo_sub;
    ros::Subscriber base_sub;
    ros::Publisher servo_pub;
    ros::Publisher motor_pub;
public:
    up2base_control();
    ~up2base_control();
    void motorcb(const std_msgs::Int16::ConstPtr &p);
    void servocb(const std_msgs::Int16::ConstPtr &p);
    void basecb(const router2ros::router2ros::ConstPtr &p);
};

up2base_control::up2base_control()
{
    base_sub = _nh.subscribe<router2ros::router2ros>("/base_value",1,&up2base_control::basecb,this);//由arduino底层接收到的遥控信号
    motor_sub = _nh.subscribe<std_msgs::Int16>("/motor",2,&up2base_control::motorcb,this);   
    servo_sub = _nh.subscribe<std_msgs::Int16>("/servo",2,&up2base_control::servocb,this);//接收由上层发送下来的信息
    motor_pub = _nh.advertise<std_msgs::Int16>("/motor_base",2);
    servo_pub = _nh.advertise<std_msgs::Int16>("/servo_base",2);//经过本包与底层直接接触的控制，发送信息给底层

}

up2base_control::~up2base_control()
{
}

//此ros包实现的是对底层电机 舵机的直接控制，因此需要着重考虑安全问题
//想要控制车辆的运动，必须要接收到遥控器信号 由arduino 传入，话题为“base_value"
//由于传入的信息由下位机按照高低电平读入，存在一定波动，因此需要设定一定的波动范围，防止误触
//遥控器的优先级为最高级，读入到的遥控器信号为未触发状态时，可以由上层信号控制，遥控器信号超过阈值，由遥控器接管
void up2base_control::servocb(const std_msgs::Int16::ConstPtr &p)
{
    servo_msg.data = p->data;
    ROS_INFO("up receive servo is %d",p->data);
}

void up2base_control::motorcb(const std_msgs::Int16::ConstPtr &p)
{
    motor_msg.data = p->data;
     ROS_INFO("up receive motor is %d",p->data);
}

void up2base_control::basecb(const router2ros::router2ros::ConstPtr &p)
{
    ROS_INFO("ROUTER receive MOTOR is %d",p->motor_value);
    ROS_INFO("ROUTER receive SERVO is %d",p->servo_value);
    if (abs(p->motor_value-1500) < 30 && abs(p->servo_value-1500) < 30 && error_flag == false)
    {
        //遥控器未介入控制
        ROS_INFO("1111111111111111");
        motor_pub.publish(motor_msg);
        servo_pub.publish(servo_msg);
    }   
    else if(/*abs(p->motor_value) < pwm_limit && */error_flag == true)
    {
        //遥控器作出动作
        ROS_INFO("222222222222222222222222");
        motor_msg.data = p->motor_value;
        servo_msg.data = p->servo_value/16.666667 + 1;
        if(p->motor_value > 1500 + pwm_limit)
            motor_msg.data = 1500 + pwm_limit;
        else if(p->motor_value < 1500 - pwm_limit)
            motor_msg.data = 1500 - pwm_limit;
        motor_pub.publish(motor_msg);
        servo_pub.publish(servo_msg);
    }
    else //出现紧急错误，权限交给遥控器,遥控器接收信号
    {
        error_flag =true;
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fid_calibration_node");
    ros::param::param<int>("/up2base_control/limit_pwm",pwm_limit,150);

    servo_msg.data = 1500;
    motor_msg.data = 1500;

    up2base_control base1;

    ros::spin();
    return 0;
}
