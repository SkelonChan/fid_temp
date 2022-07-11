#include <ros/ros.h>
#include <std_msgs/Int16.h>

// 本部分实现的测试部分的定时定量发送转角和电机速度信息，以实现对转角的标定和油门刹车标定表的测定


int main(int argc, char * argv[])
{
    
    ros::init(argc,argv,"test");
    ros::NodeHandle _nh;

    ros::Publisher pub_servo = _nh.advertise<std_msgs::Int16>("/servo",2);
    ros::Publisher pub_motor = _nh.advertise<std_msgs::Int16>("/motor",2);

    double tt = 30;
    int servo = 90;
    int motor = 1200;
    tt = ros::param::param("test/time_value",30);//second
    servo = ros::param::param("test/servo_value",90);
    motor = ros::param::param("test/motor_value",1500);
    // ros::param::get("test/time_value",tt);
    // ros::param::get("test/motor_value",motor);
    // ros::param::get("test/servo_value",servo);

    std_msgs::Int16 motor_value;
    std_msgs::Int16 servo_value;
    motor_value.data = motor;
    servo_value.data = servo;

    double t_last = ros::Time::now().toSec();
    ros::Rate r(50);
    while((ros::Time::now().toSec() - t_last) < tt )
    {
        pub_motor.publish(motor_value);
        pub_servo.publish(servo_value);
        r.sleep();
        ROS_INFO("Time is %f",ros::Time::now().toSec() - t_last);
    }
    motor_value.data = 1500;
    servo_value.data = 90;
    double end_time = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - end_time) < 1)
    {
        pub_motor.publish(motor_value);
        pub_servo.publish(servo_value);
        r.sleep();
    }


    return 0;
}