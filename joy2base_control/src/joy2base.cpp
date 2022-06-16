#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>

int pwm_max,pwm_min;
int motor_pwm,servo_num;
std_msgs::Int16 motor_pwm_value;
std_msgs::Int16 servo_num_value;
// int max_pwm_motor,min_pwm_motor;
// void joymsg_cb(const sensor_msgs::Joy::ConstPtr &joymsg);
// int button_motor_pwm_map(const float &pure_button,int &pwm_bound = pwm_max);
// int rocker_motor_pwm_map(const float &pure_button,int &pwm_bound = pwm_max);
// int servo_num_map(const float &servo_value );


class joy2base
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_motor;
    ros::Publisher pub_servo;
    ros::Subscriber sub_joy_msg;
public:
    joy2base();
    ~joy2base();
    void joymsg_cb(const sensor_msgs::Joy::ConstPtr &joymsg);
    // //LT,RT的映射函数，linear （1-- -1）map to (1500, 1700)
    int button_motor_pwm_map(const float &pure_button,int &pwm_bound = pwm_max){
        float temp = abs(pure_button - 1) / 2 ;//计算输入值占输入的比值；
        int pwm = temp * (pwm_bound - 1500);
        return 1500 + pwm;
    }
    int rocker_motor_pwm_map(const float &pure_button,int &pwm_bound = pwm_max){
        int pwm = pure_button * abs(pwm_bound - 1500);
        return 1500 + pwm;
    }
    int servo_num_map(const float &servo_value ){
        int tem = servo_value * 45;
        return tem + 90;
    }
};

joy2base::joy2base(/* args */)
{
    pub_motor = nh_.advertise<std_msgs::Int16>("/motor",3);
    pub_servo = nh_.advertise<std_msgs::Int16>("/servo",3);
    sub_joy_msg = nh_.subscribe<sensor_msgs::Joy>("/joy",1,&joy2base::joymsg_cb,this);
}

joy2base::~joy2base()
{
}




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "joy2base");

    //set param in launch default is in there
    ros::param::param<int>("motor_pwm_maximum",pwm_max,1700);
    ros::param::param<int>("motor_pwm_min",pwm_min,1300);
    
    joy2base joymsg;
    

    ros::spin();
    return 0;
}


void joy2base::joymsg_cb(const sensor_msgs::Joy::ConstPtr &joymsg)
{


    if (joymsg->buttons[4] == 1 || joymsg->buttons[5] == 1)//紧急按钮被触碰
    {
        motor_pwm = 1500;
        servo_num = 90;
    }
    //同时碰到LT RT纯前进后退,碰到了左右摇杆的前进后退
    else if((joymsg->axes[2] != 1 && joymsg->axes[5] != 1 ) || (joymsg->axes[1] != 0 && joymsg->axes[4] != 0))
    {
        motor_pwm = 1500;
        servo_num = 90;
    }
    else
    {
        //motor part
        if (joymsg->axes[2] != 1)//如果LT动作
            motor_pwm = button_motor_pwm_map(joymsg->axes[2],pwm_max);
        else if (joymsg->axes[5] != 1)//如果RT动作
            motor_pwm = button_motor_pwm_map(joymsg->axes[5],pwm_min);
        else if (joymsg->axes[1] > 0)//如果左摇杆向前动作
            motor_pwm = rocker_motor_pwm_map(joymsg->axes[1],pwm_max);
        else if (joymsg->axes[1] < 0)//如果左摇杆向后动作
            motor_pwm = rocker_motor_pwm_map(joymsg->axes[1],pwm_min);
        else if (joymsg->axes[4] > 0)//如果右摇杆向前动作
            motor_pwm = rocker_motor_pwm_map(joymsg->axes[4],pwm_max);
        else if (joymsg->axes[4] < 0)//如果右摇杆向后动作
            motor_pwm = rocker_motor_pwm_map(joymsg->axes[4],pwm_min);
        else 
            motor_pwm = 1500;
        //servo part    
        if (joymsg->axes[0] != 0 && joymsg->axes[3] != 0)//左右摇杆同时左右做作,两个信号源发出舵机信号
        {
            motor_pwm = 1500;
            servo_num = 90;
        }
        else if (joymsg->axes[0] != 0)//左摇杆左右动作
                servo_num = servo_num_map(joymsg->axes[0]);
        else if (joymsg->axes[3] != 0)//右摇杆左右动作
                servo_num = servo_num_map(joymsg->axes[3]);
        else
            servo_num = 90;
    }
    motor_pwm_value.data = motor_pwm;
    servo_num_value.data = servo_num;

    pub_motor.publish(motor_pwm_value);
    pub_servo.publish(servo_num_value);
}

