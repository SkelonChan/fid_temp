#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <msg_in_arduino2base/Encoder.h>

ros::NodeHandle nh;
msg_in_arduino2base::Encoder encoder;
ros::Publisher pub_encoder("ros_encoder", &encoder);

int motor_A = 2;// port is 0
int motor_B = 3;//port is 1
//start time - end time 
//FID one and thw motor axis is ten circle
long start_time = millis();
int interval_time = 50;
int per_rount = 13*4*10;
double vel;
double last_vel=0;
double acc;
volatile double odom;


volatile int count = 0;


void count_A(){
  if(digitalRead(motor_A) == HIGH){
    if(digitalRead(motor_B) == HIGH){
      count++;  
    } else {
      count--;  
    }
  } else {
    if(digitalRead(motor_B) == LOW){
      count++;  
    } else {
      count--;  
    }  
  }
}

//4
void count_B(){
  if(digitalRead(motor_B) == HIGH){
    if(digitalRead(motor_A) == LOW){
      count++;
    } else {
      count--;
    }
  } else {
    if(digitalRead(motor_A) == HIGH){
      count++;
    } else {
      count--;
    }
  }
}

void setup() {
  nh.initNode();
  nh.advertise(pub_encoder);
  Serial.begin(57600);  
  pinMode(motor_A,INPUT);
  pinMode(motor_B,INPUT);
  attachInterrupt(0,count_A,CHANGE);
  attachInterrupt(1,count_B,CHANGE);
  odom = 0;//accumulation odom
}


void get_current_vel(){
  long right_now = millis();
  long pass_time = right_now - start_time;
  if (pass_time >= interval_time)
  {
    noInterrupts();
    // float now_odom = (float)count / per_rount * 3.1415926 * 0.23;
    // odom = odom + (float)count;
    // vel = now_odom / pass_time * 1000;
    vel = (double)count / per_rount / pass_time * 1000;//MPM
    vel = vel * 3.1415926 * 0.23;///m/s
    odom = odom + (double)count / per_rount * 3.1415926 * 0.23;//circle * PI *D;
    acc = (vel - last_vel) / pass_time * 1000;
    last_vel = vel;
    count = 0;
    start_time = right_now;
    interrupts();
  }
}


void loop() {
  get_current_vel();
  //Serial.println(acc);
  //odom = odom + vel * 0.05;
  encoder.vel = vel;
  encoder.acc = acc;
  encoder.odom = odom;
  encoder.stamp = nh.now();
  pub_encoder.publish(&encoder);
  //Serial.println(vel);
  nh.spinOnce();
  delay(49);
}
