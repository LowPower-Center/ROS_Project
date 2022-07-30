#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ArduinoHardware.h>
#include <Servo.h>
 
#define USE_USBCON
#define SERVO_NUM 6
#define init 1500
#define  SERVO_TIME_PERIOD  20    //每隔20ms处理一次（累加）舵机的PWM增量
 
Servo myservo[SERVO_NUM];
const byte servo_pin[SERVO_NUM] = {10, A0, A1, A2, A3,9};
float jointval[SERVO_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0,0.0};
//float buf[SERVO_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0};
bool check = false;//check为true才表示订阅到消息
 
typedef struct {                  //舵机结构体变量声明
    unsigned int aim = 1500;      //舵机目标值 
    float cur = 1500.0;           //舵机当前值
    float inc= 8.48;           //舵机值增量，以20ms为周期
}duoji_struct;

duoji_struct servo_do[SERVO_NUM];           //用结构体变量声明一个舵机变量组
 
ros::NodeHandle nh;
 
void joint1_callback(const std_msgs::Float64& msg){
  jointval[0] = msg.data;
  //buf[0] = jointval[0];
  //myservo[0].writeMicroseconds((int)jointval[0]);
  check = true;
}
 
void joint2_callback(const std_msgs::Float64& msg){
  jointval[1] = msg.data;
  //buf[1] = jointval[1];
  //myservo[1].writeMicroseconds((int)jointval[1]);
}
 
void joint3_callback(const std_msgs::Float64& msg){
  jointval[2] = msg.data;
  //buf[2] = jointval[2];
  //myservo[2].writeMicroseconds((int)jointval[2]);
}
 
void joint4_callback(const std_msgs::Float64& msg){
  jointval[3] = msg.data;
  //buf[3] = jointval[3];
  //myservo[3].writeMicroseconds((int)jointval[3]);
}
 
void joint5_callback(const std_msgs::Float64& msg){
  jointval[4] = msg.data;
  //buf[4] = jointval[4];
  //myservo[4].writeMicroseconds((int)jointval[4]);
}

void joint6_callback(const std_msgs::Float64& msg){
  jointval[5] = msg.data;
  //buf[5] = jointval[5];
  //myservo[5].writeMicroseconds((int)jointval[5]);
}
 
 
ros::Subscriber<std_msgs::Float64> sub1("joint1_value", joint1_callback);
ros::Subscriber<std_msgs::Float64> sub2("joint2_value", joint2_callback);
ros::Subscriber<std_msgs::Float64> sub3("joint3_value", joint3_callback);
ros::Subscriber<std_msgs::Float64> sub4("joint4_value", joint4_callback);
ros::Subscriber<std_msgs::Float64> sub5("joint5_value", joint5_callback);
ros::Subscriber<std_msgs::Float64> sub6("joint6_value", joint6_callback);
void AttachServosAndInit()
{
  for (byte i = 0; i < SERVO_NUM; i++){
    myservo[i].attach(servo_pin[i]);
    myservo[i].writeMicroseconds(init);
  }
}
 
void subscribeToAll()
{
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
}
 
/*
时间处理函数，第一个参数是上一次处理时间点， 第二个参数是处理时间间隔，
到达时间间隔返回0，否则返回1
*/
bool handleTimePeriod( unsigned long *ptr_time, unsigned int time_period) {
  if((millis() - *ptr_time) < time_period) {//millis()返回Arduino开始运行当前程序以来经历的毫秒数
      return 1;  
  } else{
       *ptr_time = millis();
       return 0;
  }
}
 
//解析订阅得到的PWM
void analyJoint()
{
  for (byte i = 0; i < SERVO_NUM; i++)
    servo_do[i].aim = (int)jointval[i]; 
}
 
//舵机PWM增量处理函数，每隔SERVO_TIME_PERIOD毫秒处理一次，这样就实现了舵机的连续控制
void handleServo()
{
  static unsigned long systick_ms_bak = 0;
  if(handleTimePeriod(&systick_ms_bak, SERVO_TIME_PERIOD))return;  //20ms处理一次，不到20ms则返回不处理
  for(byte i = 0; i < SERVO_NUM; i++) {
    if(abs( servo_do[i].aim - servo_do[i].cur) <= abs (servo_do[i].inc) ) {//这里就体现了这个程序的精度，SERVO_TIME_PERIOD越小精度越高
       myservo[i].writeMicroseconds(servo_do[i].aim);              
    } else {
      if (servo_do[i].aim < servo_do[i].cur )
        servo_do[i].cur -= servo_do[i].inc;
      else
        servo_do[i].cur += servo_do[i].inc;
      myservo[i].writeMicroseconds((int)servo_do[i].cur); 
    }    
  }
}
 
void setup(){
  nh.initNode();
  subscribeToAll();
  AttachServosAndInit();
 
}
 
void loop(){
  nh.spinOnce(); 
  analyJoint();
  if (check){//保证起始状态为中间位置
    handleServo();
  }
}
