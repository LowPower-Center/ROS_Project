/*    myrobot_driver.cpp    */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <vector>
 
/*------只是arm group部分，因此只有五个舵机-----*/
//换算舵机PWM值与弧度之间的关系
//舵机运动范围PWM500-2500，对应角度0°-270°，中间状态为1500（设定为偏置）
//关节弧度范围-2.36-2.36，0rad对应舵机中间状态1500
//换算得到每变化1rad，PWM变化423
#define scaler 423
#define offset 1500
 
using namespace std;
 
class FollowJointTrajectoryAction
{
protected:
    sensor_msgs::JointState js;
    std_msgs::Float64 joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos;
    ros::NodeHandle nh;
    std::string action_name_;
    ros::Publisher pub_joint;//给move_group识别的publisher，代替joint_state_publisher，发布joint_states
    ros::Publisher pub_joint1;//给下位机arduino识别的publiser
    ros::Publisher pub_joint2;//同上
    ros::Publisher pub_joint3;//同上
    ros::Publisher pub_joint4;//同上
    ros::Publisher pub_joint5;//同上
    ros::Publisher pub_joint6;//同上
    //与moveit中action client通讯的action server
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
 
    control_msgs::FollowJointTrajectoryActionResult result_;
    control_msgs::FollowJointTrajectoryActionGoal goal_;
 
public:
    FollowJointTrajectoryAction(std::string name) :
        as_(nh, name, false),
        action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB, this));
        as_.start();
        pub_joint = nh.advertise<sensor_msgs::JointState>("/move_group/simple_controller_joint_states", 10);
        pub_joint1 = nh.advertise<std_msgs::Float64>("joint1_value", 100);
        pub_joint2 = nh.advertise<std_msgs::Float64>("joint2_value", 100);
        pub_joint3 = nh.advertise<std_msgs::Float64>("joint3_value", 100);
        pub_joint4 = nh.advertise<std_msgs::Float64>("joint4_value", 100);
        pub_joint5 = nh.advertise<std_msgs::Float64>("joint5_value", 100);
        pub_joint6 = nh.advertise<std_msgs::Float64>("joint6_value", 100);
        // ros::Rate loop_rate(10);

        js.name.resize(6);
        js.position.resize(6);
        //名字要与关节定义的名字一致
        js.name[0] = "joint1";
        js.name[1] = "joint2";
        js.name[2] = "joint3";
        js.name[3] = "joint4";
        js.name[4] = "joint5";
        js.name[5] = "joint2claw_r";
        
        ROS_INFO("-------action start!-------");
    }
    ~FollowJointTrajectoryAction(void)
    {
    }
 
    void goalCB()
    {
        ROS_INFO("-------goal is receive!-------");
        std::vector<trajectory_msgs::JointTrajectoryPoint> points_;
        double points_end[6];
        double Pos_length;
        if (as_.isNewGoalAvailable()){
            js.position.clear();
            points_ = as_.acceptNewGoal()->trajectory.points;
            Pos_length = points_.size();
            for (int i = 0; i < 6; i++){
                //假设v是一个vector对象,v.at(n)和v[n]是一样的
                //但是前者会检查是否越界，后者不会
                points_end[i] = points_.at(Pos_length - 1).positions[i];
                js.position.push_back(points_end[i]);
            }
            js.header.stamp = ros::Time::now();
            //向move_group节点发布规划得到的关节值
            //因为关节名称导致action中数据的排列方式和预期不同,故进行调整
            js.position[2]=points_end[3];
            js.position[3]=points_end[4];
            js.position[4]=points_end[5];
            js.position[5]=points_end[2];
            pub_joint.publish(js);
            //向下位机arduino节点发布规划得到的关节值，直接得到舵机PWM值
            //舵机2需要反向
            joint1_pos.data = -js.position[0] * scaler + offset;
            joint2_pos.data = -js.position[1] * scaler + offset;
            joint3_pos.data = js.position[2] * scaler + offset;
            joint4_pos.data = js.position[3] * scaler + offset;
            joint5_pos.data = -js.position[4] * scaler + offset;
            joint6_pos.data = js.position[5] * scaler + offset;
            pub_joint1.publish(joint1_pos);
            pub_joint2.publish(joint2_pos);
            pub_joint3.publish(joint3_pos);
            pub_joint4.publish(joint4_pos);
            pub_joint5.publish(joint5_pos);
            pub_joint6.publish(joint6_pos);
        }else{
            ROS_INFO("-------goal is not availabel!-------");
        }
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = 0;
        as_.setSucceeded(result);
    }
 
    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }
};
 
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "myrobot_driver");
    FollowJointTrajectoryAction followjointtrajectory("arm_controller/follow_joint_trajectory");//名称要与yaml配置一致
    ros::spin();
    return 0;
}
