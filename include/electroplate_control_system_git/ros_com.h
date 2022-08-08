#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_ROS_COM_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_ROS_COM_H

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

class ros_com
{
private:
    ros::NodeHandle n_;

    ros::Subscriber order_1_sub;
    ros::Subscriber position_1_sub;
    ros::Publisher order_1_pub;
    ros::Subscriber order_0_sub;
    ros::Subscriber position_0_sub;
    ros::Publisher order_0_pub;

public:
    
    //定义命令接收标志
    int robot_done_flag[2];
    int job0_id_done_flag[6];//六个工件id的命令执行状态，planjob通过其co_id[0]匹配更新状态
    int cart_done_flag[2];
    int pot_done_flag[49];

    int robot0_position;
    int robot1_position;
    int robot0_position_interval;
    int robot1_position_interval;
    
    ros_com(ros::NodeHandle* nodehandle);

    void order0FeedbackCallback(const std_msgs::String::ConstPtr& msg);
    void position0FeedbackCallback(const std_msgs::String::ConstPtr& msg);
    void order0Publish(std_msgs::String& msg);
    
    void order1FeedbackCallback(const std_msgs::String::ConstPtr& msg);
    void position1FeedbackCallback(const std_msgs::String::ConstPtr& msg);
    void order1Publish(std_msgs::String& msg);

    void send_order_01(int robot_id,int tgt_pos,int max_vel);
    void send_order_02(int robot_id,int tgt_type,int tgt_id,int job_nb,int job0_id,int job0_buffer,int job1_id,int job1_buffer);
    void send_order_03(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job0_id,int job0_buffer,int job1_id,int job1_buffer);
    void send_order_04(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job0_id,int job0_buffer,int job1_id,int job1_buffer,int clean_time);

    
};

#endif