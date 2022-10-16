#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_ROS_COM_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_ROS_COM_H

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "electroplate_control_system_git/job.h"

#include "electroplate_control_system_git/jobs.h"
#include "electroplate_control_system_git/process.h"

#include "electroplate_control_system_git/getJobInfo.h"
#include "electroplate_control_system_git/sendInfo.h"
#include "electroplate_control_system_git/loadUnload.h"
#include "electroplate_control_system_git/sendPotOrder.h"
#include "electroplate_control_system_git/sendRFIDInfo.h"


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

    ros::ServiceClient get_job_info_client;//向数据库获取工件信息的client
    ros::ServiceClient send_info_client;//向数据库发送重要信息的client
    ros::ServiceServer send_RFID_info_server;//获取RFID信息的server
    ros::Publisher reload_pub;//向plc模块发送重新上下料的信号



public:
    //开始生产标志
    bool work_start_flag;
    //结束生产标志
    bool work_end_flag;
    
    //定义命令接收标志
    int robot_done_flag[2];
    int job0_id_done_flag[6];//六个工件id的命令执行状态，planjob通过其co_id[0]匹配更新状态
    int cart_done_flag[2];
    int pot_done_flag[49];

    int robot0_position;
    int robot1_position;
    int robot0_position_interval;
    int robot1_position_interval;
    //当前工件信息
    int job_nb;
    job jobs[6];
    //开始加工的标志
    bool job_start_flag;

    ros_com(ros::NodeHandle* nodehandle);
    //机械臂相关函数
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

    //数据库相关函数
    void callGetJobInfo();
    void callSendInfo();
    
    //电镀槽相关函数
    

    //PLC相关函数
    bool getRFIDCallback(electroplate_control_system_git::sendRFIDInfo::Request &req,electroplate_control_system_git::sendRFIDInfo::Response &res);
    void reloadPublish();


 
};

#endif