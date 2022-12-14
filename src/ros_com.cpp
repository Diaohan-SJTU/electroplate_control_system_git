#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include "electroplate_control_system_git/ros_com.h"


using namespace std;

ros_com::ros_com(ros::NodeHandle* nodehandle):n_(*nodehandle)
{
    ROS_INFO("in class constructor of ros_com!");

    work_start_flag=false;
    work_end_flag=false;
    
    robot_done_flag[2]={0};
    job0_id_done_flag[6]={0};//六个工件id的命令执行状态，planjob通过其co_id[0]匹配更新状态
    cart_done_flag[2]={0};
    pot_done_flag[49]={0};

    robot0_position=10000;
    robot1_position=30000;
    robot0_position_interval=20000;
    robot1_position_interval=20000;
    
    job_nb=0;
    job_start_flag=false;

    
    order_0_sub = n_.subscribe("order0Feedback",20,&ros_com::order0FeedbackCallback,this);
    position_0_sub = n_.subscribe("position0",20,&ros_com::position0FeedbackCallback,this);
    order_0_pub = n_.advertise<std_msgs::String>("order0", 20);
    
    order_1_sub = n_.subscribe("order1Feedback",20,&ros_com::order1FeedbackCallback,this);
    position_1_sub = n_.subscribe("position1",20,&ros_com::position1FeedbackCallback,this);
    order_1_pub = n_.advertise<std_msgs::String>("order1", 20);

    get_job_info_client=n_.serviceClient<electroplate_control_system_git::getJobInfo>("getJonInfo");
    send_info_client=n_.serviceClient<electroplate_control_system_git::sendInfo>("sendInfo");
    send_RFID_info_server=n_.advertiseService("sendRFIDInfo",&ros_com::getRFIDCallback,this);
    reload_pub=n_.advertise<std_msgs::String>("reload",5);
    work_start_end_server = n_.advertiseService("workStartEnd",&ros_com::workStartEndCallback,this);
}


//轨道车工控机发送命令完成反馈信号
//移动完成【21】【21】
//抓取完成【22+目标点类型+目标点id+工件0id】【22111】目标点类型：1槽2车
//移动去放置完成【23+目标点类型+目标点id+工件0id】【22111】
//移动去水洗完成【24+目标点类型+目标点id+工件0id】【22111】
void ros_com::order0FeedbackCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("[order0Feedback]:I heard : [%s]" , msg->data.c_str()); // String 转为 str字符串
    // ROS_INFO_STREAM("thread="<<boost::this_thread::get_id());
    //解析数据
    int flag = stoi(msg->data.substr(0,2));//获取校验位
    if(flag==21)
    {
        robot_done_flag[0]=1;
    }
    if(flag==22)
    {
        int target_type=stoi(msg->data.substr(2,1));
        int target_id=stoi(msg->data.substr(3,1));
        int job0_id=stoi(msg->data.substr(4,1));
        robot_done_flag[0]=1;
        job0_id_done_flag[job0_id]=1;
        if(target_type==1)
        {
            pot_done_flag[target_id]=1;
        }
        if(target_type==2)
        {
            cart_done_flag[target_id]=1;
        }
    }
    if(flag==23)
    {
        int target_type=stoi(msg->data.substr(2,1));
        int target_id=stoi(msg->data.substr(3,1));
        int job0_id=stoi(msg->data.substr(4,1));
        robot_done_flag[0]=1;
        job0_id_done_flag[job0_id]=1;
        if(target_type==1)
        {
            pot_done_flag[target_id]=1;
        }
        if(target_type==2)
        {
            cart_done_flag[target_id]=1;
        }
    }
    if(flag==24)
    {
        int target_id=stoi(msg->data.substr(3,1));
        int job0_id=stoi(msg->data.substr(4,1));
        robot_done_flag[0]=1;
        job0_id_done_flag[job0_id]=1;
        pot_done_flag[target_id]=1;
    }
}

//轨道车工控机发送位置信号【20+绝对位置+相对位置】【201111122222】
void ros_com::position0FeedbackCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard : [%s]" , msg->data.c_str()); // String 转为 str字符串
    // ROS_INFO_STREAM("thread="<<boost::this_thread::get_id());
    int flag = stoi(msg->data.substr(0,2));//获取校验位
    if(flag==20)
    {
        robot0_position=stoi(msg->data.substr(2,5));
        robot0_position_interval=stoi(msg->data.substr(7,5));
    }
}

//向轨道车工控机发送命令信号

//抓取【02+目标点类型+目标点id+最大速度+工件数量+最大速度+工件数量】【012222218】

void ros_com::order0Publish(std_msgs::String& msg)
{
    order_0_pub.publish(msg);
}

void ros_com::order1FeedbackCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("[order1Feedback]:I heard : [%s]" , msg->data.c_str()); // String 转为 str字符串
    // ROS_INFO_STREAM("thread="<<boost::this_thread::get_id());

    //解析数据
    int flag = stoi(msg->data.substr(0,2));//获取校验位
    if(flag==21)
    {
        robot_done_flag[1]=1;
    }
    if(flag==22)
    {
        int target_type=stoi(msg->data.substr(2,1));
        int target_id=stoi(msg->data.substr(3,1));
        int job0_id=stoi(msg->data.substr(4,1));
        robot_done_flag[1]=1;
        job0_id_done_flag[job0_id]=1;
        if(target_type==1)
        {
            pot_done_flag[target_id]=1;
        }
        if(target_type==2)
        {
            cart_done_flag[target_id]=1;
        }
    }
    if(flag==23)
    {
        int target_type=stoi(msg->data.substr(2,1));
        int target_id=stoi(msg->data.substr(3,1));
        int job0_id=stoi(msg->data.substr(4,1));
        robot_done_flag[1]=1;
        job0_id_done_flag[job0_id]=1;
        if(target_type==1)
        {
            pot_done_flag[target_id]=1;
        }
        if(target_type==2)
        {
            cart_done_flag[target_id]=1;
        }
    }
    if(flag==24)
    {
        int target_id=stoi(msg->data.substr(3,1));
        int job0_id=stoi(msg->data.substr(4,1));
        robot_done_flag[1]=1;
        job0_id_done_flag[job0_id]=1;
        pot_done_flag[target_id]=1;
    }
}

void ros_com::position1FeedbackCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard : [%s]" , msg->data.c_str()); // String 转为 str字符串
    // ROS_INFO_STREAM("thread="<<boost::this_thread::get_id());
    int flag = stoi(msg->data.substr(0,2));//获取校验位
    if(flag==21)
    {
        robot1_position=stoi(msg->data.substr(2,5));
        robot1_position_interval=stoi(msg->data.substr(7,5));
    }
}

void ros_com::order1Publish(std_msgs::String& msg)
{
    order_1_pub.publish(msg);
}

//发送移动命令
//移动【01+机械臂id+距离+最大速度】【012222218】
void ros_com::send_order_01(int robot_id,int tgt_pos,int max_vel)
{
    std_msgs::String msg_ord;
    stringstream ss_ord;
    ss_ord<<0;
    ss_ord<<1;
    int tgt_pos5=tgt_pos/10000;
    int tgt_pos4=(tgt_pos-tgt_pos5*10000)/1000;
    int tgt_pos3=tgt_pos-tgt_pos5*10000-tgt_pos4*1000;
    ss_ord<<tgt_pos5;
    ss_ord<<tgt_pos4;
    ss_ord<<tgt_pos3;
    ss_ord<<max_vel;

    msg_ord.data = ss_ord.str();
    if(robot_id==0)
    {
        order0Publish(msg_ord);
    }
    else
    {
        order1Publish(msg_ord);
    }
}

//发送抓取命令
//抓取【02+机械臂id+目标点类型+目标点id+工件数量+工件0id+工件0缓冲区id+工件1id+工件1缓冲区id】【0213320011】
void ros_com::send_order_02(int robot_id,int tgt_type,int tgt_id,int job_nb,int job0_id,int job0_buffer,int job1_id,int job1_buffer)
{
    std_msgs::String msg_ord;
    stringstream ss_ord;
    ss_ord<<0;
    ss_ord<<2;
    ss_ord<<tgt_type;
    if(tgt_id<10){ss_ord<<0;}
    ss_ord<<tgt_id;
    ss_ord<<job_nb;
    ss_ord<<job0_id;
    ss_ord<<job0_buffer;
    ss_ord<<job1_id;
    ss_ord<<job1_buffer;

    msg_ord.data = ss_ord.str();
    if(robot_id==0)
    {
        order0Publish(msg_ord);
    }
    else
    {
        order1Publish(msg_ord);
    }
}

//发送移动去抓取命令
//移动去抓取【03+机械臂id+目标点类型+目标点id+最大速度+工件数量+工件0id+工件0缓冲区id+工件1id+工件1缓冲区id】【031331820011】
void ros_com::send_order_03(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job0_id,int job0_buffer,int job1_id,int job1_buffer)
{
    std_msgs::String msg_ord;
    stringstream ss_ord;
    ss_ord<<0;
    ss_ord<<3;
    ss_ord<<tgt_type;
    if(tgt_id<10){ss_ord<<0;}
    ss_ord<<tgt_id;
    ss_ord<<job_nb;
    if(max_vel<10){ss_ord<<0;}
    ss_ord<<max_vel;
    ss_ord<<job0_id;
    ss_ord<<job0_buffer;
    ss_ord<<job1_id;
    ss_ord<<job1_buffer;

    msg_ord.data = ss_ord.str();
    if(robot_id==0)
    {
        order0Publish(msg_ord);
    }
    else
    {
        order1Publish(msg_ord);
    }
}

//发送移动去水洗命令
//移动去抓取【04+机械臂id+目标点类型+目标点id+最大速度+工件数量+工件0id+工件0缓冲区id+工件1id+工件1缓冲区id+水洗时间】【021331820011030】
void ros_com::send_order_04(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job0_id,int job0_buffer,int job1_id,int job1_buffer,int clean_time)
{
    std_msgs::String msg_ord;
    stringstream ss_ord;
    ss_ord<<0;
    ss_ord<<4;
    ss_ord<<tgt_type;
    if(tgt_id<10){ss_ord<<0;}
    ss_ord<<tgt_id;
    ss_ord<<job_nb;
    if(max_vel<10){ss_ord<<0;}
    ss_ord<<max_vel;
    ss_ord<<job0_id;
    ss_ord<<job0_buffer;
    ss_ord<<job1_id;
    ss_ord<<job1_buffer;
    if(clean_time<100){ss_ord<<0;}
    if(clean_time<10){ss_ord<<0;}
    ss_ord<<clean_time;

    msg_ord.data = ss_ord.str();
    if(robot_id==0)
    {
        order0Publish(msg_ord);
    }
    else
    {
        order1Publish(msg_ord);
    }
}


bool ros_com::getRFIDCallback(electroplate_control_system_git::sendRFIDInfo::Request &req,electroplate_control_system_git::sendRFIDInfo::Response &res)
{
    electroplate_control_system_git::getJobInfo srv;
    srv.request.cart_id=req.load_cart_id;
    if(get_job_info_client.call(srv))
    {
        job_nb=srv.response.job_nb;
        for(int i=0;i<job_nb;i++)
        {
            jobs[i].id=i;
            jobs[i].type=srv.response.jobs[i].type;
            jobs[i].pot_process_nb=srv.response.jobs[i].process_nb;
            for(int j=0;j<jobs[i].pot_process_nb;j++)
            {
                //槽号
                jobs[i].pot_process[j][0]=srv.response.jobs[i].all_process[j].pot_id;
                //最小加工时间
                jobs[i].pot_process[j][1]=srv.response.jobs[i].all_process[j].min_time;
                //最大加工时间
                jobs[i].pot_process[j][2]=srv.response.jobs[i].all_process[j].max_time;
                //是否为水洗槽
                jobs[i].pot_process[j][3]=srv.response.jobs[i].all_process[j].is_water_pot;
            }
            
        }
        work_start_flag=true;
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
    return true;
}

//向plc节点发送重新上下料消息
void ros_com::reloadPublish()
{
    std_msgs::String msg;
    stringstream ss_ord;
    ss_ord<<0;
    reload_pub.publish(msg);
}
//呼叫服务，向数据库获取工件信息
// void ros_com::callGetJobInfo()
// {
//     electroplate_control_system_git::getJobInfo srv;
//     srv.request.check_id=1;
//     if(get_job_info_client.call(srv))
//     {

//     }
//     else
//     {
//         ROS_ERROR("Failed to call service");
//     }
// }

//处理gui节点的开启生产，结束生产服务
bool ros_com::workStartEndCallback(electroplate_control_system_git::workStartEnd::Request &req,electroplate_control_system_git::workStartEnd::Response &res)
{
    if(req.work_start_flag)
    {
        if(work_start_flag==true)
        {
            res.work_start_check_flag=false;
        }
        else{
            res.work_start_check_flag=true;
            work_start_flag=true;
        }
    }
    if(req.work_end_flag)
    {
        if(job_start_flag==true)
        {
            res.work_end_check_flag=false;
        }
        else{
            res.work_end_check_flag=true;
            work_end_flag=true;
            work_start_flag=false;
        }
    }
    return true;
}