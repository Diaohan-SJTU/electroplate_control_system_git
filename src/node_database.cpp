#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

#include "electroplate_control_system_git/jobs.h"
#include "electroplate_control_system_git/process.h"
#include "electroplate_control_system_git/getJobInfo.h"

using namespace std;

vector<vector<int>> process_0{{13,180,300},{12,30,180},{11,30,180},{6,900,1000},{7,30,180},{8,5,15},{9,30,180},{27,30,180},{26,900,1000}};

bool getJobInfoCallback(electroplate_control_system_git::getJobInfo::Request &req,
                        electroplate_control_system_git::getJobInfo::Response &res)
{
    string cart_id=req.cart_id;
    res.job_nb=4;
    for(int i=0;i<res.job_nb;i++)//every job
    {
        electroplate_control_system_git::jobs cur_job;
        cur_job.type=0;
        cur_job.process_nb=process_0.size();
        for(int j=0;j<cur_job.process_nb;j++){
            electroplate_control_system_git::process cur_process;
            cur_process.pot_id=process_0[j][0];
            cur_process.min_time=process_0[j][1];
            cur_process.max_time=process_0[j][2];
            cur_job.all_process.push_back(cur_process);
        }
        res.jobs.push_back(cur_job);
    }
    return true;
}

int main(int argc, char** argv)
{

    // 创建节点
    
    ros::init(argc, argv, "database_node");
    ROS_INFO("database_node started!");
    ros::NodeHandle n;
    //获取工件信息的server
    ros::ServiceServer get_job_info_server=n.advertiseService("getJobInfo",getJobInfoCallback);
    ros::spin();
//     ros::Rate loop_rate(5);

//     while(ros::ok())//持续更新plc节点状态，以及接受中控的重新上下料指令
//     {
        
//         loop_rate.sleep();
//     }
}