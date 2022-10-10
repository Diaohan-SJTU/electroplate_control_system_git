#include <iostream>
#include "electroplate_control_system_git/ros_com.h"
#include "electroplate_control_system_git/plan_system.h"
#include <unistd.h>
#include <ros/ros.h>
using namespace std;

int main(int argc, char** argv)
{

//    socket_unity client;
//    client.socket_unity_send_03(0,1,1,18,2,1,0,2,1);

    // int job_type_list[6]={0,0,0,0,0,0};
    // plan_system planSystem;
    // planSystem.resourceManagement.resource_init();
    // planSystem.job_init(job_type_list);//需要读取数据库
    // planSystem.plan_job_init();

    // 创建节点
    ros::init(argc, argv, "main_control_node");
    ros::NodeHandle n;
    
    // ros_com rc(&n);
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    // emergency em;
    // em.check_status(n);

    // sleep(5);
    // rc.send_order_01(0,800,18);
    // while(rc.robot_done_flag[0]==0){}
    // rc.send_order_02(0,1,12,2,0,0,1,1);
    // planSystem.plan_v1(rc);
    // sleep(100);
    
    return 0;


//
//    double position[2]={0,12.34};
//    bool flag;
//    flag = client.socket_unity_send(0,1,position);
//    cout<<flag;
}

