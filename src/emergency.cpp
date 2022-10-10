//ros节点：emergency_node
//
#include <iostream>
#include <time.h>
#include <ros/ros.h>

using namespace std;



bool check_time(int sec_now,int sec_last)
{
    int sec=(sec_now-sec_last+60)%60;
    if(sec>0&&sec<5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//检验系统状态与连接的函数
void check_status(ros::NodeHandle &n_){
    //测试部分
    //更改rosparam
    // n_.setParam("robot0/whole_status", true);
    //更改更新时间
    // time_t timeReal;
    // time(&timeReal);
	// timeReal = timeReal + 8*3600;
	// tm* t = gmtime(&timeReal); 
    // int sec=t->tm_sec;
    // n_.setParam("plc_update_time", sec);
    //读取flag
    // bool flag;
    // int a;
    // n_.getParam("robot0/whole_status", flag);
    // n_.getParam("plc_update_time", a);
    // ROS_INFO_STREAM(flag);
    // ROS_INFO_STREAM(a);

    //验证状态安全
    bool flag_robot0,flag_robot1,flag_plc,flag_pot,flag_database=false;
    bool flag_update_robot0,flag_update_robot1,flag_update_plc,flag_update_pot,flag_update_database=false;
    int sec_robot0,sec_robot1,sec_plc,sec_pot,sec_database=0;
    n_.getParam("/params/robot0/whole_status", flag_robot0);
    n_.getParam("/params/robot1/whole_status", flag_robot1);
    n_.getParam("/params/plc_whole_status", flag_plc);
    n_.getParam("/params/pot_whole_status", flag_pot);
    n_.getParam("/params/database_whole_status", flag_database);
    bool flag_status=flag_database&&flag_plc&&flag_pot&&flag_robot0&&flag_robot1;
    //验证更新时间。即连接情况
    time_t timeReal;
    time(&timeReal);
	timeReal = timeReal + 8*3600;
	tm* t = gmtime(&timeReal); 
    int sec=t->tm_sec;
    n_.getParam("/params/robot0/update_time", sec_robot0);
    n_.getParam("/params/robot1/update_time", sec_robot1);
    n_.getParam("/params/plc_update_time", sec_plc);
    n_.getParam("/params/pot_update_time", sec_pot);
    n_.getParam("/params/database_update_time", sec_database);
    flag_update_robot0=check_time(sec,sec_robot0);
    n_.setParam("/params/robot0/connection_status", flag_update_robot0);
    flag_update_robot1=check_time(sec,sec_robot1);
    n_.setParam("/params/robot1/connection_status", flag_update_robot1);
    flag_update_plc=check_time(sec,sec_plc);
    n_.setParam("/params/plc_connection_status", flag_update_plc);
    flag_update_pot=check_time(sec,sec_pot);
    n_.setParam("/params/pot_connection_status", flag_update_pot);
    flag_update_database=check_time(sec,sec_database);
    n_.setParam("/params/database_connection_status", flag_update_database);
    //更新总状态
    bool flag_update=flag_update_database&&flag_update_plc&&flag_update_pot&&flag_update_robot0&&flag_update_robot1;
    if(flag_status&&flag_update){
        n_.setParam("/paramssystem_whole_status", true);
    }
    else{
        n_.setParam("/paramssystem_whole_status", false);
    }
}


int main(int argc, char** argv)
{
    // 创建节点
    ros::init(argc, argv, "emergency_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        check_status(n);
        loop_rate.sleep();
    }
}