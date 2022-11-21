#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

void order0Callback(const std_msgs::String::ConstPtr& msg){
    int a=3;
};
void order1Callback(const std_msgs::String::ConstPtr& msg){
    int a=3;
};

int main(int argc, char** argv)
{
    // 创建节点
    ros::init(argc, argv, "robot_node");
    ROS_INFO("robot_node started!");
    ros::NodeHandle n;
    //获取工件信息的server
    ros::Subscriber order0_sub=n.subscribe("order0",20,order0Callback);
    ros::Subscriber order1_sub=n.subscribe("order1",20,order1Callback);
    ros::Publisher position0_pub = n.advertise<std_msgs::String>("position0", 20);
    ros::Publisher position1_pub = n.advertise<std_msgs::String>("position1", 20);
    ros::Publisher order0Feedback_pub = n.advertise<std_msgs::String>("order0Feedback", 20);
    ros::Publisher order1Feedback_pub = n.advertise<std_msgs::String>("order1Feedback", 20);
    return 0;
}