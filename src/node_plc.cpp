#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <std_msgs/String.h>
#include "electroplate_control_system_git/loadUnload.h"
#include "electroplate_control_system_git/sendRFIDInfo.h"

#define PORT 8024    //端口号
#define LOG  1       //请求队列中最大连接数量

using namespace std;


bool unload_cart_status=false;
bool load_cart_status=false;
bool unload_status=false;
bool load_status=false;
string unload_cart_id="";
string load_cart_id="";
string last_unload_cart_id="";
string last_load_cart_id="";

bool plc_whole_status=false;
bool plc_connection_status=false;
bool raster1_status=false;
bool raster2_status=false;
bool raster3_status=false;
bool raster4_status=false;
bool emergency_status=false;

//申请下料执行函数
void unload_check()
{
    //未完成下料操作，可以执行下料操作
    if(!unload_status){
        //socket发送可以下料消息
    }
    else{
        //socket发送不可以下料消息
    }
}
//申请上料执行函数
void load_check()
{
    //已经执行下料操作，且上料操作未完成，可以执行上料操作
    if(!unload_status&&load_status){
        //socket发送可以上料消息
    }
    else{
        //socket发送不可以上料消息
    }
}
//完成下料执行函数
void unload_done(string unload_cart_str)
{
    unload_status=true;
    unload_cart_id=unload_cart_str;
}
//完成上料执行函数
void load_done(string load_cart_str,ros::ServiceClient &client)
{
    load_status=true;
    load_cart_id=load_cart_str;
    //呼叫服务sendRFIDInfo
    electroplate_control_system_git::sendRFIDInfo srv;
    srv.request.unload_cart_id=unload_cart_id;
    srv.request.load_cart_id=load_cart_id;
    if(client.call(srv))
    {
        ROS_INFO("Succeed to send RFID information!");
        //socket发送消息，产线变绿
    }
    else{
        ROS_INFO("failed to send RFID information!");
    };
}
//重新上下料函数
//上下料通知函数——与plc进行sockect通讯——通过产线四色灯展示
void reload_callback(const std_msgs::String::ConstPtr& msg)
{
    unload_status=false;
    load_status=false;
    //socket发送重新个上下料消息，产线变黄灯，上下料按钮变红（或其他提示方式
}

void update_plc_status(ros::NodeHandle& n_)
{
    n_.setParam("/params/plc_onnection_status", plc_connection_status);
    n_.setParam("/params/raster1_status", raster1_status);
    n_.setParam("/params/raster2_status", raster2_status);
    n_.setParam("/params/raster3_status", raster3_status);
    n_.setParam("/params/raster4_status", raster4_status);
    n_.setParam("/params/emergency_status", emergency_status);
    //写入更新时间
    plc_whole_status=plc_connection_status&&raster1_status&&raster2_status&&raster3_status&&raster4_status&&emergency_status;
    n_.setParam("/params/plc_whole_status", plc_whole_status);

}

int main(int argc, char** argv)
{

    // 创建节点
    ros::init(argc, argv, "plc_node");
    ROS_INFO("plc_node started!");
    ros::NodeHandle n;
    ros::ServiceClient sendRFIDInfo_client=n.serviceClient<electroplate_control_system_git::sendRFIDInfo>("sendRFIDInfo");
    ros::Subscriber reload_sub=n.subscribe("reload",20,reload_callback);
    ros::Rate loop_rate(5);

    //socket server设置
    int listenfd, connectfd;
	struct sockaddr_in sever;
	struct sockaddr_in client;
	socklen_t addrlen;
   /*
	 *@fuc: 使用socket()函数产生套节字描述符
	 */
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	printf("listenfd:%d",listenfd);
	if(listenfd == -1)
	{
		printf("socket() error\n");
		return -1;
	}
	/*
	 *@fuc: 初始化server套节字地址信息 
	 */
	memset((void *)&sever,0,sizeof(sever));
	sever.sin_family = AF_INET;
	sever.sin_addr.s_addr = inet_addr("127.0.0.1");//ip
	sever.sin_port = htons(PORT);//port
 	/*
	 *@fuc: 用bind()函数，将套接字与指定的协议地址绑定 
	 */
	if(bind(listenfd,(struct sockaddr *)&sever,sizeof(sever)) < 0)
	{
		printf("bind() error\n");
		return -1;
	}
    /*
	 *@fuc: 使用listen()函数，等待客户端的连接 
	 */
    if(listen(listenfd, LOG) < 0)
    {
        printf("listen() error.\n");
        return -1;
    }
    addrlen = sizeof(client);
	char recvbuf[1024];
    //等待客户端连接，从listen队列中取出连接
    connectfd = accept(listenfd,(struct sockaddr *)&client,&addrlen);
    if(connectfd < 0)
    {
        printf("connect() error \n");
        return -1;
    }
    printf("You got a connection from client's IP is %s, port is %d\n",
            inet_ntoa(client.sin_addr), ntohs(client.sin_port));
    
    //额外线程监听socket通讯并执行相应函数：下料信息，下料完成信息，上料消息，上料完成消息，状态更新
    //socket用法
    while(1)
	{	
		recv(connectfd, recvbuf, sizeof(recvbuf), 0);
		printf("client message: %s\n", recvbuf);
		
		send(connectfd, "hello!",8,0);
	}
	    close(connectfd);
	close(listenfd);


    while(ros::ok())//持续更新plc节点状态，以及接受中控的重新上下料指令
    {
        update_plc_status(n);
        loop_rate.sleep();
    }
    
    // 发送推车id测试
    // electroplate_control_system_git::sendRFIDInfo srv;
    // srv.request.cart_id="1234";
    // if(sendRFIDInfo_client.call(srv))
    // {
    //     ROS_INFO("Succeed to send RFID information!");
    // }
    // else{
    //     ROS_INFO("failed to send RFID information!");
    // };
    
    return 0;
}




