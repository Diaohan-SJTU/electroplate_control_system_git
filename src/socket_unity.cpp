//socket_unity源文件
// Created by A on 2022/5/3.
//
#include "socket_unity.h"
#include<iostream>
#include<winsock.h>
#include<sstream>
#include<thread>
using namespace std;

void socket_unity::socket_unity_init()
{
    //初始化套接字库
    WORD w_req = MAKEWORD(2, 2);//版本号
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        cout <<"初始化套接字库失败！"<< endl;
    }
    else {
        cout <<"初始化套接字库成功！"<< endl;
    }
    //检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        cout << "套接字库版本号不符！" << endl;
        WSACleanup();
    }
    else {
        cout << "套接字库版本正确！" << endl;
    }

    //填充服务端信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
    server_addr.sin_port = htons(52000);
    //创建套接字
    s_server = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        cout << "服务器连接失败！" << endl;
        WSACleanup();
    }
    else {
        cout << "服务器连接成功！" << endl;
    }
    //接收一次消息，更新机械臂位置！！！！


    //开启接收消息的新线程
    thread t(&socket_unity::socket_unity_recv,this);
    t.detach();
    cout<<"时刻准备接收消息中......"<<endl;
}

void socket_unity::socket_unity_recv()
{
    while(1)
    {
        recv_len = recv(s_server, recv_buf, 20, 0);
        if (recv_len < 0) {
            cout << "接受失败！" << endl;
            break;
        }
        else {
            recv_parse(recv_buf);
            cout << "服务端信息:" << recv_buf << endl;
        }
    }
}

bool socket_unity::socket_unity_send_01(int robot_id,int tgt_pos,int max_vel)//发送移动指令
{
    stringstream ss;
    ss<<0;
    ss<<1;
    ss<<robot_id;
    int tgt_pos5=tgt_pos/10000;
    int tgt_pos4=(tgt_pos-tgt_pos5*10000)/1000;
    int tgt_pos3=tgt_pos-tgt_pos5*10000-tgt_pos4*1000;
    ss<<tgt_pos5;
    ss<<tgt_pos4;
    ss<<tgt_pos3;
    ss<<max_vel;
    ss>>send_buf;
    send_len = send(s_server, send_buf, 30, 0);
    if (send_len < 0) {
        cout <<send_buf<< "发送失败！" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" 发送成功！" << endl;
        return true;
    }
}
bool socket_unity::socket_unity_send_02(int robot_id,int tgt_type,int tgt_id,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer)//发送抓取指令
{
    stringstream ss;
    ss<<0;
    ss<<2;
    ss<<robot_id;
    ss<<tgt_type;
    if(tgt_id<10){ss<<0;}
    ss<<tgt_id;
    ss<<job_nb;
    ss<<job1_id;
    ss<<job1_buffer;
    ss<<job2_id;
    ss<<job2_buffer;
    ss>>send_buf;
    send_len = send(s_server, send_buf, 30, 0);
    if (send_len < 0) {
        cout <<send_buf<< "发送失败！" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" 发送成功！" << endl;
        return true;
    }
}
bool socket_unity::socket_unity_send_03(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer)//发送抓取指令
{
    stringstream ss;
    ss<<0;
    ss<<3;
    ss<<robot_id;
    ss<<tgt_type;
    if(tgt_id<10){ss<<0;}
    ss<<tgt_id;
    ss<<max_vel;
    ss<<job_nb;
    ss<<job1_id;
    ss<<job1_buffer;
    ss<<job2_id;
    ss<<job2_buffer;
    ss>>send_buf;
    send_len = send(s_server, send_buf, 30, 0);
    if (send_len < 0) {
        cout <<send_buf<< "发送失败！" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" 发送成功！" << endl;
        return true;
    }

//    send_len = send(s_server, send_buf, 20, 0);
//    if (send_len < 0) {
//        cout << "发送失败！" << endl;
//        return false;
//    }
//    else
//    {
//        cout << send_buf<<" 发送成功！" << endl;
//        return true;
//    }
}
bool socket_unity::socket_unity_send_04(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer,int clean_time)//发送移动去水洗指令
{
    stringstream ss;
    ss<<0;
    ss<<4;
    ss<<robot_id;
    ss<<tgt_type;
    if(tgt_id<10){ss<<0;}
    ss<<tgt_id;
    ss<<max_vel;
    ss<<job_nb;
    ss<<job1_id;
    ss<<job1_buffer;
    ss<<job2_id;
    ss<<job2_buffer;
    if(clean_time<100){ss<<0;}
    if(clean_time<10){ss<<0;}
    ss<<clean_time;
    ss>>send_buf;
    send_len = send(s_server, send_buf, 30, 0);
    if (send_len < 0) {
        cout << send_buf<<"发送失败！" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" 发送成功！" << endl;
        return true;
    }

//    send_len = send(s_server, send_buf, 20, 0);
//    if (send_len < 0) {
//        cout << "发送失败！" << endl;
//        return false;
//    }
//    else
//    {
//        cout << send_buf<<" 发送成功！" << endl;
//        return true;
//    }
}

void socket_unity::socket_unity_close()
{
    //关闭套接字
    cout<<"关闭socket连接！";
    closesocket(s_server);
    //释放DLL资源
    WSACleanup();
}

void socket_unity::recv_parse(char recv_inf[])
{
    int order_type=(recv_inf[0]-'0')*10+(recv_inf[1]-'0');
    if(order_type==20)//机械臂位置反馈
    {
        int rbt_id=recv_inf[2]-'0';
        int position=10000*(recv_inf[3]-'0')+1000*(recv_inf[4]-'0')+100*(recv_inf[5]-'0')+10*(recv_inf[6]-'0')+(recv_inf[7]-'0');
        if(rbt_id==0)
        {
            robot0_position=position;
        }
        if(rbt_id==1)
        {
            robot1_position=position;
        }
    }
    if(order_type==21)//移动命令反馈
    {
        int rbt_id=recv_inf[2]-'0';
        robot_done_flag[rbt_id]=1;
    }
    if(order_type==22)//抓取命令反馈
    {
        int rbt_id=recv_inf[2]-'0';
        int target_type=recv_inf[3]-'0';
        int target_id=(recv_inf[4]-'0')*10+recv_inf[5]-'0';
        int job1_id=recv_inf[6]-'0';
        robot_done_flag[rbt_id]=1;
        job1_id_done_flag[job1_id]=1;
        if(target_type==1)
        {
            pot_done_flag[target_id]=1;
        }
        if(target_type==2)
        {
            cart_done_flag[target_id]=1;
        }
    }
    if(order_type==23)//移动放置命令反馈
    {
        int rbt_id=recv_inf[2]-'0';
        int target_type=recv_inf[3]-'0';
        int target_id=(recv_inf[4]-'0')*10+recv_inf[5]-'0';
        int job1_id=recv_inf[6]-'0';
        robot_done_flag[rbt_id]=1;
        job1_id_done_flag[job1_id]=1;
        if(target_type==1)
        {
            pot_done_flag[target_id]=1;
        }
        if(target_type==2)
        {
            cart_done_flag[target_id]=1;
        }
    }
    if(order_type==24)//移动水洗命令反馈
    {
        int rbt_id=recv_inf[2]-'0';
        int target_id=(recv_inf[4]-'0')*10+recv_inf[5]-'0';
        int job1_id=recv_inf[6]-'0';
        robot_done_flag[rbt_id]=1;
        job1_id_done_flag[job1_id]=1;
        pot_done_flag[target_id]=1;
    }
}
