//socket_unity
// Created by A on 2022/5/3.
//
#include<winsock.h>
#ifndef V1_SOCKET_UNITY_H
#define V1_SOCKET_UNITY_H

class socket_unity
{
public:
    //定义长度变量
    int send_len;
    int recv_len;
    //定义发送缓冲区和接受缓冲区
    char send_buf[20];
    char recv_buf[20];
    //定义服务端套接字，接受请求套接字
    SOCKET s_server;
    //服务端地址客户端地址
    SOCKADDR_IN server_addr;
    //定义命令接收标志
    int robot_done_flag[2];
    int job1_id_done_flag[6];//六个工件id的命令执行状态，planjob通过其co_id[0]匹配更新状态
    int cart_done_flag[2];
    int pot_done_flag[49];

    int robot0_position;
    int robot1_position;

    void socket_unity_init();
    bool socket_unity_send_01(int robot_id,int tgt_pos,int max_vel);
    bool socket_unity_send_02(int robot_id,int tgt_type,int tgt_id,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer);
    bool socket_unity_send_03(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer);
    bool socket_unity_send_04(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer,int clean_time);
    void socket_unity_recv();
    void recv_parse(char recv_inf[]);
    void socket_unity_close();
};
#endif //V1_SOCKET_UNITY_H
