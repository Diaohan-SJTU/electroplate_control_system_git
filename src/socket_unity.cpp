//socket_unityԴ�ļ�
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
    //��ʼ���׽��ֿ�
    WORD w_req = MAKEWORD(2, 2);//�汾��
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        cout <<"��ʼ���׽��ֿ�ʧ�ܣ�"<< endl;
    }
    else {
        cout <<"��ʼ���׽��ֿ�ɹ���"<< endl;
    }
    //���汾��
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        cout << "�׽��ֿ�汾�Ų�����" << endl;
        WSACleanup();
    }
    else {
        cout << "�׽��ֿ�汾��ȷ��" << endl;
    }

    //���������Ϣ
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
    server_addr.sin_port = htons(52000);
    //�����׽���
    s_server = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        cout << "����������ʧ�ܣ�" << endl;
        WSACleanup();
    }
    else {
        cout << "���������ӳɹ���" << endl;
    }
    //����һ����Ϣ�����»�е��λ�ã�������


    //����������Ϣ�����߳�
    thread t(&socket_unity::socket_unity_recv,this);
    t.detach();
    cout<<"ʱ��׼��������Ϣ��......"<<endl;
}

void socket_unity::socket_unity_recv()
{
    while(1)
    {
        recv_len = recv(s_server, recv_buf, 20, 0);
        if (recv_len < 0) {
            cout << "����ʧ�ܣ�" << endl;
            break;
        }
        else {
            recv_parse(recv_buf);
            cout << "�������Ϣ:" << recv_buf << endl;
        }
    }
}

bool socket_unity::socket_unity_send_01(int robot_id,int tgt_pos,int max_vel)//�����ƶ�ָ��
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
        cout <<send_buf<< "����ʧ�ܣ�" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" ���ͳɹ���" << endl;
        return true;
    }
}
bool socket_unity::socket_unity_send_02(int robot_id,int tgt_type,int tgt_id,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer)//����ץȡָ��
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
        cout <<send_buf<< "����ʧ�ܣ�" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" ���ͳɹ���" << endl;
        return true;
    }
}
bool socket_unity::socket_unity_send_03(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer)//����ץȡָ��
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
        cout <<send_buf<< "����ʧ�ܣ�" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" ���ͳɹ���" << endl;
        return true;
    }

//    send_len = send(s_server, send_buf, 20, 0);
//    if (send_len < 0) {
//        cout << "����ʧ�ܣ�" << endl;
//        return false;
//    }
//    else
//    {
//        cout << send_buf<<" ���ͳɹ���" << endl;
//        return true;
//    }
}
bool socket_unity::socket_unity_send_04(int robot_id,int tgt_type,int tgt_id,int max_vel,int job_nb,int job1_id,int job1_buffer,int job2_id,int job2_buffer,int clean_time)//�����ƶ�ȥˮϴָ��
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
        cout << send_buf<<"����ʧ�ܣ�" << endl;
        return false;
    }
    else
    {
        cout << send_buf<<" ���ͳɹ���" << endl;
        return true;
    }

//    send_len = send(s_server, send_buf, 20, 0);
//    if (send_len < 0) {
//        cout << "����ʧ�ܣ�" << endl;
//        return false;
//    }
//    else
//    {
//        cout << send_buf<<" ���ͳɹ���" << endl;
//        return true;
//    }
}

void socket_unity::socket_unity_close()
{
    //�ر��׽���
    cout<<"�ر�socket���ӣ�";
    closesocket(s_server);
    //�ͷ�DLL��Դ
    WSACleanup();
}

void socket_unity::recv_parse(char recv_inf[])
{
    int order_type=(recv_inf[0]-'0')*10+(recv_inf[1]-'0');
    if(order_type==20)//��е��λ�÷���
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
    if(order_type==21)//�ƶ������
    {
        int rbt_id=recv_inf[2]-'0';
        robot_done_flag[rbt_id]=1;
    }
    if(order_type==22)//ץȡ�����
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
    if(order_type==23)//�ƶ����������
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
    if(order_type==24)//�ƶ�ˮϴ�����
    {
        int rbt_id=recv_inf[2]-'0';
        int target_id=(recv_inf[4]-'0')*10+recv_inf[5]-'0';
        int job1_id=recv_inf[6]-'0';
        robot_done_flag[rbt_id]=1;
        job1_id_done_flag[job1_id]=1;
        pot_done_flag[target_id]=1;
    }
}
