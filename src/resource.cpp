//resource源文件
// Created by A on 2022/4/29.
//
#include "electroplate_control_system_git/resource.h"
#include <iostream>
#include<cmath>
#include <time.h>
//#include <unistd.h>
using namespace std;

int robot::move_judge(double target_position1, double target_position2)//����0-100����æΪ1000
{
    double a=0;
    if(is_on_use){return 1000;}
    else
    {
        a=abs(position-target_position1)+abs(position-target_position2);
        a=a/800;
        return a;
    }

}

//移动去抓取
void robot::move2put_v1(double tg_position,int job_number)
{
    is_on_use=true;
    target_position=tg_position;
    //更改缓冲区状态
    int buffer_status_flag=0;
    int buffer_change_nb=0;
    job_nb-=job_number;
    for(int i=0;i<2;i++)
    {
        if(buffer_change_nb>=job_number){break;}//更改完毕
        if(buffer_status[i]==(1-buffer_status_flag))
        {
            buffer_status[i]=buffer_status_flag;
            buffer_change_nb++;
        }
    }
}


void robot::move_v1(double tg_position,bool is_avoid,int pj_id)
{
    is_on_use=true;
    if(!is_avoid){plan_job_id=pj_id;}
    is_position=(is_avoid)? false:true;
    target_position=tg_position;

}

void robot::grab_v1(int job_number)
{
    is_on_use=true;
    plan_job_id=-1;
    is_position=false;
    int buffer_status_flag=1;
    int buffer_change_nb=0;
    job_nb+=job_number;
    for(int i=0;i<2;i++)
    {
        if(buffer_change_nb>=job_number){break;}
        if(buffer_status[i]==(1-buffer_status_flag))
        {
            buffer_status[i]=buffer_status_flag;
            buffer_change_nb++;
        }
    }
}

void robot::move2clean_v1(double tg_position)
{
    is_on_use=true;
    target_position=tg_position;
}



void robot::status_detect_v1(ros_com &rc)
{
    if(id==0){position=rc.robot0_position;}
    else{position=rc.robot1_position;}
    if(is_on_use==true)
    {
        if(rc.robot_done_flag[id]==1)
        {
            cout<<id<<"_robot_get_free!"<<endl;
            is_on_use= false;
            //position=target_position;
            rc.robot_done_flag[id]=0;
        }
    }
}



void pot::work_v1(int action_type,int job_number,int process_time)
{
    is_on_use=true;
    countdown_start= false;
    wait_time=process_time;
    if(action_type==3){return;}//水洗不需要更改缓冲区状态
    int buffer_status_flag=buffer_status;
    if(action_type==2)//放置
    {
        buffer_status_flag=1;
        job_nb=job_number;
    }
    if(action_type==1)
    {
        buffer_status_flag=0;
        job_nb=0;
    }
    //更改缓冲区状态
    if(buffer_status==buffer_status_flag)
    {
        cout<<"error!!!!!!!!!pot_buffer_status_modify_error"<<endl;
    }
    buffer_status=buffer_status_flag;
    //更改挂架数量
}

void pot::status_detect_v1(ros_com &rc)
{
    if(is_on_use==true)
    {
        if(rc.pot_done_flag[id]==1)
        {
//            cout<<id<<"�Ų۱�Ϊ����"<<endl;
//            is_on_use= false;
            cout<<id<<"_pot_countdown_for_"<<wait_time<<"s"<<endl;
            start_time=time(NULL);
            countdown_start= true;
            rc.pot_done_flag[id]=0;
        }
        if(countdown_start)
        {
            time_t now;
            double dif;
            now=time(NULL);
            dif = difftime (now,start_time);
            if(dif>=wait_time)
            {
                cout<<id<<"_pot_get_free"<<endl;
                is_on_use= false;
                countdown_start= false;
            }
        }


    }
}


void cart::work_v1(int action_type,int job_number,const int co_id[])
{
    is_on_use=true;
    int buffer_status_flag=0;
    if(action_type==2){buffer_status_flag=1;}
    for(int i =0;i<job_number;i++)
    {
        if(buffer_status[co_id[i]]==buffer_status_flag)
        {
            cout<<"error!!!!!!!!!pot_buffer_status_modify_error"<<endl;
        }
        buffer_status[co_id[i]]=buffer_status_flag;
    }
}

void cart::status_detect_v1(ros_com &rc)
{
    if(is_on_use==true)
    {
        if(rc.cart_done_flag[id]==1)
        {
            cout<<id<<"_cart_get_free"<<endl;
            is_on_use= false;
            rc.cart_done_flag[id]=0;
        }
    }
}
