//resourceԴ�ļ�
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

//�ƶ�ȥץȡ
//��ʼ��ʱ,���Ļ�����״̬
//ʵ�ʣ������ƶ�ץȡָ��
void robot::move2put_v1(double tg_position,int job_number)
{
    is_on_use=true;
    target_position=tg_position;
    //���Ļ�����״̬
    int buffer_status_flag=0;
    int buffer_change_nb=0;
    job_nb-=job_number;
    for(int i=0;i<2;i++)
    {
        if(buffer_change_nb>=job_number){break;}//�������
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

    //���Ļ�����״̬
    int buffer_status_flag=1;
    int buffer_change_nb=0;
    job_nb+=job_number;
    for(int i=0;i<2;i++)
    {
        if(buffer_change_nb>=job_number){break;}//�������
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



void robot::status_detect_v1(ros_com &rc)//��is_on_useΪtrueʱ������ʱ����м��
{
    if(id==0){position=rc.robot0_position;}
    else{position=rc.robot1_position;}
    if(is_on_use==true)
    {
        if(rc.robot_done_flag[id]==1)
        {
            cout<<id<<"�Ż�е�۱�Ϊ����"<<endl;
            is_on_use= false;
            //position=target_position;
            rc.robot_done_flag[id]=0;
        }
    }
}

//��


void pot::work_v1(int action_type,int job_number,int process_time)
{
    is_on_use=true;
    countdown_start= false;
    wait_time=process_time;
    if(action_type==3){return;}//ˮϴ����Ҫ���Ļ�����״̬
    int buffer_status_flag=buffer_status;
    if(action_type==2)//����ʱ
    {
        buffer_status_flag=1;
        job_nb=job_number;
    }
    if(action_type==1)
    {
        buffer_status_flag=0;
        job_nb=0;
    }
    //���Ļ�����
    if(buffer_status==buffer_status_flag)
    {
        cout<<"error!!!!!!!!!:�ۻ�����״̬�޸Ĵ���"<<endl;
    }
    buffer_status=buffer_status_flag;
    //���ĹҼ�����
}

//��⵽unity�˴������������ָ�������Ҫ��ʼ��ʱ���ȴ���С�ӹ�ʱ��֮��pot.is_on_use��true!
void pot::status_detect_v1(ros_com &rc)
{
    if(is_on_use==true)
    {
        if(rc.pot_done_flag[id]==1)
        {
//            cout<<id<<"�Ų۱�Ϊ����"<<endl;
//            is_on_use= false;
            //��ʼ��ʱ
            cout<<id<<"�Ųۿ�ʼ�ӹ���ʱ"<<wait_time<<"s"<<endl;
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
                cout<<id<<"�Ų۱�Ϊ����"<<endl;
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
    //���Ļ�����
    for(int i =0;i<job_number;i++)
    {
        if(buffer_status[co_id[i]]==buffer_status_flag)
        {
            cout<<"error!!!!!!!!!:���ϳ�������״̬�޸Ĵ���"<<endl;
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
            cout<<id<<"���Ƴ���Ϊ����"<<endl;
            is_on_use= false;
            rc.cart_done_flag[id]=0;
        }
    }
}
