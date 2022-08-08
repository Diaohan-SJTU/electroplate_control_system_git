//plan_job源文件
// Created by A on 2022/5/7.
//
#include "electroplate_control_system_git/plan_job.h"
#include "electroplate_control_system_git/ros_com.h"
#include <time.h>
#include <iostream>

using namespace std;

//plan_job的状态检测函数
//当前时间减去开始时间，超过dif
void plan_job::status_detect_v1(ros_com &rc)
{
    if(next_operation_nb>=operation_nb)
    {
        is_done=true;
        priority=0;
    }
    if(!is_ready)
    {
        int job0_id=co_id[0];
        if(rc.job0_id_done_flag[job0_id]==1)//机械臂执行动作完成，进行加工时间计时
        {
            rc.job0_id_done_flag[job0_id]=0;
            if(wait_time==0)//无加工时间，无需计时
            {
                cout<<co_id[0]<<co_id[1]<<"free!"<<" pirority:"<<priority<<endl;
                is_ready= true;
                next_operation_nb++;
            }
            else
            {
                countdown_start=true;
                start_time=time(NULL);
            }
        }
        if(countdown_start)
        {
            time_t now;
            double dif;
            now=time(NULL);
            dif = difftime (now,start_time)+t_ahead;
            // cout<<"dif:"<<dif<<endl;
            // cout<<"wait_time:"<<wait_time<<endl;
            if(dif>=wait_time)
            {
                cout<<co_id[0]<<co_id[1]<<"free!"<<" pirority:"<<priority<<endl;
                is_ready= true;
                next_operation_nb++;
                countdown_start= false;
            }
        }
    }
    else if(is_process)
    {
        priority+=1;
        cout<<id<<"priority"<<priority<<endl;
    }
}


void plan_job::work_v1(bool process_flag)
{
    countdown_start=false;
    int action_type=operation_list[next_operation_nb].action_type;
    int target_type=operation_list[next_operation_nb].target_type;
    is_ready=false;
    wait_time=10*operation_list[next_operation_nb].min_time4process;
    if(action_type==3)
    {
        wait_time=0;
    }
    //更改plan_job.priority，该步操作是放置操作时，下步抓取调整优先级至12/11；该步操作是抓取操作时，下步抓取调整优先级至1/2
    //放置操作且目标是槽时，提前1min开始为下一步抓取分配资源
    if(action_type==2)
    {
        priority=(job_number==2)? 12:11;
        t_ahead=(target_type==1)? 30:0;
    }
    else{priority=(job_number==2)? 2:1;t_ahead=0;}
    is_process=process_flag;
}