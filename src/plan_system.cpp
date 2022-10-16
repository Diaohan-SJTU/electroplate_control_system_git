//plan_system源文件
// Created by A on 2022/4/27.
//
#include "electroplate_control_system_git/plan_system.h"
#include "electroplate_control_system_git/ros_com.h"
#include <iostream>
#include <time.h>
#include <unistd.h>
using namespace std;

//使用内置工艺储存，后续需更改为调用ros service
// void plan_system::job_init(int *type_list)
// {
//     for(int i=0;i<6;i++)
//     {
//         jobs[i].id=i;
//         jobs[i].priority=1;
//         jobs[i].type=type_list[i];
//         jobs[i].pot_process_nb=11;
//         //使用内置的两种工艺
//         switch(type_list[i])
//         {
//             case 0:
//                 for(int m=0;m<20;m++)
//                 {
//                     for(int n=0;n<4;n++)
//                     {
//                         jobs[i].pot_process[m][n]=pot_process_information0[m][n];
//                     }
//                 }
//                 break;
//             case 1:
//                 for(int m=0;m<20;m++)
//                 {
//                     for(int n=0;n<4;n++)
//                     {
//                         jobs[i].pot_process[m][n]=pot_process_information1[m][n];
//                     }
//                 }
//                 break;
//         }
//     }
//     for(int i=0;i<6;i++)
//     {
//         resourceManagement.carts[0].buffer_status[i]=1;
//     }
//     cout<<"job_init_over"<<endl;

// }

//根据ros_com的job信息初始化plan_job类，作为调度的基本单元
void plan_system::plan_job_init(ros_com &rc)
{
    int job_nb=rc.job_nb;
    for(int i=0;i<job_nb;i++)
    {
        resourceManagement.carts[0].buffer_status[i]=1;
    }

    //根据jobs信息得到job_repeat_information,plan_job_init;即获取工件的重复信息
    for(int i=0;i<6;i++)
    {
        job_repeat_information[i][0]=0;
        job_repeat_information[i][1]=0;
    }
    plan_job_nb=1;
    bool join_flag;
    for(int i=1;i<6;i++)
    {
        join_flag=false;
        for(int j=0;j<plan_job_nb;j++)
        {
            if(job_repeat_information[j][1]==0&&rc.jobs[job_repeat_information[j][0]].type==rc.jobs[i].type)
            {
                job_repeat_information[j][1]=i;
                join_flag=true;
                break;
            }
        }
        if(!join_flag)
        {
            job_repeat_information[plan_job_nb][0]=i;
            plan_job_nb++;
        }
    } 

    //按照上述重复信息，初始化数个plan_job
    for (int i=0;i<plan_job_nb;i++)
    {
        priority_list[i]=i;

        int a= job_repeat_information[i][0];//plan_job's job1_id
        int b= job_repeat_information[i][1];//plan_job's job2_id
        plan_jobs[i].id=i;
        plan_jobs[i].type=rc.jobs[a].type;
        plan_jobs[i].priority=rc.jobs[a].priority;
        plan_jobs[i].is_ready=true;
        plan_jobs[i].is_process=false;
        plan_jobs[i].next_operation_nb=0;
        plan_jobs[i].job_number=1;
        plan_jobs[i].co_id[0]=rc.jobs[a].id;
        plan_jobs[i].co_id[1]=0;
        plan_jobs[i].start_time=time(NULL);
        plan_jobs[i].wait_time=0;
        plan_jobs[i].t_ahead=0;
        plan_jobs[i].is_done=false;
        plan_jobs[i].operation_nb=0;

        if(b!=0)//存在重复
        {
            plan_jobs[i].job_number=2;
            plan_jobs[i].priority+=1;
            plan_jobs[i].co_id[1]=rc.jobs[b].id;
        }

        //first_operation:cart to robot
        plan_jobs[i].operation_list[0].robot_id=0;//默认0
        plan_jobs[i].operation_list[0].is_assign= false;
        plan_jobs[i].operation_list[0].target_type=2;//cart
        plan_jobs[i].operation_list[0].target_id=0;
        plan_jobs[i].operation_list[0].action_type=1;
        plan_jobs[i].operation_list[0].min_time4process=0;
        plan_jobs[i].operation_list[0].max_time4process=0;
        plan_jobs[i].operation_nb++;
        int n = 0;

        //operation[1-2m+2]
        for (int m=0;m<rc.jobs[i].pot_process_nb;m++)
        {
            if(rc.jobs[a].pot_process[m][3]==0)//工艺槽
            {
                //robot to pot
                // put
                n = plan_jobs[i].operation_nb;
                plan_jobs[i].operation_list[n].robot_id=0;//robot
                plan_jobs[i].operation_list[n].is_assign= false;
                plan_jobs[i].operation_list[n].target_type=1;//pot
                plan_jobs[i].operation_list[n].target_id=rc.jobs[a].pot_process[m][0];
                plan_jobs[i].operation_list[n].action_type=2;
                plan_jobs[i].operation_list[n].min_time4process=rc.jobs[a].pot_process[m][1];
                plan_jobs[i].operation_list[n].max_time4process=rc.jobs[a].pot_process[m][2];
                plan_jobs[i].operation_nb++;
                //pot to robot
                // grab
                n = plan_jobs[i].operation_nb;
                plan_jobs[i].operation_list[n].robot_id=0;//robot
                plan_jobs[i].operation_list[n].is_assign= false;
                plan_jobs[i].operation_list[n].target_type=1;//pot
                plan_jobs[i].operation_list[n].target_id=rc.jobs[a].pot_process[m][0];
                plan_jobs[i].operation_list[n].action_type=1;
                plan_jobs[i].operation_list[n].min_time4process=0;
                plan_jobs[i].operation_list[n].max_time4process=0;
                plan_jobs[i].operation_nb++;
            }
            else//水洗槽
            {
                n = plan_jobs[i].operation_nb;
                plan_jobs[i].operation_list[n].robot_id=0;//robot
                plan_jobs[i].operation_list[n].is_assign= false;
                plan_jobs[i].operation_list[n].target_type=1;//pot
                plan_jobs[i].operation_list[n].target_id=rc.jobs[a].pot_process[m][0];
                plan_jobs[i].operation_list[n].action_type = 3;
                plan_jobs[i].operation_list[n].min_time4process=rc.jobs[a].pot_process[m][1];
                plan_jobs[i].operation_list[n].max_time4process=rc.jobs[a].pot_process[m][2];
                plan_jobs[i].operation_nb++;
            }

        }
        //last_operation:robot to cart
        n = plan_jobs[i].operation_nb;
        plan_jobs[i].operation_list[n].robot_id=0;//robot
        plan_jobs[i].operation_list[n].is_assign= false;
        plan_jobs[i].operation_list[n].target_type=2;//cart
        plan_jobs[i].operation_list[n].target_id=1;
        plan_jobs[i].operation_list[n].action_type= 2;
        plan_jobs[i].operation_list[n].min_time4process=0;
        plan_jobs[i].operation_list[n].max_time4process=0;
        plan_jobs[i].operation_nb++;
    }
    cout<<"plan_job_init_over"<<endl;
}



void plan_system::plan_v1(ros_com &rc)
{
    int plan_job_id=0;
    while(true)
    {
        //根据ros通信情况进行状态更新
        status_detect_v1(rc);
        resourceManagement.resource_status_detect_v1(rc);

        //先判断是否全部加工完成
        if(isdone())
        {
            cout<<"all_work_done!";
            break;
        }
        else
        {
            //按优先级进行判断
            for(int i=0;i<plan_job_nb;i++)
            {
                plan_job_id=priority_list[i];
                if(plan_jobs[plan_job_id].is_done){continue;}//已加工完成的不予考虑
                if(resourceManagement.is_resource_assign(plan_jobs[plan_job_id]))
                {
                    cout<<"start_to_assign_robots_for_plan_job:"<<plan_job_id<<endl;
                    resourceManagement.resource_assign_v1(plan_jobs[plan_job_id],rc);
                }
                else
                {
//                    cout<<"busy!plan_job:"<<i<<endl;
                }
            }
        }
        sleep(1);
        cout<<"wait_1s_to_update_status!"<<endl;
    }
}

bool plan_system::isdone()//判断总体结束
{
    bool isdone_flag=true;
    for(int i=0;i<plan_job_nb;i++)
    {
        if (!plan_jobs[i].is_done)
        {
            isdone_flag=false;
        }
    }
    return isdone_flag;
}

void plan_system::status_detect_v1(ros_com &rc)
{
    //优先级排序
    for (int i = 0; i < plan_job_nb; i++)
    {
        for (int j = 0; j < plan_job_nb -  i - 1; j++)
        {
            if (plan_jobs[priority_list[j]].priority < plan_jobs[priority_list[j+1]].priority)
            {
                int temp;
                temp = priority_list[j + 1];
                priority_list[j + 1] = priority_list[j];
                priority_list[j] = temp;
            }
        }
    }

    for(int i=0;i<plan_job_nb;i++)
    {
        if(plan_jobs[i].is_done)
        {
            continue;
        }
        plan_jobs[i].status_detect_v1(rc);
    }
}