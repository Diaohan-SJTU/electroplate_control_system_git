//plan_systemԴ�ļ�
// Created by A on 2022/4/27.
//
#include "electroplate_control_system_git/plan_system.h"
#include "electroplate_control_system_git/ros_com.h"
#include <iostream>
#include <time.h>
#include <unistd.h>
using namespace std;

void plan_system::job_init(int *type_list)
{
    for(int i=0;i<6;i++)
    {
        jobs[i].id=i;
        jobs[i].priority=1;
        jobs[i].type=type_list[i];
        jobs[i].pot_process_nb=11;
        //******������Ϣ��ʼ����ʽv1******
        switch(type_list[i])
        {
            case 0:
                for(int m=0;m<20;m++)
                {
                    for(int n=0;n<4;n++)
                    {
                        jobs[i].pot_process[m][n]=pot_process_information0[m][n];
                    }
                }
                break;
            case 1:
                for(int m=0;m<20;m++)
                {
                    for(int n=0;n<4;n++)
                    {
                        jobs[i].pot_process[m][n]=pot_process_information1[m][n];
                    }
                }
                break;
        }
    }
    for(int i=0;i<6;i++)
    {
        resourceManagement.carts[0].buffer_status[i]=1;
    }
    cout<<"job_init_over"<<endl;

}
//******plan_job_unit()��ʼ����ʽv1******//
void plan_system::plan_job_init()
{
    //����jobs��Ϣ�õ�job_repeat_information,plan_job_init;����ȡ�������ظ���Ϣ
    job_repeat_information[0][0]=0;
    job_repeat_information[0][1]=0;
    plan_job_nb=1;
    int repeat_time=0;
    for(int i=1;i<6;i++)
    {
        job_repeat_information[i][1]=0;
        for(int j=0;j<i;j++)
        {
            if (jobs[i].type==jobs[j].type)
            {
                repeat_time++;
            }
        }
        if(repeat_time%2==0)
        {
            plan_job_nb++;
        }
        job_repeat_information[plan_job_nb-1][repeat_time%2]=i;
        repeat_time=0;
    }

    for (int i=0;i<plan_job_nb;i++)
    {
        priority_list[i]=i;

        int a= job_repeat_information[i][0];//plan_job's job1_id
        int b= job_repeat_information[i][1];//plan_job's job2_id
        plan_jobs[i].id=i;
        plan_jobs[i].type=jobs[a].type;
        plan_jobs[i].priority=jobs[a].priority;
        plan_jobs[i].is_ready=true;
        plan_jobs[i].is_process=false;
        plan_jobs[i].next_operation_nb=0;
        plan_jobs[i].job_number=1;
        plan_jobs[i].co_id[0]=jobs[a].id;
        plan_jobs[i].co_id[1]=0;
        plan_jobs[i].start_time=time(NULL);
        plan_jobs[i].wait_time=0;
        plan_jobs[i].t_ahead=0;
        plan_jobs[i].is_done=false;
        plan_jobs[i].operation_nb=0;

        if(b!=0)//���ڰ�
        {
            plan_jobs[i].job_number=2;
            plan_jobs[i].priority+=1;
            plan_jobs[i].co_id[1]=jobs[b].id;
        }

        //first_operation:cart to robot
        plan_jobs[i].operation_list[0].robot_id=0;//Ĭ��0
        plan_jobs[i].operation_list[0].is_assign= false;
        plan_jobs[i].operation_list[0].target_type=2;//cart
        plan_jobs[i].operation_list[0].target_id=0;
        plan_jobs[i].operation_list[0].action_type=1;
        plan_jobs[i].operation_list[0].min_time4process=0;
        plan_jobs[i].operation_list[0].max_time4process=0;
        plan_jobs[i].operation_nb++;
        int n = 0;

        //operation[1-2m+2]
        for (int m=0;m<jobs[i].pot_process_nb;m++)
        {
            if(jobs[a].pot_process[m][3]==0)//���ղ�
            {
                //robot to pot
                // put
                n = plan_jobs[i].operation_nb;
                plan_jobs[i].operation_list[n].robot_id=0;//robot
                plan_jobs[i].operation_list[n].is_assign= false;
                plan_jobs[i].operation_list[n].target_type=1;//pot
                plan_jobs[i].operation_list[n].target_id=jobs[a].pot_process[m][0];
                plan_jobs[i].operation_list[n].action_type=2;
                plan_jobs[i].operation_list[n].min_time4process=jobs[a].pot_process[m][1];
                plan_jobs[i].operation_list[n].max_time4process=jobs[a].pot_process[m][2];
                plan_jobs[i].operation_nb++;
                //pot to robot
                // grab
                n = plan_jobs[i].operation_nb;
                plan_jobs[i].operation_list[n].robot_id=0;//robot
                plan_jobs[i].operation_list[n].is_assign= false;
                plan_jobs[i].operation_list[n].target_type=1;//pot
                plan_jobs[i].operation_list[n].target_id=jobs[a].pot_process[m][0];
                plan_jobs[i].operation_list[n].action_type=1;
                plan_jobs[i].operation_list[n].min_time4process=0;
                plan_jobs[i].operation_list[n].max_time4process=0;
                plan_jobs[i].operation_nb++;
            }
            else//ˮϴ��
            {
                n = plan_jobs[i].operation_nb;
                plan_jobs[i].operation_list[n].robot_id=0;//robot
                plan_jobs[i].operation_list[n].is_assign= false;
                plan_jobs[i].operation_list[n].target_type=1;//pot
                plan_jobs[i].operation_list[n].target_id=jobs[a].pot_process[m][0];
                plan_jobs[i].operation_list[n].action_type = 3;
                plan_jobs[i].operation_list[n].min_time4process=jobs[a].pot_process[m][1];
                plan_jobs[i].operation_list[n].max_time4process=jobs[a].pot_process[m][2];
                plan_jobs[i].operation_nb++;
            }

        }
        //last_operation:robot to robot
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
        //״̬����
        //��������ʱͨ����ʱʵ��
        status_detect_v1(rc);
        resourceManagement.resource_status_detect_v1(rc);

        //�ж��Ƿ�ȫ���ӹ����
        if(isdone())
        {
            cout<<"�ӹ�������";
            break;
        }
        else
        {
            //�����ȼ������ж�
            for(int i=0;i<plan_job_nb;i++)
            {
                plan_job_id=priority_list[i];
                if(plan_jobs[plan_job_id].is_done){continue;}//�Ѽӹ���ɵĲ��迼��
                if(resourceManagement.is_resource_assign(plan_jobs[plan_job_id]))
                {
                    cout<<"��ʼ�����е�ۣ�plan_job:"<<plan_job_id<<endl;
                    resourceManagement.resource_assign_v1(plan_jobs[plan_job_id],rc);
                }
                else
                {
//                    cout<<"busy!plan_job:"<<i<<endl;
                }
            }
        }
        sleep(1);
        cout<<"1s�󣬿�ʼ״̬��⣡"<<endl;
    }
}

bool plan_system::isdone()//�������
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
    //���ȼ�����
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