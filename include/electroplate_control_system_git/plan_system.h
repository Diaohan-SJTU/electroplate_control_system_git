//plan_system头文件
// Created by A on 2022/4/27.
//
#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_PLAN_SYSTEM_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_PLAN_SYSTEM_H
#include "electroplate_control_system_git/job.h"
#include "electroplate_control_system_git/resource_management.h"
#include "electroplate_control_system_git/ros_com.h"
//#include "plan_job.h"


class plan_system
{
public:
    job jobs[6];
    //无氢镀镉前处理
    //槽号（程序0，实际1）、最小加工时间、最大加工时间
    int pot_process_information0[11][4]={{40,18,30,0},{39,3,18,1},{37,3,12,0},{36,3,18,1},{25,6,12,0},{24,3,18,1},{35,3,6,0},{34,3,18,1},{48,30,60,0},{24,3,18,1},{47,3,18,1}};
    int pot_process_information1[20][3]={{7,3,5},{1,3,5},{5,3,5},{9,3,5},{10,3,5},{8,3,5},{12,3,5},{13,3,5},{2,3,5},{4,3,5},{3,3,5},{6,3,5},{11,3,5}};
    int job_repeat_information[6][2];
    int priority_list[6];
    int plan_job_nb;
    plan_job plan_jobs[6];
    resource_management resourceManagement;

    void job_init(int type_list[6]);
    void plan_job_init();
    void plan_v1(ros_com &rc);
    void status_detect_v1(ros_com &rc);
    bool isdone();

};
#endif //V1_PLAN_SYSTEM_H
