//plan_job头文件
// Created by A on 2022/4/27.
//
#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_PLAN_JOB_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_PLAN_JOB_H

#include "electroplate_control_system_git/operation.h"
#include "electroplate_control_system_git/ros_com.h"
#include <time.h>
class plan_job
{
public:
    int type;//corresponding the job.type
    int id;
    int job_number;
    int co_id[2];//the binding job id
    operation operation_list[40];
    int operation_nb;
    int next_operation_nb;
    int priority;

    time_t start_time;
    double wait_time;
    double t_ahead;
    bool countdown_start;
    
    bool is_ready;//是否被机械臂操作
    bool is_process;//是否在加工
    bool is_done;
    
    void status_detect_v1(ros_com &rc);
    void work_v1(bool process_flag);
};


#endif //V1_PLAN_JOB_H
