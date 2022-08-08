//resource类头文件
// Created by A on 2022/4/28.
//
#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_RESOURCE_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_RESOURCE_H
#include <time.h>
#include "electroplate_control_system_git/ros_com.h"

class resource
{
public:
    int type;
    int id;
    bool is_on_use;//true:busy,false:availabe
    time_t start_time;
    double wait_time;
    int abnormal_status;
    bool countdown_start;
};

class robot:public resource
{
public:
    int buffer_status[2];
    int job_nb;
    int plan_job_id;
    int position;
    int target_position;
    bool is_position;

    int move_judge(double target_position1,double target_position2);//输入target_position，返回cost值0-100：1000表示繁忙，不考虑
    void status_detect_v1(ros_com &rc);
    int status_detect_unity();
    void move2put_v1(double tg_position,int job_number);
    void move_v1(double tg_position,bool is_avoid,int pj_id);
    void grab_v1(int job_number);
    void move2clean_v1(double tg_position);

};

class pot:public resource
{
public:
    int buffer_status;//0:free,1:filled and busy,2:filled and free
    int job_nb;
    int position[2];
    int lid_state;
    int electric_state;

    int move_judge();
    void time_record();//计时功能：开始电镀后启动计时、对buffer_status进行更改
    void status_detect_v1(ros_com &rc);
    void work_v1(int action_type,int job_number,int proscess_time);
    void lid_control();
    void lid_control_unity();
    void electric_control();
    void electric_control_unity();

};

class cart:public resource
{
public:
    int buffer_status[6];
    int position[2];
    int lock_state;// 0:open, 1:lock

    void status_detect_v1(ros_com &rc);
    void work_v1(int action_type,int job_number,const int co_id[]);
    int status_detect_unity();
    void lock_control();
    void lock_control_unity();

};
#endif //V1_RESOURCE_H
