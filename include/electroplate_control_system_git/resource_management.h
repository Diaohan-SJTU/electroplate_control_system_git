//resource_management头文件
// Created by A on 2022/4/28.
//
#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_RESOURCE_MANAGEMENT_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_RESOURCE_MANAGEMENT_H
#include "electroplate_control_system_git/resource.h"
#include "electroplate_control_system_git/operation.h"
#include "electroplate_control_system_git/plan_job.h"
#include "electroplate_control_system_git/ros_com.h"



class resource_management
{
public:
    robot robots[2];
    pot pots[49];
    cart carts[2];

    void resource_init();
    bool is_resource_assign(plan_job &planJob);//判断该operation是否处于加工中 以及当operation目标位置是槽时，判断该槽是否空闲
    bool resource_assign_v1(plan_job &planJob,ros_com &rc);//判断机械臂是否可用，通过比较机械臂成本得到可用机械臂id，进行碰撞判断，ok则下发指令
    bool collision_judge(int robot_id, int target_position);//可以执行，返回true，否则返回false
    void order_make_v1(plan_job &planJob,int target_position,bool is_avoid_need,ros_com &rc);//指令下发
    void resource_status_detect_v1(ros_com &rc);
    int position_query(operation &oper);
    bool is_dead(plan_job &planJob, int robot_id);//判断是否会发生死局

    
};
#endif //V1_RESOURCE_MANAGEMENT_H
