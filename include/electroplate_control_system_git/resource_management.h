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
private:
    //电镀槽数据：方位(0:A;1:B)、距离(电镀槽中心到A1一侧反光板距离)、槽类型：0：冷水洗槽；1：热水洗槽；2：手持工艺槽；3：有杆工艺槽
    int pots_information[50][3]={{0,400,3},{0,1450,3},{0,2500,3},{0,3550,3},{0,4600,3},{0,5650,3},{0,7150,3},{0,8900,0},{0,10400,2},{0,11900,0},
                              {0,13650,3},{0,15400,0},{0,16900,1},{0,18650,3},{0,20583,3},{0,22376,3},{0,24263,3},{0,26063,1},{0,27613,0},{0,29163,2},
                              {0,30713,0},{0,32263,2},{0,33813,0},{0,35365,2},{0,36915,0},{0,38465,2},
                              {1,900,3},{1,2630,1},{1,4360,3},{1,6090,0},{1,7820,3},{1,9800,3},{1,11530,0},{1,13260,3},{1,14990,0},{1,16470,2},
                              {1,17950,0},{1,19430,2},{1,20910,0},{1,22390,1},{1,24120,3},{1,25850,0},{1,27330,2},{1,28810,0},{1,30540,3},
                              {1,32270,0},{1,34000,3},{1,35730,0},{1,37460,3},{1,39190,1}};
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
    int pot_type_query(int pot_id);//通过槽号判断该槽是否有杆；
};
#endif //V1_RESOURCE_MANAGEMENT_H
