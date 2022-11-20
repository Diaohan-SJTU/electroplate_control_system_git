//operation类头文件
// Created by A on 2022/4/27.
//

#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_OPERATION_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_OPERATION_H

class operation
{
public:
    int robot_id;//0,1
    bool is_assign;
    int target_type;//1:port;2:cart
    int target_id;
    int action_type;//1:grab 2:put 3:water clean in hand 4:work in hand
    int min_time4process;//最小加工时间,s
    int max_time4process;
    int job_nb;
    int job_id[2];
};

#endif 
