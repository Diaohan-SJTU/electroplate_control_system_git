//job类头文件
// Created by A on 2022/4/27.
//

#ifndef ELECTROPLATE_CONTROL_SYSTEM_GIT_JOB_H
#define ELECTROPLATE_CONTROL_SYSTEM_GIT_JOB_H
class job
{
public:
    int type;//工艺类型
    int id;//1-6
    int pot_process_nb;//工艺数量
    int pot_process[20][3];//槽号、最小加工时间、最大加工时间；
};
#endif
