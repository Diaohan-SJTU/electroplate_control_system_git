# electroplate_control_system_git  中央控制程序
负责人：刁涵

## 功能
---
1. 使用基于规则的动态调度方法，完成复杂的电镀任务分配
2. 从数据库模块获取当前规划任务、向数据库模块发送重要信息
3. 与机械臂模块、电镀槽模块、产线模块等资源通讯，以获取状态并下发命令

## 实现方案
---
### 1. 
### 2. 数据库
- 通过service进行调用
### 3. 产线资源
- 通过topic、service进行通讯

## 接口
### 1. 
### 2. 数据库
- 获取待加工工件信息
- service名称：getJobInfo.srv
- service内容：

    uint8 check_id
    ---
    uint8 job_nb
    electroplate_control_system_git/job[] jobs
    uint8 type
    uint8 process_nb
    electroplate_control_system_git/process[] all_process
        uint8 pot_id
        uint8 min_time
        uint8 max_time
        bool is_water_pot
- server名：

- 发送重要信息
- service名称：sendInfo.srv
- service内容：

    //机械臂行为
    string robotmovements
    ---
    bool check_flag
- server名：

### 3. 产线资源
#### 机械臂模块0
- 发送命令
- topic名称：order0
- msg类型：std_msgs::String

- 接受位置反馈
- topic名称：position0
- msg类型：std_msgs::String

- 接受命令完成反馈
- topic名称：order0Feedback
- msg类型：std_msgs::String

#### 电镀槽模块
- 发送命令
- service名称：sendOrder2Pot.srv
- service内容：

    //槽号
    uint8 pot_id
    //数量
    uint8 job_nbs
    //通电时间
    uint8 work_time
    ---
    bool check_flag
- server名：

#### 现场模块
- 发送重新上下料信息
- topic:reload
- std_msgs::String

- 发送RFID信息
- server：中央控制模块 client：现场模块
- service名称：sendRFIDInfo.srv
- service内容：

    //推车RFID编号
    string cart_id
    ---
    //验证标识
    bool check_flag 


## 文件介绍


## 编译依赖


## 运行逻辑
