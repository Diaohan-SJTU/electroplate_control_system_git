//resource_management源文件
// Created by A on 2022/4/28.
//

#include "electroplate_control_system_git/resource_management.h"
#include "electroplate_control_system_git/operation.h"
#include "electroplate_control_system_git/ros_com.h"
#include <time.h>
#include <iostream>

#include <algorithm>
using namespace std;

void resource_management::resource_init()//资源初始化
{
    for(int i=0;i<2;i++)//robots_init
    {
        robots[i].type=0;
        robots[i].id=i;
        robots[i].is_on_use= false;
        robots[i].wait_time=0;
        robots[i].abnormal_status=0;
        robots[i].buffer_status[0]=0;
        robots[i].buffer_status[1]=0;
        robots[i].job_nb=0;
        robots[i].position=10000+20000*i;
        robots[i].target_position=10000+20000*i;
        robots[i].start_time=time(NULL);
        robots[i].is_position=false;
        robots[i].plan_job_id=-1;
    }

    int pots_position[49][2]={{0,538},{0,1588},{0,2638},{0,3688},{0,4738},{0,5788},{0,7326},{0,9074},{0,10574},{0,12074},
                              {0,13826},{0,15574},{0,17074},{0,18826},{0,20778},{0,22497},{0,24472},{0,26270},{0,27820},{0,29370},
                              {0,30920},{0,32470},{0,34020},{0,35570},{0,37120},{0,38670},{1,1035},{1,2837},{1,4635},{1,6437},
                              {1,8235},{1,10285},{1,12087},{1,13885},{1,15687},{1,17237},{1,18787},{1,20337},{1,21887},{1,23437},
                              {1,25235},{1,27037},{1,28587},{1,30137},{1,31935},{1,33737},{1,35535},{1,37337},{1,39135}};
    for(int i=0;i<49;i++)//pots_init
    {
        pots[i].type=1;
        pots[i].id=i;
        pots[i].is_on_use= false;
        pots[i].wait_time=0;
        pots[i].abnormal_status=0;
        pots[i].buffer_status=0;
        pots[i].job_nb=0;
        pots[i].position[0]=pots_position[i][0];
        pots[i].position[1]=pots_position[i][1];
        pots[i].lid_state=0;
        pots[i].electric_state=0;
        pots[i].start_time=time(NULL);
    }

    double carts_position[2][2];
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<2;j++)
        {
            carts_position[i][j]=pots_position[i+14][j];
        }
    }
    for(int i=0;i<2;i++)//carts_init
    {
        carts[i].type=2;
        carts[i].id=i;
        carts[i].is_on_use= false;
        carts[i].wait_time=0;
        carts[i].abnormal_status=0;
        for (int j=0;j<6;j++)
        {
            carts[i].buffer_status[j]=0;
        }
        carts[i].position[0]=carts_position[i][0];
        carts[i].position[1]=carts_position[i][1];
        carts[i].lock_state=1;
        carts[i].start_time=time(NULL);
    }
}

//分配机械臂预判断
bool resource_management::is_resource_assign(plan_job &planJob)
{

    //判断当前plan_job是否处于加工中
    if(!planJob.is_ready)
    {
        cout<<"plan_job_on_work!"<<endl;
        return false;
    }

    //放置或水洗操作时需要考虑目标槽是否空闲且有空间，目标推车是否空闲
    if(planJob.operation_list[planJob.next_operation_nb].action_type!=1)//��ץȡʱ
    {
        if(planJob.operation_list[planJob.next_operation_nb].target_type==1)
        {
            int pot_id=planJob.operation_list[planJob.next_operation_nb].target_id;
            if(pots[pot_id].is_on_use== true)//槽在加工，返回false
            {
                cout<<"target_pot_on_work!"<<endl;
                return false;
            }
            else
            {
                if(pots[pot_id].buffer_status==1)
                {
                    cout<<"target_pot_not_empty!"<<endl;
                    return false;
                }
            }
        }
        else
        {
            int cart_id=planJob.operation_list[planJob.next_operation_nb].target_id;
            if(carts[cart_id].is_on_use== true)//推车在加工，返回false
            {
            cout<<"target_cart_on_work!"<<endl;
                return false;
            }
        }
    }
    //抓取时直接分配机械臂
    return true;
}

//分配机械臂
//判断机械臂是否可用，通过比较机械臂成本得到可用机械臂id，进行碰撞判断，ok则下发指令
bool resource_management::resource_assign_v1(plan_job &planJob, ros_com &rc)
{
    int robot0_cost = 1000;
    int robot1_cost = 1000;
    int robot_to_work_id = 0;//Ĭ�ϵ�һ̨
    int target_position=0;
    int target_position_next=0;
    int robot_buffer_free_nb=0;
    int robot_free_nb=0;

    //首先判断plan_job是否已被分配过机械臂，没有则计算可用机械臂数目与id
    if(!planJob.operation_list[planJob.next_operation_nb].is_assign)
    {
        for(int i=0;i<2;i++)//判断几台机械臂可用
        {
            robot_buffer_free_nb=0;
            if((!robots[i].is_on_use)&&(robots[i].plan_job_id==-1))//机械臂可用判断条件
            {
                for(int j=0;j<2;j++)//获得机械臂缓冲区空余位置
                {
                    if(robots[i].buffer_status[j]==0){robot_buffer_free_nb++;}
                }
                if(robot_buffer_free_nb>=planJob.job_number)//机械臂缓冲区空余足够
                {
                    robot_free_nb++;
                    robot_to_work_id=i;
                }
            }
        }
        cout<<"robot_free_nb:"<<robot_free_nb<<"robot_free_id:"<<robot_to_work_id<<endl;

        if(robot_free_nb==0)//无机械臂可用，分配失败
        {
            cout<<"no_available_robots!fail_to_assign!"<<endl;
            return false;
        }

        //获得目标位置以及后续连带路线位置的最值
        target_position= position_query(planJob.operation_list[planJob.next_operation_nb]);
        int max_pos=target_position;
        int min_pos=target_position;
        int i=1;
        while(true)//获取路线位置最值
        {
            target_position_next= position_query(planJob.operation_list[planJob.next_operation_nb+i]);
            max_pos=max(max_pos,target_position_next);
            min_pos=min(min_pos,target_position_next);
            //工艺槽为路线终点，水洗槽继续向后计算
            if(planJob.operation_list[planJob.next_operation_nb+i].action_type==3) {i++;}
            else {break;}
        }

        if(robot_free_nb==1)//一台机械臂可用
        {
            if(max_pos>35000&&robot_to_work_id==0)//两侧限制分配的机械臂不可用！
            {
                cout<<"binded_robot_on_work!"<<endl;
                return false;
            }
            if(min_pos<5000&&robot_to_work_id==1)
            {
                cout<<"binded_robot_on_work!"<<endl;
                return false;
            }
        }

        if(robot_free_nb==2)//两台机械臂可用，比较成本
        {
            if(max_pos>35000)//特殊区域
            {
                robot_to_work_id=1;
            }
            if(min_pos<5000)
            {
                robot_to_work_id=0;
            }
            if(min_pos>5000&&max_pos<35000)//非特殊区域
            {
                robot0_cost=robots[0].move_judge(target_position,target_position_next);
                robot1_cost=robots[1].move_judge(target_position,target_position_next);
                if (abs(robot0_cost-robot1_cost)<0.2) 
                {
                    robot_to_work_id= (abs(robots[0].position-target_position)< abs(robots[1].position-target_position))? 0:1;
                }
                else
                {
                    robot_to_work_id=(robot0_cost<robot1_cost)? 0:1;
                }
            }//更“近”的机械臂获得分配
        }
        planJob.operation_list[planJob.next_operation_nb].robot_id=robot_to_work_id;
        
        if(is_dead(planJob,robot_to_work_id)==true)//判断是否会发生死局
        {
            cout<<"��������֣�"<<endl;
            return false;
        }
    }
    else//已分配过的情况，需要判断分配的机械臂是否可用
    {
        robot_to_work_id=planJob.operation_list[planJob.next_operation_nb].robot_id;
        if(robots[robot_to_work_id].is_on_use==true){cout<<"assigned_robot_on_work!"<<endl;return false;}
    }

    robot_to_work_id=planJob.operation_list[planJob.next_operation_nb].robot_id;
    target_position= position_query(planJob.operation_list[planJob.next_operation_nb]);


    //调用collision_judge函数，对机械臂进行碰撞判断
    //不会碰撞则需要修改状态并下发执行命令
    //移动完就为可以直接进行抓取，不需要进行碰撞检测
    if(collision_judge(robot_to_work_id,target_position))
    {
        //修改is_assign
        planJob.operation_list[planJob.next_operation_nb].is_assign=true;
        planJob.operation_list[planJob.next_operation_nb].robot_id =robot_to_work_id;
        //抓取或水洗操作时，修改下一个工序的is_assign,robot_id
        if(planJob.operation_list[planJob.next_operation_nb].action_type!=2)
        {
            planJob.operation_list[planJob.next_operation_nb+1].is_assign=true;
            planJob.operation_list[planJob.next_operation_nb+1].robot_id=robot_to_work_id;
        }
        //可以执行，调用order_make函数
        cout<<"order_make:No."<<planJob.co_id[0]<<planJob.co_id[1]<<"job!"<<endl;
        order_make_v1(planJob,target_position,false, rc);
        return true;
    }
    else if(robots[1-robot_to_work_id].is_on_use== false&&robots[1-robot_to_work_id].plan_job_id==-1)//阻挡的空闲机械臂执行让位操作
    {
        //修改is_assign
        planJob.operation_list[planJob.next_operation_nb].is_assign=true;
        planJob.operation_list[planJob.next_operation_nb].robot_id =robot_to_work_id;
        //抓取或水洗操作时，修改下一个工序的is_assign,robot_id
        if(planJob.operation_list[planJob.next_operation_nb].action_type!=2)
        {
            planJob.operation_list[planJob.next_operation_nb+1].is_assign=true;
            planJob.operation_list[planJob.next_operation_nb+1].robot_id=robot_to_work_id;
        }
        //可以执行，调用order_make函数
        cout<<"order_make:No."<<planJob.co_id[0]<<planJob.co_id[1]<<"job!"<<endl;
        order_make_v1(planJob,target_position,true, rc);
        return true;
    }
    else{return false;}


}

bool resource_management::collision_judge(int robot_id, int target_position)//碰撞判断
{
    cout<<"collision_judging!";
    int robot_position=0;
    int _robot_pos=0;
    int _robot_tgt_pos=0;
    robot_position=robots[robot_id].position;
    _robot_pos=robots[1-robot_id].position;
    _robot_tgt_pos=robots[1-robot_id].target_position;
    cout<<"position:"<<robot_position<<"target_position"<<target_position<<endl;
    cout<<"_position:"<<_robot_pos<<"_target_position"<<_robot_tgt_pos<<endl;
    //当另一台机械臂的position与target_position区间与当前机械臂的position与target_position前后3.5m的区间不重复时，视为无碰撞风险
    if(((max(robot_position,target_position)+3500)<min(_robot_pos,_robot_tgt_pos))||((min(robot_position,target_position)-3500)>max(_robot_pos,_robot_tgt_pos)))
    {
        cout<<"no_collision:robot_id:"<<robot_id<<endl;
        return true;
    }
    else{
        cout<<"collision:robot_id:"<<robot_id<<endl;
        return false;

    }
}

//指令下法
//机械臂缓冲区与jobid对应未解决
void resource_management::order_make_v1(plan_job &planJob,int target_position,bool is_avoid_need,ros_com &rc)
{
    int action_type=planJob.operation_list[planJob.next_operation_nb].action_type;
    int robot_id=planJob.operation_list[planJob.next_operation_nb].robot_id;
    int job_number=planJob.job_number;
    int tgt_type=planJob.operation_list[planJob.next_operation_nb].target_type;
    int tgt_id=planJob.operation_list[planJob.next_operation_nb].target_id;
    int pj_id=planJob.id;

    if(action_type==2)//放置操作
    {
        //更改plan_job状态
        planJob.work_v1(true);
        //更改机械臂状态
        robots[robot_id].move2put_v1(target_position,job_number);
        cout<<"  robot_id:"<<robot_id<<"move2put!";
        //更改槽或推车状态
        if(tgt_type==1)
        {
            int process_time=10*planJob.operation_list[planJob.next_operation_nb].min_time4process;
            pots[tgt_id].work_v1(action_type,job_number,process_time);
            cout<<"  pot_id:"<<tgt_id<<endl;
        }
        else
        {
            carts[tgt_id].work_v1(action_type,job_number,planJob.co_id);
            cout<<" cart_id:"<<tgt_id<<endl;
        }
        //发送ros消息
        rc.send_order_03(robot_id,tgt_type,tgt_id,18,job_number,planJob.co_id[0],0,planJob.co_id[1],1);
    }
    if(action_type==1)//抓取
    {
        //机械臂未到达时，执行move4grab指令
        if(!robots[robot_id].is_position)
        {
            cout<<"robot_id:"<<robot_id<<"move!"<<endl;
            robots[robot_id].move_v1(target_position,false,pj_id);
            planJob.priority+=100;//保证移动过程中该planjob优先级最高，保证移动完优先执行抓取。
            

            rc.send_order_01(robot_id,target_position,18);
        }

        else//机械臂已到达，执行grab
        {
            if(robots[robot_id].plan_job_id!=pj_id){cout<<"ERR_robot_to_work!"<<endl;}
            //槽还在加工则不执行抓取
            if(tgt_type==1)
            {
                if(pots[tgt_id].is_on_use){cout<<"plan_job_in_pot_on_work！"<<endl;return;}
            }

            //槽ok
            robots[robot_id].grab_v1(job_number);
            cout<<"robot_id:"<<robot_id<<"grab!";
            planJob.work_v1(false);
            //更改槽或推车状态

            if(tgt_type==1)
            {
                //打印等待时间
//                time_t now;
//                double dif;
//                now=time(NULL);
//                dif = difftime (now,pots[tgt_id].start_time);
//                cout<<"overtime_for_"<<dif-pots[tgt_id].wait_time;
                int process_time=10*planJob.operation_list[planJob.next_operation_nb].min_time4process;
                pots[tgt_id].work_v1(action_type,job_number,process_time);
                cout<<"  pot_id:"<<tgt_id<<endl;
            }
            else
            {
                carts[tgt_id].work_v1(action_type,job_number,planJob.co_id);
                //
                cout<<" cart_id:"<<tgt_id<<endl;
            }
            rc.send_order_02(robot_id,tgt_type,tgt_id,job_number,planJob.co_id[0],0,planJob.co_id[1],1);
        }

    }
    if(action_type==3)//水洗操作
    {
        //更改plan_job状态
        planJob.work_v1(false);
        //更改机械臂状态
        robots[robot_id].move2clean_v1(target_position);
        cout<<"  robot_id:"<<robot_id<<"move2clean!";
        //更改槽状态
        int time_clean=10*planJob.operation_list[planJob.next_operation_nb].min_time4process;
        pots[tgt_id].work_v1(action_type,job_number,0);//ˮϴ������ȴ��ӹ�ʱ�䣬����е��ִ���궯������Ҳ�������״̬
        cout<<"  pot_id:"<<tgt_id<<endl;
        //发送消息
        rc.send_order_04(robot_id,tgt_type,tgt_id,18,job_number,planJob.co_id[0],0,planJob.co_id[1],1,time_clean);
    }

    //让位操作
    if(is_avoid_need)
    {

        int avoid_position;
        if(target_position>robots[robot_id].position)//往左运动
        {
            if(robots[1-robot_id].position>robots[robot_id].position){avoid_position=target_position+4000;}
            else{avoid_position=robots[robot_id].position-4000;}
        }
        else//wangyouyundong
        {
            if(robots[1-robot_id].position>robots[robot_id].position){avoid_position=robots[robot_id].position+4000;}
            else{avoid_position=target_position-4000;}
        }

        robots[1-robot_id].move_v1(avoid_position,is_avoid_need,pj_id);
        cout<<"robot_id:"<<(1-robot_id)<<"avoid!target_position:"<<avoid_position<<endl;
        rc.send_order_01(1-robot_id,avoid_position,18);
    }
}
//资源状态更新
void resource_management::resource_status_detect_v1(ros_com &rc)
{
    for(int i=0;i<2;i++)
    {
        robots[i].status_detect_v1(rc);
    }
    for(int i=0;i<49;i++)
    {
        pots[i].status_detect_v1(rc);
    }
    for(int i=0;i<2;i++)
    {
        carts[i].status_detect_v1(rc);
    }
}

//目标位置查询
int resource_management::position_query(operation &oper)
{
    int position_q;
    if(oper.target_type==1)
    {
        position_q = pots[oper.target_id].position[1];
    }
    else
    {
        position_q = carts[oper.target_id].position[1];
    }
    return  position_q;
}

//判断死局
bool resource_management::is_dead(plan_job &planJob, int robot_id)
{
    int target_position;
    int target_position_next;
    int pot_id;
    target_position= position_query(planJob.operation_list[planJob.next_operation_nb]);

    int i=1;
    int max_pos=target_position;
    int min_pos=target_position;
    while(true)
    {

        target_position_next= position_query(planJob.operation_list[planJob.next_operation_nb+i]);
        max_pos=max(max_pos,target_position_next);
        min_pos=min(min_pos,target_position_next);
        //工艺槽结束，水洗槽继续计算
        if(planJob.operation_list[planJob.next_operation_nb+i].action_type==3) {i++;}
        else {break;}
    }

    if (planJob.operation_list[planJob.next_operation_nb+i].target_type==2){return false;}//放置在推车时不会发生死局现象
    pot_id=planJob.operation_list[planJob.next_operation_nb+i].target_id;
    if(min_pos<5000||max_pos>35000)//存在机械臂限制
    {
        if((pots[pot_id].job_nb+planJob.job_number+robots[robot_id].job_nb)>2)
        {
            return true;
        }
    }
    else//不存在机械臂限制
    {
        if((pots[pot_id].job_nb+planJob.job_number+robots[robot_id].job_nb>2)&&(pots[pot_id].job_nb+robots[1-robot_id].job_nb>2)){return true;}
    }
    return false;
}

