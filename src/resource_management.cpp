//resource_managementԴ�ļ�
// Created by A on 2022/4/28.
//

#include "electroplate_control_system_git/resource_management.h"
#include "electroplate_control_system_git/operation.h"
#include "electroplate_control_system_git/ros_com.h"
#include <time.h>
#include <iostream>

#include <algorithm>
using namespace std;

void resource_management::resource_init()//��Դ��ʼ��
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
//        if(i<26)
//        {
//            pots[i].position[0]=0;
//            pots[i].position[1]=1500*(i+1);
//        }
//        else
//        {
//            pots[i].position[0]=1;
//            pots[i].position[1]=1700*(i-25);
//        }
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

bool resource_management::is_resource_assign(plan_job &planJob)
{

    //�жϸ�operation�Ƿ��ڼӹ���
    if(!planJob.is_ready)
    {
        cout<<"plan_job�ӹ��У�"<<endl;
        return false;
    }

    //���û�ˮϴʱ��Ҫ���Ƿ��õĲ��Ƿ�����ҿգ��Ƴ��Ƿ����
    if(planJob.operation_list[planJob.next_operation_nb].action_type!=1)//��ץȡʱ
    {
        if(planJob.operation_list[planJob.next_operation_nb].target_type==1)
        {
            int pot_id=planJob.operation_list[planJob.next_operation_nb].target_id;
            if(pots[pot_id].is_on_use== true)//���ڼӹ��У�����false
            {
                cout<<"Ŀ����ڼӹ��У�"<<endl;
                return false;
            }
            else
            {
                if(pots[pot_id].buffer_status==1)
                {
                    cout<<"���д��ڹҼܣ��޷����ã�"<<endl;
                    return false;
                }
            }
        }
        else
        {
            int cart_id=planJob.operation_list[planJob.next_operation_nb].target_id;
            if(carts[cart_id].is_on_use== true)//���ϳ���ʹ���У�����false
            {
            cout<<"���ϳ���ʹ���У�"<<endl;
                return false;
            }
        }
    }
    //ץȡ����ֱ�ӷ����е��
    return true;
}

bool resource_management::resource_assign_v1(plan_job &planJob, ros_com &rc)//�жϻ�е���Ƿ���ã�ͨ���Ƚϻ�е�۳ɱ��õ����û�е��id��������ײ�жϣ�ok���·�ָ��
{
    int robot0_cost = 1000;
    int robot1_cost = 1000;
    int robot_to_work_id = 0;//Ĭ�ϵ�һ̨
    int target_position=0;
    int target_position_next=0;
    int robot_buffer_free_nb=0;
    int robot_free_nb=0;

    //�ж�is_assign,�������ֱ��ִ����ײ��⼰��������ִ��
    if(!planJob.operation_list[planJob.next_operation_nb].is_assign)//ֻ��ץȡ����Ҫ����
    {
        for(int i=0;i<2;i++)//�жϼ�̨��е�ۿ���
        {
            robot_buffer_free_nb=0;
            if((!robots[i].is_on_use)&&(robots[i].plan_job_id==-1))//��е�ۿ���
            {
                for(int j=0;j<2;j++)//�����е�ۻ�������������
                {
                    if(robots[i].buffer_status[j]==0){robot_buffer_free_nb++;}
                }
                if(robot_buffer_free_nb>=planJob.job_number)//��е�ۻ����������㹻
                {
                    robot_free_nb++;
                    robot_to_work_id=i;
                }
            }
        }//��ÿ��û�е����Ŀ����š�
        cout<<"robot_free_nb:"<<robot_free_nb<<"robot_free_id:"<<robot_to_work_id<<endl;

        if(robot_free_nb==0)//�޿��û�е�ۣ�����ʧ��
        {
            cout<<"�޿��û�е�ۣ�����ʧ�ܣ�"<<endl;
            return false;
        }

        //���Ŀ��λ��
        //���Ŀ��λ��,�Լ�����·��λ�õ���ֵ
        target_position= position_query(planJob.operation_list[planJob.next_operation_nb]);
        int max_pos=target_position;
        int min_pos=target_position;
        int i=1;
        while(true)
        {
            target_position_next= position_query(planJob.operation_list[planJob.next_operation_nb+i]);
            max_pos=max(max_pos,target_position_next);
            min_pos=min(min_pos,target_position_next);
            //���ղ۽�����ˮϴ�ۼ�������
            if(planJob.operation_list[planJob.next_operation_nb+i].action_type==3) {i++;}
            else {break;}
        }

        if(robot_free_nb==1)//һ̨���ã����������Ʒ���Ļ�е�ۣ�Ҳ����ʧ��
        {
            if(max_pos>35000&&robot_to_work_id==0)
            {
                cout<<"���Ʒ���Ļ�е�۲����ã�"<<endl;
                return false;
            }
            if(min_pos<5000&&robot_to_work_id==1)
            {
                cout<<"���Ʒ���Ļ�е�۲����ã�"<<endl;
                return false;
            }
        }

        if(robot_free_nb==2)//��̨���ã��Ƚϳɱ�
        {
            if(max_pos>35000)
            {
                robot_to_work_id=1;
            }
            if(min_pos<5000)
            {
                robot_to_work_id=0;
            }
            if(min_pos>5000&&max_pos<35000)
            {
                robot0_cost=robots[0].move_judge(target_position,target_position_next);
                robot1_cost=robots[1].move_judge(target_position,target_position_next);
                if (abs(robot0_cost-robot1_cost)<0.2) //�ܾ�����ͬ�������ڲ�
                {
                    robot_to_work_id= (abs(robots[0].position-target_position)< abs(robots[1].position-target_position))? 0:1;
                }
                else
                {
                    robot_to_work_id=(robot0_cost<robot1_cost)? 0:1;
                }
            }//�Ƚ�˭��������÷���Ļ����˱��
        }
        planJob.operation_list[planJob.next_operation_nb].robot_id=robot_to_work_id;
        if(is_dead(planJob,robot_to_work_id)==true)
        {
            cout<<"��������֣�"<<endl;
            return false;
        }//�ж��Ƿ�ᷢ������
    }
    else//�ѷ��������£�ֻ���жϷ���Ļ�е���Ƿ����
    {
        robot_to_work_id=planJob.operation_list[planJob.next_operation_nb].robot_id;
        if(robots[robot_to_work_id].is_on_use==true){cout<<"�ѷ���Ļ�е�۲�����!"<<endl;return false;}
    }

    robot_to_work_id=planJob.operation_list[planJob.next_operation_nb].robot_id;
    target_position= position_query(planJob.operation_list[planJob.next_operation_nb]);


    //����collision_judge�������Ի�е�۽�����ײ�ж�
    //�ƶ����λ����ֱ�ӽ���ץȡ������Ҫ������ײ���

    if(collision_judge(robot_to_work_id,target_position))
    {
        //�޸�is_assign
        planJob.operation_list[planJob.next_operation_nb].is_assign=true;
        planJob.operation_list[planJob.next_operation_nb].robot_id =robot_to_work_id;
        //ץȡ��ˮϴ����ʱ���޸���һ�������is_assign,robot_id
        if(planJob.operation_list[planJob.next_operation_nb].action_type!=2)
        {
            planJob.operation_list[planJob.next_operation_nb+1].is_assign=true;
            planJob.operation_list[planJob.next_operation_nb+1].robot_id=robot_to_work_id;
        }
        //����ִ�У������move�����������ط�����
        cout<<"�·�ָ� "<<planJob.co_id[0]<<planJob.co_id[1]<<"�ŹҼ�"<<endl;
        order_make_v1(planJob,target_position,false, rc);
        return true;
    }
    else if(robots[1-robot_to_work_id].is_on_use== false&&robots[1-robot_to_work_id].plan_job_id==-1)
    {
        //�޸�is_assign
        planJob.operation_list[planJob.next_operation_nb].is_assign=true;
        planJob.operation_list[planJob.next_operation_nb].robot_id =robot_to_work_id;
        //ץȡ��ˮϴ����ʱ���޸���һ�������is_assign,robot_id
        if(planJob.operation_list[planJob.next_operation_nb].action_type!=2)
        {
            planJob.operation_list[planJob.next_operation_nb+1].is_assign=true;
            planJob.operation_list[planJob.next_operation_nb+1].robot_id=robot_to_work_id;
        }
        //����ִ�У������move�����������ط�����
        cout<<"�·�ָ� "<<planJob.co_id[0]<<planJob.co_id[1]<<"�ŹҼ�"<<endl;
        order_make_v1(planJob,target_position,true, rc);
        return true;
    }
    else{return false;}


}

bool resource_management::collision_judge(int robot_id, int target_position)//��ײ�ж�
{
    cout<<"��ײ�жϣ�";
    int robot_position=0;
    int _robot_pos=0;
    int _robot_tgt_pos=0;
    robot_position=robots[robot_id].position;
    _robot_pos=robots[1-robot_id].position;
    _robot_tgt_pos=robots[1-robot_id].target_position;
    cout<<"position:"<<robot_position<<"target_position"<<target_position<<endl;
    cout<<"_position:"<<_robot_pos<<"_target_position"<<_robot_tgt_pos<<endl;
    //����һ̨��е�۵�position��target_position������û�е�۵�position��target_positionǰ��3500�����䲻�ظ�ʱ������ײ����
    if(((max(robot_position,target_position)+3500)<min(_robot_pos,_robot_tgt_pos))||((min(robot_position,target_position)-3500)>max(_robot_pos,_robot_tgt_pos)))
    {
        cout<<"������ײ robot_id:"<<robot_id<<endl;
        return true;
    }
    else{
        cout<<"����ײ robot_id:"<<robot_id<<endl;
        return false;

    }
}


//��е�ۻ�������jobid��Ӧδ���
void resource_management::order_make_v1(plan_job &planJob,int target_position,bool is_avoid_need,ros_com &rc)//ָ���·�
{
    int action_type=planJob.operation_list[planJob.next_operation_nb].action_type;
    int robot_id=planJob.operation_list[planJob.next_operation_nb].robot_id;
    int job_number=planJob.job_number;
    int tgt_type=planJob.operation_list[planJob.next_operation_nb].target_type;
    int tgt_id=planJob.operation_list[planJob.next_operation_nb].target_id;
    int pj_id=planJob.id;

    if(action_type==2)//����ʱ
    {
        //����planjob״̬
        planJob.work_v1(true);
        //���Ļ�е��״̬
        robots[robot_id].move2put_v1(target_position,job_number);
        //���1
        cout<<"  robot_id:"<<robot_id<<"move2put!";
        //��target_typeʱ��ʱ�����Ĳۻ��Ƴ���״̬
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
        //�жϻ�е�ۻ�������������
        rc.send_order_03(robot_id,tgt_type,tgt_id,18,job_number,planJob.co_id[0],0,planJob.co_id[1],1);
    }
    if(action_type==1)//ץȡʱ
    {
        //��е��δ����ʱ��ִ��move4grabָ��
        if(!robots[robot_id].is_position)
        {
            cout<<"robot_id:"<<robot_id<<"move!"<<endl;
            robots[robot_id].move_v1(target_position,false,pj_id);
            planJob.priority+=100;//��֤�ƶ������и�planjob�����ȼ��������
            

            rc.send_order_01(robot_id,target_position,18);
        }

        else//��е���ѵ���ʱ��ִ��grabָ��
        {
            if(robots[robot_id].plan_job_id!=pj_id){cout<<"ERR��ִ�еĻ�е�۴���"<<endl;}
            //���ۻ��ڼӹ�����ִ��ץȡ����
            if(tgt_type==1)
            {
                if(pots[tgt_id].is_on_use){cout<<"�ӹ�δ��ɣ�"<<endl;return;}
            }

            //��ok
            robots[robot_id].grab_v1(job_number);
            //���1
            cout<<"robot_id:"<<robot_id<<"grab!";
            planJob.work_v1(false);
            //��target_typeʱ��ʱ�����Ĳۻ��Ƴ���״̬

            if(tgt_type==1)
            {
                //ץȡʱ��ӡ�ȴ�ʱ��
//                time_t now;
//                double dif;
//                now=time(NULL);
//                dif = difftime (now,pots[tgt_id].start_time);
//                cout<<"��ʱ"<<dif-pots[tgt_id].wait_time;
                int process_time=10*planJob.operation_list[planJob.next_operation_nb].min_time4process;
                pots[tgt_id].work_v1(action_type,job_number,process_time);
                //���2
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
    if(action_type==3)//ˮϴʱ
    {
        //����planjob״̬
        planJob.work_v1(false);
        //���Ļ�е��״̬
        robots[robot_id].move2clean_v1(target_position);
        //���1
        cout<<"  robot_id:"<<robot_id<<"move2clean!";
        //���Ĳ۵�״̬
        int time_clean=10*planJob.operation_list[planJob.next_operation_nb].min_time4process;
        pots[tgt_id].work_v1(action_type,job_number,0);//ˮϴ������ȴ��ӹ�ʱ�䣬����е��ִ���궯������Ҳ�������״̬
        cout<<"  pot_id:"<<tgt_id<<endl;
        //������Ϣ
        rc.send_order_04(robot_id,tgt_type,tgt_id,18,job_number,planJob.co_id[0],0,planJob.co_id[1],1,time_clean);
    }

    //�Ƿ�����λ����
    if(is_avoid_need)
    {

        int avoid_position;
        if(target_position>robots[robot_id].position)//��е�������˶�
        {
            if(robots[1-robot_id].position>robots[robot_id].position){avoid_position=target_position+4000;}
            else{avoid_position=robots[robot_id].position-4000;}
        }
        else//��е�������˶�
        {
            if(robots[1-robot_id].position>robots[robot_id].position){avoid_position=robots[robot_id].position+4000;}
            else{avoid_position=target_position-4000;}
        }

        robots[1-robot_id].move_v1(avoid_position,is_avoid_need,pj_id);
        cout<<"robot_id:"<<(1-robot_id)<<"avoid!target_position:"<<avoid_position<<endl;
        rc.send_order_01(1-robot_id,avoid_position,18);
    }
}

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


int resource_management::position_query(operation &oper)
{
    int position_q;
    //���Ŀ��λ��
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

bool resource_management::is_dead(plan_job &planJob, int robot_id) //ץȡʱ�ж��Ƿ���������
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
        //���ղ۽�����ˮϴ�ۼ�������
        if(planJob.operation_list[planJob.next_operation_nb+i].action_type==3) {i++;}
        else {break;}
    }

    if (planJob.operation_list[planJob.next_operation_nb+i].target_type==2){return false;}//���õ�Ϊ�Ƴ�ʱ�����ᷢ����������
    pot_id=planJob.operation_list[planJob.next_operation_nb+i].target_id;
    if(min_pos<5000||max_pos>35000)//���ڻ�е������
    {
        if((pots[pot_id].job_nb+planJob.job_number+robots[robot_id].job_nb)>2)
        {
            return true;
        }
    }
    else//û�л�е������
    {
        if((pots[pot_id].job_nb+planJob.job_number+robots[robot_id].job_nb>2)&&(pots[pot_id].job_nb+robots[1-robot_id].job_nb>2)){return true;}
    }
    return false;
}

