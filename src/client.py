#  coding=utf-8
 
import socket
import time
import rospy
from std_msgs.msg import String


# 位置信号
def callback1(data):
    global sendmsg1
    sendmsg1 = data.data
    rospy.loginfo(data.data)

def callback2(data):
    global sendmsg2
    sendmsg2 = data.data
    rospy.loginfo(data.data)


if __name__=='__main__':
    global sendmsg1
    sendmsg1 = "00500"
    global sendmsg2
    sendmsg2 = "0"#0:safe,1:danger
    HOST = '192.168.50.102'
    PORT = 3000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 定义socket类型，网络通信，TCP
    s.connect((HOST, PORT))  # 要连接的IP与端口
    print("connected!")
    rospy.init_node('unity_sendmsg',anonymous = True)
    rospy.Subscriber("danger",String,callback2)
    rospy.Subscriber("position0",String,callback1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()
        print("sendmsg1:"+'0' + sendmsg1)
        s.sendall(('0' + sendmsg1).encode("utf-8"))  # 把命令发送给对端，按utf-8格式编码
        rate.sleep()
        print("sendmsg2:"+'1' + sendmsg2)
        s.sendall(('1' + sendmsg2).encode("utf-8"))  # 把命令发送给对端，按utf-8格式编码
s.close()  # 关闭连接