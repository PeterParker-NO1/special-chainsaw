#!/usr/bin/env python3
# -*- coding: utf-8 -*
 
import  os
import  sys
import  tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)

# global can_grasp
# global can_release

# def grasp_status_cp(msg):
#     global can_release,can_grasp
#     # 物体抓取成功,让机器人回起始点
#     if msg.data=='1':
#         can_release=True
#     if msg.data=='0' or msg.data=='-1':
#         can_grasp=True
# grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=1)

def keyboardLoop():
    rospy.init_node('teleop')
    #初始化监听键盘按钮时间间隔
    rate = rospy.Rate(rospy.get_param('~hz', 50))
 
    #速度变量
    # 慢速
    walk_vel_ = 0.08 #rospy.get_param('walk_vel', 0.08)
    # 快速
    run_vel_ = 0.5 #rospy.get_param('run_vel', 0.5)
    yaw_rate_ = 0.08 #rospy.get_param('yaw_rate', 0.08)
    yaw_rate_run_ = 0.5 #rospy.get_param('yaw_rate_run', 0.5)
    # walk_vel_前后速度
    max_tv = walk_vel_
    # yaw_rate_旋转速度
    max_rv = yaw_rate_
    # 参数初始化
    speed=0
    # global can_release,can_grasp
    # can_grasp=True
    # can_release=False
    
    print ("使用[WASD]控制机器人")
    print ("按[e]抓取水源")
    print ("按[q]放下物体到第一L")
    print ("按[P]退出" )
 
    #读取按键循环
    while not rospy.is_shutdown():
        # linux下读取键盘按键
        fd = sys.stdin.fileno()
        turn =0
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # ch代表获取的键盘按键
        if ch == 'b':
            # if can_release:
            msg=String()
            msg.data='19'
            grasp_pub.publish(msg)
            # can_grasp=False
        if ch == 'p' or ch == 'P':
            # if can_release:
            msg=String()
            msg.data='14'
            grasp_pub.publish(msg)
        if ch == 'n' or ch == 'n':
            # if can_release:
            msg=String()
            msg.data='8'
            grasp_pub.publish(msg)
            #can_release=False
        elif ch == 'j' or ch == 'J':
            # if can_grasp:
            msg=String()
            msg.data='7'
            grasp_pub.publish(msg)
            #can_grasp=False
        elif ch == 'k' or ch == 'K':
            # if can_grasp:
            msg=String()
            msg.data='6'
            grasp_pub.publish(msg)
            #can_grasp=False
        elif ch == 'l' or ch == 'L':
            # if can_grasp:
            msg=String()
            msg.data='4'
            grasp_pub.publish(msg)
            #can_grasp=False
        elif ch == 'u' or ch == 'U':
            # if can_release:
            msg=String()
            msg.data='1'
            grasp_pub.publish(msg)
            #can_release=False
        elif ch == 'i' or ch == 'I':
            # if can_release:
            msg=String()
            msg.data='2'
            grasp_pub.publish(msg)
            #can_release=False
        elif ch == 'o' or ch == 'O':
            # if can_release:
            msg=String()
            msg.data='3'
            grasp_pub.publish(msg)
            #can_release=False

        
        if ch == ' ' or ch == '5':
            speed = 0
            turn = 0
        elif ch == 'w':
            max_tv = walk_vel_
            speed = 1
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 1
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -1
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1
        elif ch == '`':
            exit()
        elif ch == '8':
            max_tv = 2
            speed = 1
            turn = 0
        elif ch == '2':
            max_tv = 2
            speed = -1
            turn = 0
        elif ch == '4':
            max_rv = 2
            speed = 0
            turn = 1
        elif ch == '6':
            max_rv = 2
            speed = 0
            turn = -1
        elif ch == '9':
            max_rv = 0.6
            max_tv = 0.3
            speed = 1
            turn = 1
        elif ch == '7':
            max_rv = 0.6
            max_tv = 0.3
            speed = 1
            turn = -1
        elif ch == '3':
            max_rv = 0.6
            max_tv = 0.3
            speed = -1
            turn = 1
        elif ch == '1':
            max_rv = 0.6
            max_tv = 0.3
            speed = -1
            turn = -1
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #发送消息
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()
		#停止机器人
        #stop_robot()
 
def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
 
if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass

