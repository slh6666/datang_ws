#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import rospy
#导入最主要的Python for ROS库
import RPi.GPIO  as GPIO
from geometry_msgs.msg import Twist
from math import pi
#导入geometry_msgs包中的Twist消息类型


class Control():
    def __init__(self,linear_speed,angular_speed):
        # 节点名称
        rospy.init_node('control', anonymous=False)
        # 当终端按下Ctrl＋C之后可以终止节点      
        rospy.on_shutdown(self.shutdown) 
        # 定义在/cmd_vel Topic中发布Twist消息，控制机器人速度
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        rate = 100
        # 设置更新频率为100HZ
        r = rospy.Rate(rate) 
        # 线速度
        self.linear_speed = linear_speed 
        # 角速度 
        self.angular_speed = angular_speed 

    def left(self):
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed
        while GPIO.input(13):
            self.cmd_vel.publish(move_cmd)

    def right(self):
        move_cmd = Twist()
        move_cmd.angular.z = -self.angular_speed
        while GPIO.input(6):
            self.cmd_vel.publish(move_cmd)

    def forward(self): 
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        while GPIO.input(21):
            self.cmd_vel.publish(move_cmd)

    def back(self):    
        move_cmd = Twist()
        move_cmd.linear.x = -self.linear_speed
        while GPIO.input(20):
            self.cmd_vel.publish(move_cmd)

    def stop(self):
        move_cmd = Twist()
        move_cmd.angular.z =0
        move_cmd.linear.x =0
        #while(1):
        #while GPIO.input(26):
        self.cmd_vel.publish(move_cmd)


        # 定义 shutdown(self)可以手动停止机器人
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        #rospy.loginfo("Stopping the robot...")
        #self.cmd_vel.publish(Twist())
        #rospy.sleep(1)
        #print("1")
        rospy.signal_shutdown(control)
        

if __name__ == '__main__':
    control=Control(0.2,30.0)
    GPIO.setmode(GPIO.BCM)
    chan_list  =  [21, 20, 13, 19, 26]
    for i in range(len(chan_list)):
        GPIO.setup(chan_list[i],GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        #print(GPIO.input(chan_list[i]))
    while 1:
        if GPIO.input(21):
            print("forward")
            try:
                control.forward()
            except:
                rospy.loginfo("control terminated.")
        if GPIO.input(20):
            print("back")
            try:
                control.back()
            except:
                rospy.loginfo("control terminated.")
        if GPIO.input(13):
            print("left")
            try:
                control.left()
            except:
                rospy.loginfo("control terminated.")    
        if GPIO.input(19):
            print("right")
            try:
                control.right()
            except:
                rospy.loginfo("control terminated.")
        if GPIO.input(26):
            print("stop")
            try:
                control.stop()
            except:
                rospy.loginfo("control terminated.")           
        else:
            control.stop()
            print('all input was LOW')
            
