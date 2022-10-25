#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import pygame
from pygame.locals import *
import time

class joyconController:
    def __init__(self):
        rospy.init_node('joycon_node', anonymous=True)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)

        # ジョイコン操作用
        pygame.joystick.init()
        self.joystick0 = pygame.joystick.Joystick(0)
        self.joystick0.init()
        self.joystickx = 0
        self.joysticky = 0
        pygame.init()

        # ボタンのカウントと速度の係数
        self.button_cnt = 0
        self.coff_linearx = 1
        self.coff_angularz = 1

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

        print 
        print '▲ : UP Linear X,  ▼ : DOWN Linear X'
        print '◀ : UP Angular Z, ▶ : DOWN Angular Z'
        print

    def timerCallback(self, event):
        twist = Twist()
        eventlist = pygame.event.get()

        # イベント処理
        for e in eventlist:
            if e.type == QUIT:
                return
            # スティックの処理
            if e.type == pygame.locals.JOYHATMOTION:
                self.button_cnt += 1
                self.joystickx, self.joysticky = self.joystick0.get_hat(0)
                print 'linear x:' + str(-self.joystickx * self.coff_linearx) + ', angular z:' + str(-self.joysticky * self.coff_angularz)
            # ボタンの処理
            elif e.type == pygame.locals.JOYBUTTONDOWN:
                self.button_cnt += 1
                if e.button==2:
                    self.coff_linearx += 0.1
                elif e.button==1:
                    self.coff_linearx -= 0.1
                    if self.coff_linearx <=0.1: self.coff_linearx = 0.1
                elif e.button==0:
                    self.coff_angularz += 0.1
                elif e.button==3:
                    self.coff_angularz -= 0.1
                    if self.coff_angularz <= 0.1: self.coff_angularz = 0.1
                print 'linear x:' + str(self.coff_linearx) + ', angular z:' + str(self.coff_angularz)

        twist.linear.x  = -self.joystickx * self.coff_linearx
        twist.angular.z = -self.joysticky * self.coff_angularz

        self.twist_pub.publish(twist)

        if self.button_cnt>=10:
            print 
            print '▲ : UP Linear X,  ▼ : DOWN Linear X'
            print '◀ : UP Angular Z, ▶ : DOWN Angular Z'
            print
            self.button_cnt = 0

if __name__ == '__main__':
    try:
        jc = joyconController()
        rospy.spin()
    except pygame.error:
        print 'joystickが見つかりませんでした。'
    except rospy.ROSInterruptException: pass