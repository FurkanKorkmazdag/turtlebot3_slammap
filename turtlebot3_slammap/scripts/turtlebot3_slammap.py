#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 20:36:23 2025

@author: furkan
"""

import cv2
import numpy as np 
import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tf
import subprocess

class TurtleBotKontrol():
    def __init__(self):
        rospy.init_node("turtlebot3_slammap")
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaj = Twist()
        rospy.Subscriber("scan", LaserScan, self.lazerCallback)
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_meaj = Twist()
        self.tur = 0
        
        self.min_on = 0.0
        self.min_sol = 0.0
        self.min_sag = 0.0
        self.min_arka = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.teta = 0.0
        
        self.map = False
        
        rospy.spin()

    def lazerCallback(self, mesaj):
        sol_on = list(mesaj.ranges[0:10])
        sag_on = list(mesaj.ranges[349:359])
        on = sol_on + sag_on
        sol = list(mesaj.ranges[80:100])
        sag = list(mesaj.ranges[260:280])
        arka = list(mesaj.ranges[170:190])

        self.min_on = min(on)
        self.min_sol = min(sol)
        self.min_sag = min(sag)
        self.min_arka = min(arka)

        self.HareketFonksiyonu()

    def odomCallback(self, data):

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.teta = yaw
        
    def kameraCallback(self,mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        alt_sari = np.array([20,100,100])
        ust_sari = np.array([40,255,255])
        mask = cv2.inRange(hsv,alt_sari,ust_sari)
        h,w,d = img.shape
        M = cv2.moments(mask)
        if M['m00']>0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img,(cx,cy),3,(0,0,255),-1)
            if self.map==False:
                subprocess.call(['rosrun', 'map_server', 'map_saver', '-f', '/home/furkan/Resimler/Map'])
                self.map = True

            
        cv2.imshow("Kamera",img)
        cv2.waitKey(1)
        
    def HareketFonksiyonu(self):
        
        """Laser verilerine göre hareket mantığı"""
        if self.min_on > 1.0:  
            if self.teta > (-177* 3.14159 / 180) and self.teta <= (-145 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = -0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (3 * 3.14159 / 180) and self.teta <= (35* 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = -0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (55 * 3.14159 / 180) and self.teta <= (87 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = 0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (93 * 3.14159 / 180) and self.teta <= (125 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = -0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (145* 3.14159 / 180) and self.teta <= (177 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = 0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (-35 * 3.14159 / 180) and self.teta <= (-3 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = 0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (-87 * 3.14159 / 180) and self.teta <= (-55 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = -0.3
                self.pub.publish(self.hiz_mesaj)
            elif self.teta > (-125* 3.14159 / 180) and self.teta <= (-93 * 3.14159 / 180):
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = 0.3
                self.pub.publish(self.hiz_mesaj)
            else:
                self.hiz_mesaj.linear.x = 0.3
                self.hiz_mesaj.angular.z = 0.0
                self.pub.publish(self.hiz_mesaj)


                
        else:
            if self.min_sag > self.min_sol:  
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = -0.5  
                self.pub.publish(self.hiz_mesaj)
                time.sleep(0.33)
            elif self.min_sag>1.9 and self.min>1.9:
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = 0.5  
                self.pub.publish(self.hiz_mesaj)
                time.sleep(0.33)
            else: 
                self.hiz_mesaj.linear.x = 0.0
                self.hiz_mesaj.angular.z = 0.5
                self.pub.publish(self.hiz_mesaj)
                time.sleep(0.33)
        

if __name__ == "__main__":
    TurtleBotKontrol()