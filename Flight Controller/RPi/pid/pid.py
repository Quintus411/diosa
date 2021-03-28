#!/usr/bin/python

import os
import rospy
import RPi.GPIO as IO
import pigpio
import time
import PPM
from std_msgs.msg import Int16MultiArray

global ax
global ay
global az
global gx
global gy
global gz

ax = 0
ay = 0
az = 0
gx = 0
gy = 0
gz = 0

global throttle
global roll
global pitch
global yaw
global aux1
global aux2
global aux3
global aux4

throttle = 0
roll = 0
pitch = 0
yaw = 0
aux1 = 0
aux2 = 0
aux3 = 0
aux4 = 0

def update_imu_values(msg):
    global ax
    global ay
    global az
    global gx
    global gy
    global gz

    ax = msg.data[0]
    ay = msg.data[1]
    az = msg.data[2]
    gx = msg.data[3]
    gy = msg.data[4]
    gz = msg.data[5]

def update_axis_values(msg):
    #rospy.loginfo(msg.data[1])

    global throttle
    global roll
    global pitch
    global yaw
    global aux1
    global aux2
    global aux3
    global aux4

    throttle = msg.data[0]
    roll = msg.data[1]
    pitch = msg.data[2]
    yaw = msg.data[3]
    aux1 = msg.data[4]
    aux2 = msg.data[5]
    aux3 = msg.data[6]
    aux4 = msg.data[7]


if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    rospy.loginfo('motor_control_node started')
    rate = rospy.Rate(1000)
    sub = rospy.Subscriber('/rx_tx', Int16MultiArray, update_axis_values)
    imu_sub = rospy.Subscriber('/imu_readings', Float32MultiArray, update_imu_values)
    global pub
    pub = rospy.Publisher('/motor_levels', Int16MultiArray, queue_size=10)
    
    while not rospy.is_shutdown():
        #hello
        #RANDOM MATH BULLSHIT HERE
        motor1 = 1500
        motor2 = 1500
        motor3 = 1500
        motor4 = 1500
        msg = Int16MultiArray()
        msg.data = [motor1, motor2, motor3, motor4]
        pub.publish(msg)
        rate.sleep()