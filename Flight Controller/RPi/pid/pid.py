#!/usr/bin/env python3

import os
import rospy
import pigpio
import time
from std_msgs.msg import Int16MultiArray, Float32MultiArray

global imu_pitch
global imu_roll
global imu_yaw

imu_pitch = 0
imu_roll = 0
imu_yaw = 0

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
    global imu_pitch
    global imu_roll
    global imu_yaw

    imu_pitch = msg.data[0]
    imu_roll = msg.data[1]
    imu_yaw = msg.data[2]

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
    rospy.init_node('pid_node')
    rospy.loginfo('pid_node started')
    rate = rospy.Rate(1000)
    sub = rospy.Subscriber('/rx_tx', Int16MultiArray, update_axis_values)
    imu_sub = rospy.Subscriber('/imu_readings', Float32MultiArray, update_imu_values)
    global pub
    pub = rospy.Publisher('/motor_levels', Int16MultiArray, queue_size=10)
    
    while not rospy.is_shutdown():
        #hello
        #RANDOM MATH BULLSHIT HERE
        motor1 = 1510
        motor2 = 1510
        motor3 = 1510
        motor4 = 1510
        msg = Int16MultiArray()
        msg.data = [motor1, motor2, motor3, motor4]
        pub.publish(msg)
        rate.sleep()