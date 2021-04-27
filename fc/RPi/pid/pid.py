#!/usr/bin/env python3

import os
import rospy
import pigpio
import time
from std_msgs.msg import Int16MultiArray, Float32MultiArray
import datetime
import math
import configparser

config = configparser.ConfigParser()
config.read('config.cfg')

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

throttle = 1500
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
    ###### PID VALUES ######
    roll_kp = 20#float(config['vars']['roll_kp'])
    roll_ki = 20#float(config['vars']['roll_ki'])
    roll_kd = 20#float(config['vars']['roll_kd'])

    pitch_kp = 20#float(config['vars']['pitch_kp'])
    pitch_ki = 20#float(config['vars']['pitch_ki'])
    pitch_kd = 20#float(config['vars']['pitch_kd'])

    yaw_kp = 20#float(config['vars']['yaw_kp'])
    yaw_ki = 20#float(config['vars']['yaw_ki'])
    yaw_kd = 20#float(config['vars']['yaw_kd'])


    ###### PID Aux Variables ######
    roll_integral_error = 0
    pitch_integral_error = 0
    yaw_integral_error = 0

    roll_prev_error = 0
    pitch_prev_error = 0
    yaw_prev_error = 0

    I_THR = 200#float(config['vars']['I_THR'])
    length = 0.200#float(config['vars']['length'])
    height = 0.200#float(config['vars']['height'])
    mass = 1.2#float(config['vars']['mass'])

    max_motor_speed = 2500#int(config['vars']['max_motor_speed'])
    min_motor_speed = 500#int(config['vars']['min_motor_speed'])

    motor1 = 0
    motor2 = 0
    motor3 = 0
    motor4 = 0

    
    rospy.init_node('pid_node')
    rospy.loginfo('pid_node started')
    rate = rospy.Rate(1000)
    sub = rospy.Subscriber('/rx_tx', Int16MultiArray, update_axis_values)
    imu_sub = rospy.Subscriber('/imu_readings', Float32MultiArray, update_imu_values)
    global pub
    pub = rospy.Publisher('/motor_levels', Int16MultiArray, queue_size=10)
    
    current_time = datetime.datetime.now()
    T = [0, 0, 0, 0]

    while not rospy.is_shutdown():
        
        prev_time = current_time
        current_time = datetime.datetime.now()
        elapsed_time = (current_time - prev_time).total_seconds()

        ########################### ROLL CALCULATION
        roll_error = roll - imu_roll
        roll_p = roll_kp * roll_error
        roll_i = roll_ki * roll_error * elapsed_time + roll_integral_error
        roll_d = roll_kd * (roll_error - roll_prev_error) * elapsed_time
        roll_prev_error = roll_error
        roll_integral_error = roll_integral_error + roll_ki * roll_error * elapsed_time
        if (math.fabs(roll_i) > I_THR):
            roll_integral_error = I_THR if (roll_integral_error > 0) else (0 - I_THR)
            roll_i = I_THR if (roll_prev_error > 0) else (0 - I_THR)

        ctrl_roll = roll_p + roll_i + roll_d
        ############################ END ROLL CALCULATION

        ########################### PITCH CALCULATION
        pitch_error = pitch - imu_pitch
        pitch_p = pitch_kp * pitch_error
        pitch_i = pitch_ki * pitch_error * elapsed_time + pitch_integral_error
        pitch_d = pitch_kd * (pitch_error - pitch_prev_error) * elapsed_time
        pitch_prev_error = pitch_error
        pitch_integral_error = pitch_integral_error + pitch_ki * pitch_error * elapsed_time
        if (math.fabs(pitch_i) > I_THR):
            pitch_integral_error = I_THR if (pitch_integral_error > 0) else (0 - I_THR)
            pitch_i = I_THR if (pitch_prev_error > 0) else (0 - I_THR)

        ctrl_pitch = pitch_p + pitch_i + pitch_d
        ############################ END PITCH CALCULATION

        ########################### YAW CALCULATION
        yaw_error = yaw - imu_yaw
        yaw_p = yaw_kp * yaw_error
        yaw_i = yaw_ki * yaw_error * elapsed_time + yaw_integral_error
        yaw_d = yaw_kd * (yaw_error - yaw_prev_error) * elapsed_time
        yaw_prev_error = yaw_error
        yaw_integral_error = yaw_integral_error + yaw_ki * yaw_error * elapsed_time
        if (math.fabs(yaw_i) > I_THR):
            yaw_integral_error = I_THR if (yaw_integral_error > 0) else (0 - I_THR)
            yaw_i = I_THR if (yaw_prev_error > 0) else (0 - I_THR)

        ctrl_yaw = yaw_p + yaw_i + yaw_d
        ############################ END YAW CALCULATION

        ########## APPLY THROTTLE
        T[0] = throttle
        T[1] = throttle
        T[2] = throttle
        T[3] = throttle

        ########## APPLY GRAVITY COMPENSATION
        mg_factor = ((mass * 9.81) / 4) * math.cos(imu_pitch) * math.cos(imu_roll)

        T[0] += mg_factor
        T[1] += mg_factor
        T[2] += mg_factor
        T[3] += mg_factor

        ########## APPLY PITCH
        alpha = height * elapsed_time / (2 * 0.0907)
        T[0] += ctrl_pitch / alpha
        T[1] += ctrl_pitch / alpha
        T[2] -= ctrl_pitch / alpha
        T[3] -= ctrl_pitch / alpha

        ########## APPLY ROLL
        beta = length * elapsed_time / (2 * 0.5005)
        T[0] += ctrl_roll / beta
        T[1] -= ctrl_roll / beta
        T[2] -= ctrl_roll / beta
        T[3] += ctrl_roll / beta

        ########## APPLY YAW
        T[0] += ctrl_yaw 
        T[1] -= ctrl_yaw
        T[2] += ctrl_yaw
        T[3] -= ctrl_yaw

        for i in range(4):
            if (T[i] > max_motor_speed):
                T[i] = max_motor_speed
            elif (T[i] < min_motor_speed):
                T[i] = min_motor_speed

        motor1 = int(T[2])
        motor2 = int(T[1])
        motor3 = int(T[3])
        motor4 = int(T[0]) 

        msg = Int16MultiArray()
        msg.data = [motor1, motor2, motor3, motor4]
        pub.publish(msg)
        rate.sleep()