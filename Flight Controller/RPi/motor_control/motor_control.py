#!/usr/bin/env python3
import rospy
import RPi.GPIO as IO
import time

#Pin configuration
throttle_pin = 26
roll_pin = 19
pitch_pin = 13
yaw_pin = 12
aux1_pin = 6
aux2_pin = 5

pwm_freq = 490

if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    rospy.loginfo('motor_control_node started')
    IO.setwarnings(False)
    IO.setmode (IO.BCM)
    rate = rospy.Rate(10)

    IO.setup(throttle_pin, IO.OUT)
    IO.setup(roll_pin, IO.OUT)
    IO.setup(pitch_pin, IO.OUT)
    IO.setup(yaw_pin, IO.OUT)
    IO.setup(aux1_pin, IO.OUT)
    IO.setup(aux2_pin, IO.OUT)

    throttle = IO.PWM(throttle_pin,pwm_freq)
    roll = IO.PWM(roll_pin,pwm_freq)
    pitch = IO.PWM(pitch_pin,pwm_freq)
    yaw = IO.PWM(yaw_pin,pwm_freq)
    aux1 = IO.PWM(aux1_pin,pwm_freq)
    aux2 = IO.PWM(aux2_pin,pwm_freq)

    throttle.start(0)
    roll.start(0)
    pitch.start(0)
    yaw.start(0)
    aux1.start(0)
    aux2.start(0)

    while not rospy.is_shutdown():
        throttle.ChangeDutyCycle(53)
        #roll.ChangeDutyCycle(74)
        #pitch.ChangeDutyCycle(67)
        #yaw.ChangeDutyCycle(60)
        #aux1.ChangeDutyCycle(53)
        #aux2.ChangeDutyCycle(46)
        rospy.loginfo('Hello')
        rate.sleep()
