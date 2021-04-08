#!/usr/bin/env python3

import os
import rospy
import RPi.GPIO as IO
import time
os.system ("sudo pigpiod")
time.sleep(1)
import pigpio
from std_msgs.msg import Int16MultiArray

motor1 = 23
motor2 = 27
motor3 = 22
motor4 = 18

pi = pigpio.pi()
pi.set_servo_pulsewidth(motor1, 0)
pi.set_servo_pulsewidth(motor2, 0)
pi.set_servo_pulsewidth(motor3, 0)
pi.set_servo_pulsewidth(motor4, 0) 

max_value = 2000
min_value = 700

def manual_drive(): #You will use this function to program your ESC if required
    print( "You have selected manual option so give a value between 0 and you max value"  )  
    while True:
        inp = input()
        if inp == "stop":
            stop()
            break
        elif inp == "control":
            control()
            break
        elif inp == "arm":
            arm()
            break	
        else:
            pi.set_servo_pulsewidth(ESC,inp)
                
def calibrate():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC, 0)
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == '':
        pi.set_servo_pulsewidth(ESC, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input()
        if inp == '':            
            pi.set_servo_pulsewidth(ESC, min_value)
            print( "Wierd eh! Special tone")
            time.sleep(7)
            print( "Wait for it ....")
            time.sleep (5)
            print( "Im working on it, DONT WORRY JUST WAIT.....")
            pi.set_servo_pulsewidth(ESC, 0)
            time.sleep(2)
            print( "Arming ESC now...")
            pi.set_servo_pulsewidth(ESC, min_value)
            time.sleep(1)
            print( "See.... uhhhhh")
            control() # You can change this to any other function you want
            
def control(): 
    print( "I'm Starting the motor, I hope its calibrated and armed, if not restart by giving 'x'")
    time.sleep(1)
    speed = 1500    # change your speed if you want to.... it should be between 700 - 2000
    print( "Controls - a to decrease speed & d to increase speed OR q to decrease a lot of speed & e to increase a lot of speed")
    while True:
        pi.set_servo_pulsewidth(ESC, speed)
        inp = input()
        
        if inp == "q":
            speed -= 100    # decrementing the speed like hell
            print( "speed = " + str(speed))
        elif inp == "e":    
            speed += 100    # incrementing the speed like hell
            print( "speed = " + str(speed))
        elif inp == "d":
            speed += 10     # incrementing the speed 
            print( "speed = " + str(speed))
        elif inp == "a":
            speed -= 10     # decrementing the speed
            print ("speed = " + str(speed))
        elif inp == "stop":
            stop()          #going for the stop function
            break
        elif inp == "manual":
            manual_drive()
            break
        elif inp == "arm":
            arm()
            break	
        else:
            print( "WHAT DID I SAID!! Press a,q,d or e")
            
def arm(): #This is the arming procedure of an ESC 
    
    pi.set_servo_pulsewidth(motor1, 0)
    pi.set_servo_pulsewidth(motor2, 0)
    pi.set_servo_pulsewidth(motor3, 0)
    pi.set_servo_pulsewidth(motor4, 0)
    time.sleep(1)
    pi.set_servo_pulsewidth(motor1, max_value)
    pi.set_servo_pulsewidth(motor2, max_value)
    pi.set_servo_pulsewidth(motor3, max_value)
    pi.set_servo_pulsewidth(motor4, max_value)
    time.sleep(1)
    pi.set_servo_pulsewidth(motor1, min_value)
    pi.set_servo_pulsewidth(motor2, min_value)
    pi.set_servo_pulsewidth(motor3, min_value)
    pi.set_servo_pulsewidth(motor4, min_value)
    time.sleep(10)
    pi.set_servo_pulsewidth(motor1, 1500)
    pi.set_servo_pulsewidth(motor2, 1500)
    pi.set_servo_pulsewidth(motor3, 1500)
    pi.set_servo_pulsewidth(motor4, 1500)
    #control() 
        
def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(motor1, 0)
    pi.set_servo_pulsewidth(motor2, 0)
    pi.set_servo_pulsewidth(motor3, 0)
    pi.set_servo_pulsewidth(motor4, 0)
    pi.stop()

def axis_control_callback(msg):
    #rospy.loginfo(msg.data[1])
    motor1_level = msg.data[0]
    motor2_level = msg.data[1]
    motor3_level = msg.data[2]
    motor4_level = msg.data[3]

    pi.set_servo_pulsewidth(motor1, motor1_level)
    pi.set_servo_pulsewidth(motor2, motor2_level)
    pi.set_servo_pulsewidth(motor3, motor3_level)
    pi.set_servo_pulsewidth(motor4, motor4_level)
    


if __name__ == '__main__':
    arm()
    time.sleep(1)

    rospy.init_node('motor_control_node')
    rospy.loginfo('motor_control_node started')
    rate = rospy.Rate(1000)
    sub = rospy.Subscriber('/motor_levels', Int16MultiArray, axis_control_callback)


    rospy.spin()
    
    stop()