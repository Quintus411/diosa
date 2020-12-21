#!/usr/bin/python
import rospy
import RPi.GPIO as IO
import pigpio
import time
import PPM
from std_msgs.msg import Int16MultiArray

#Pin configuration
throttle_pin = 26

def axis_control_callback(msg):
    #rospy.loginfo(msg.data[1])
    throttle = msg.data[0]
    roll = msg.data[1]
    pitch = msg.data[2]
    yaw = msg.data[3]
    aux1 = msg.data[4]
    aux2 = msg.data[5]
    aux3 = msg.data[6]
    aux4 = msg.data[7]
    print("pitch: " + str(pitch) + "    roll: " + str(roll) + "    throttle: " + str(throttle) + "    yaw: " + str(yaw) + "    aux1: " + str(aux1) + "    aux2: " + str(aux2) + "    aux3: " + str(aux3) + "    aux4: " + str(aux4))
    ppm.update_channels([throttle, roll, pitch, yaw, aux1, aux2, aux3, aux4])


if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    rospy.loginfo('motor_control_node started')
    rate = rospy.Rate(10)
    sub = rospy.Subscriber('/rx_tx', Int16MultiArray, axis_control_callback)

    pi = pigpio.pi()
    pi.wave_tx_stop()
    global ppm
    ppm = PPM.X(pi, throttle_pin, frame_ms=20)

    rospy.spin()
    
    ppm.cancel()
    pi.stop()