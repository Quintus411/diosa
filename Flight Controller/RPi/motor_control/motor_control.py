import rospy
import RPi.GPIO as IO
import time

if __name__ == '__main__':
    rospy.init_node('motor_control')
    rospy.loginfo('motor_control node started')
    IO.setwarnings(False) #do not show any warnings
    IO.setmode (IO.BCM)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo('Hello')
        rate.sleep()