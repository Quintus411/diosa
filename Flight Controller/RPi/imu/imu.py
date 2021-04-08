#!/usr/bin/env python3

""" This program asks a client for data and waits for the response, then sends an ACK. """

# Copyright 2018 Rui Silva.
#
# This file is part of rpsreal/pySX127x, fork of mayeranalytics/pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.

import time
import datetime
import configparser
import rospy
from std_msgs.msg import Float32MultiArray
import sys
import struct
import smbus
import math


def MPU6050_start():
    # alter sample rate (stability)
    samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    # reset all sensors
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x00)
    time.sleep(0.1)
    # power management and crystal settings
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #Write to Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    #Write to Gyro configuration register
    gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
    gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
    gyro_indx = 0
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    #Write to Accel configuration register
    accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
    accel_indx = 0                            
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]
    
def read_raw_bits(register):
    # read accel and gyro values
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register+1)

    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def mpu6050_conv():
    # raw acceleration bits
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)

    # raw temp bits
##    t_val = read_raw_bits(TEMP_OUT_H) # uncomment to read temp
    
    # raw gyroscope bits
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)

    #convert to acceleration in g and gyro dps
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens

    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens

##    temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
    return a_x,a_y,a_z,w_x,w_y,w_z

    
# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

if __name__ == '__main__':

    # start I2C driver
    bus = smbus.SMBus(1) # start comm with i2c bus
    gyro_sens,accel_sens = MPU6050_start() # instantiate gyro/accel

    rospy.sleep(1)

    rospy.init_node('imu_node')
    rospy.loginfo('imu_node started')
    global pub
    pub = rospy.Publisher('/imu_readings', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(1000)

    total_angle_X = 0
    total_angle_Y = 0
    total_angle_Z = 0

    x_offset = 3.3
    y_offset = -5

    current_time = datetime.datetime.now()

    while not rospy.is_shutdown():
        
        prev_time = current_time

        try:
            ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
            #mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
        except:
            continue
        
        #print('{}'.format('-'*30))
        #print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z {2:2.2f}= '.format(ax,ay,az))
        #print('gyro [dps]:  x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(wx,wy,wz))
        #print('{}'.format('-'*30))

        #Acceleration_angle[0] = atan( ay /sqrt(pow(ax,2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg

        accel_X = math.degrees( math.atan( ay / math.sqrt( math.pow(ax, 2) + math.pow(az, 2) ) ) )
        accel_Y = math.degrees( math.atan2( ax , math.sqrt( math.pow(ay, 2) + math.pow(az, 2) ) ) )
        #accel_Y = math.degrees( math.atan2( ay , az ) )

        current_time = datetime.datetime.now()
        elapsed_time = (current_time - prev_time).total_seconds()

        total_angle_X = 0.98 * (total_angle_X + wx * elapsed_time) + 0.02 * accel_X# + x_offset
        total_angle_Y = 0.98 * (total_angle_Y + wy * elapsed_time) + 0.02 * accel_Y# + y_offset
        total_angle_Z = total_angle_Z + wz * elapsed_time

        print_string = 'X: ' + str(int(total_angle_X)) + '        Y: ' + str(int(total_angle_Y)) + '        Z: ' + str(int(total_angle_Z))
        print(print_string, end = "\r")

        msg = Float32MultiArray()
        msg.data = [total_angle_X, total_angle_Y, total_angle_Z]
        pub.publish(msg)
        rate.sleep()


    