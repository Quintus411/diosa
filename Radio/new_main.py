"""
A simple example of sending data from 1 nRF24L01 transceiver to another.
This example was written to be used on 2 devices acting as 'nodes'.
"""
import sys
import argparse
import time
import struct
from RF24 import RF24, RF24_PA_LOW, RF24_PA_MAX, RF24_2MBPS
import Adafruit_ADS1x15
import configparser
import RPi.GPIO as GPIO

## ADC SETUP
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
stick_readings = [0]*4

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

sw1 = 5
sw2 = 6
sw3 = 12
sw4 = 13

GPIO.setup(sw1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(sw2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(sw3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(sw4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

global radio
radio = RF24(17, 0)
global packets_sent
packets_sent = 0
global failed_packets
failed_packets = 0

def await_telemetry_data(timeout = 6):
    radio.startListening()  # put radio in RX mode

    start_timer = time.monotonic()
    telem_data_present = False
    payload = []
    while ((time.monotonic() - start_timer) < timeout and not telem_data_present):
        has_payload, pipe_number = radio.available_pipe()
        if has_payload:
            # fetch 1 payload from RX FIFO
            #length = radio.getDynamicPayloadSize()
            buffer = radio.read(radio.payloadSize)
            
            payload = struct.unpack("hhhhhhhhh", buffer)
            
            start_timer = time.monotonic()  # reset the timeout timer
            telem_data_present = True

    radio.stopListening()  # put the radio in TX mode
    return payload

def send_axes():
    global packets_sent
    global failed_packets
    radio.stopListening()
    
    # use struct.pack() to packet your data into the payload
    command_id = 3
    
    for i in range(4):
        stick_readings[i] = round((adc.read_adc(i, gain=GAIN) / 30700) * 1000)
    #print (stick_readings)
    pitch = stick_readings[0] + 1000
    roll = stick_readings[1] + 1000
    throttle = stick_readings[2] + 1000
    yaw = stick_readings[3] + 1000
    aux1 = 1000 if GPIO.input(sw1) else 2000
    aux2 = 1000 if GPIO.input(sw2) else 2000
    aux3 = 1000 if GPIO.input(sw3) else 2000
    aux4 = 1000 if GPIO.input(sw4) else 2000
    
    buffer = struct.pack("hhhhhhhhh", command_id, throttle, roll, yaw, pitch, aux1, aux2, aux3, aux4)
    success = False
    while not success:
        start_timer = time.monotonic_ns()  # start timer
        packets_sent = packets_sent + 1
        result = radio.write(buffer)

        if not result:
            #err_string = 'Transmission failed or timed out'
            #print(err_string)
            failed_packets = failed_packets + 1
            
        else:
            success = True
            has_payload, pipe_number = radio.available_pipe()
            end_timer = time.monotonic_ns()  # end timer
            if has_payload:
                length = radio.getDynamicPayloadSize()
                response_buffer = radio.read(length)
                response = struct.unpack("hhhhhhhhh", response_buffer)
                t_pitch = response[0]
                t_roll = response[1]
                t_yaw = response[2]
                packet_loss = (failed_packets / packets_sent) * 100
                print("Pitch: {} | Roll: {} | Yaw: {} | Packet Loss: {}% | Round Trip Time (us): {}".format(t_pitch, t_roll, t_yaw, packet_loss, (end_timer - start_timer) / 1000), end='\r')
            else :
                print("\nno payload")
            #print((end_timer - start_timer)/1000, end='\r')
            #print("Pitch: {} | Roll: {} | Yaw: {} | Time: {}".format(t_pitch, t_roll, t_yaw, (end_timer - start_timer) / 1000))

    radio.startListening()





if __name__ == "__main__":

    # initialize the nRF24L01 on the spi bus
    if not radio.begin():
        raise RuntimeError("radio hardware is not responding")

    # For this example, we will use different addresses
    # An address need to be a buffer protocol object (bytearray)
    base_address = b"base_station"
    fc_address = b"flight_controller"
    # It is very helpful to think of an address as a path instead of as
    # an identifying device destination

    # set the Power Amplifier level to -12 dBm since this test example is
    # usually run with nRF24L01 transceivers in close proximity of each other
    radio.setPALevel(RF24_PA_MAX)  # RF24_PA_MAX is default
    radio.setDataRate(RF24_2MBPS)

    # set TX address as base station address
    radio.openWritingPipe(base_address)  # always uses pipe 0

    # set RX address as flight controller address
    radio.openReadingPipe(1, fc_address)  # using pipe 1

    # To save time during transmission, we'll set the payload size to be only
    # what we need. A float value occupies 4 bytes in memory using
    # struct.pack(); "<f" means a little endian unsigned float
    #radio.payloadSize = 18
    radio.enableAckPayload()

    # for debugging, we have 2 options that print a large block of details
    # (smaller) function that prints raw register values
    # radio.printDetails()
    # (larger) function that prints human readable data
    # radio.printPrettyDetails()

    try:
        while True:
            send_axes()
            #time.sleep(1)
    except KeyboardInterrupt:
        print(" Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        sys.exit()
