"""
A simple example of sending data from 1 nRF24L01 transceiver to another.
This example was written to be used on 2 devices acting as 'nodes'.
"""
import sys
import argparse
import time
import struct
from RF24 import RF24, RF24_PA_LOW
import Adafruit_ADS1x15
import configparser

## ADC SETUP
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
stick_readings = [0]*4

RF24
radio = RF24(17, 0)

def send_axes():
    radio.stopListening()
    
    # use struct.pack() to packet your data into the payload
    command_id = 3
    
    for i in range(4):
        stick_readings[i] = round((adc.read_adc(i, gain=GAIN) / 30700) * 1000)
    print (stick_readings)
    pitch = stick_readings[0] + 1000
    roll = stick_readings[1] + 1000
    throttle = stick_readings[2] + 1000
    yaw = stick_readings[3] + 1000
    aux1 = 200 + 1000
    aux2 = 0 + 1000
    aux3 = 0 + 1000
    aux4 = 0 + 1000
    
    buffer = struct.pack("sssssssss", command_id, throttle, roll, yaw, pitch, aux1, aux2, aux3, aux4)#remember to put aux3 and aux4 back in
    success = False
    while not success:
        start_timer = time.monotonic_ns()  # start timer
        result = radio.write(buffer)

        if not result:
            print("Transmission failed or timed out")
            time.sleep(1)
        else:
            success = True
            await_telemetry_data()
            end_timer = time.monotonic_ns()  # end timer
            print(
                "Transmission successful! Roundtrip time: "
                "{} us.".format(
                    (end_timer - start_timer) / 1000
                )
            )

    radio.startListening()


def await_telemetry_data(timeout = 6):
    radio.startListening()  # put radio in RX mode

    start_timer = time.monotonic()
    telem_data_present = False
    while ((time.monotonic() - start_timer) < timeout and not telem_data_present):
        has_payload, pipe_number = radio.available_pipe()
        if has_payload:
            # fetch 1 payload from RX FIFO
            #length = radio.getDynamicPayloadSize()
            buffer = radio.read(radio.payloadSize)
            # use struct.unpack() to convert the buffer into usable data
            # expecting a little endian float, thus the format string "<f"
            # buffer[:4] truncates padded 0s in case payloadSize was not set
            payload = struct.unpack("sssssssss", buffer)
            # print details about the received packet
            print(
                "Received {} bytes on pipe {}: {}".format(
                    radio.payloadSize,
                    pipe_number,
                    payload
                )
            )
            start_timer = time.monotonic()  # reset the timeout timer
            telem_data_present = True

    radio.stopListening()  # put the radio in TX mode





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
    radio.setPALevel(RF24_PA_LOW)  # RF24_PA_MAX is default

    # set TX address as base station address
    radio.openWritingPipe(base_address)  # always uses pipe 0

    # set RX address as flight controller address
    radio.openReadingPipe(1, fc_address)  # using pipe 1

    # To save time during transmission, we'll set the payload size to be only
    # what we need. A float value occupies 4 bytes in memory using
    # struct.pack(); "<f" means a little endian unsigned float
    radio.payloadSize = 18

    # for debugging, we have 2 options that print a large block of details
    # (smaller) function that prints raw register values
    # radio.printDetails()
    # (larger) function that prints human readable data
    # radio.printPrettyDetails()

    try:
        while True:
            send_axes()
            time.sleep(1)
    except KeyboardInterrupt:
        print(" Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        sys.exit()
