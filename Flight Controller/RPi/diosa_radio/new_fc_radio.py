"""
A simple example of sending data from 1 nRF24L01 transceiver to another.
This example was written to be used on 2 devices acting as 'nodes'.
"""
import sys
import argparse
import time
import struct
from RF24 import RF24, RF24_PA_LOW


########### USER CONFIGURATION ###########
# See https://github.com/TMRh20/RF24/blob/master/pyRF24/readme.md
# Radio CE Pin, CSN Pin, SPI Speed
# CE Pin uses GPIO number with BCM and SPIDEV drivers, other platforms use
# their own pin numbering
# CS Pin addresses the SPI bus number at /dev/spidev<a>.<b>
# ie: RF24 radio(<ce_pin>, <a>*10+<b>); spidev1.0 is 10, spidev1.1 is 11 etc..

# Generic:
radio = RF24(17, 0)
################## Linux (BBB,x86,etc) #########################
# See http://nRF24.github.io/RF24/pages.html for more information on usage
# See http://iotdk.intel.com/docs/master/mraa/ for more information on MRAA
# See https://www.kernel.org/doc/Documentation/spi/spidev for more
# information on SPIDEV

# using the python keyword global is bad practice. Instead we'll use a 1 item
# list to store our float number for the payloads sent/received
def process_axes(payload):
    throttle = payload[1]
    roll = payload[2]
    yaw = payload[3]
    pitch = payload[4]
    aux_1 = payload[5]
    aux_2 = payload[6]
    aux_3 = payload[7]
    aux_4 = payload[8]

    print(
        "Throttle:{} | Roll:{} | Yaw:{} | Pitch:{} | Aux1: {} | Aux2: {} | Aux3: {} | Aux4: {} ".format(
            throttle,
            roll,
            yaw,
            pitch,
            aux_1,
            aux_2,
            aux_3,
            aux_4
        )
    )

def send_telemetry():
    radio.stopListening()
    
    # use struct.pack() to packet your data into the payload
    accelerometer = 420
    gyroscope = 421
    barometer = 422
    buffer = struct.pack("sssssssss", accelerometer, gyroscope, barometer, 0, 0, 0, 0, 0, 0)
    success = False
    while not success:
        start_timer = time.monotonic_ns()  # start timer
        result = radio.write(buffer)
        end_timer = time.monotonic_ns()  # end timer
        if not result:
            print("Transmission failed or timed out")
            time.sleep(1)
        else:
            success = True
            print(
                "Transmission successful! Time to Transmit: "
                "{} us.".format(
                    (end_timer - start_timer) / 1000
                )
            )

    radio.startListening()

def await_request():
    radio.startListening()  # put radio in RX mode
    print("awaiting request...")
    while True:
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
            
            command_id = payload[0]
            if command_id == 3: # process axis commands and send telemetry data
                process_axes(payload)
                send_telemetry()






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
    radio.openWritingPipe(fc_address)  # always uses pipe 0

    # set RX address as flight controller address
    radio.openReadingPipe(1, base_address)  # using pipe 1

    # To save time during transmission, we'll set the payload size to be only
    # what we need. A float value occupies 4 bytes in memory using
    # struct.pack(); "<f" means a little endian unsigned float
    radio.payloadSize = 18
    #radio.enableDynamicPayloads()
    # for debugging, we have 2 options that print a large block of details
    # (smaller) function that prints raw register values
    # radio.printDetails()
    # (larger) function that prints human readable data
    # radio.printPrettyDetails()

    try:
        await_request()
    except KeyboardInterrupt:
        print(" Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        sys.exit()
