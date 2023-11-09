"""
Proof of concept for transmitting data to the pi
"""

import struct
import time

from smbus2 import SMBus

ARD_ADDR = 8


def i2c_setup():
    return SMBus(1)


def main():
    i2cBus = i2c_setup()

    #while True:
        # Get string to send
    #    user_in = float(input("float (string) to send: "))

    #    send_bytes = bytearray(struct.pack("f", user_in))

    #    i2cBus.write_i2c_block_data(ARD_ADDR, 0, send_bytes)

    data = i2cBus.read_block_data(ARD_ADDR, 4)
    print(data)



if __name__ == "__main__":
    main()
