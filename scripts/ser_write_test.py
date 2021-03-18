import time
import serial

ser = serial.Serial('/dev/picoM', 115200)


REQ_H_BYTE = 0xA4
REQ_DEPTH_BYTE = 0x02
REQ_E_BYTE = 0xA0

REQ_DEPTH = bytearray([REQ_H_BYTE, REQ_DEPTH_BYTE, REQ_E_BYTE])

while(True):

    ser.write(REQ_DEPTH)
    time.sleep(0.10)
