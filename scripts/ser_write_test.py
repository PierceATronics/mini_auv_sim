import time
import serial

ser = serial.Serial('/dev/picoM', 115200)

while(True):

    ser.write(bytes('H', 'utf-8'))
    time.sleep(0.10)
