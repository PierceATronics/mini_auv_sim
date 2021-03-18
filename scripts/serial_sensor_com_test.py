import time
import serial
import struct
ser = serial.Serial('/dev/picoM', 115200)


REQ_H_BYTE = 0xA4
REQ_DEPTH_CMD = 0x02
REQ_IMU_CMD = 0x03
REQ_ALL_SENSOR_CMD = 0x04
REQ_E_BYTE = 0xA0

REQ_DEPTH = bytearray([REQ_H_BYTE, REQ_DEPTH_CMD, REQ_E_BYTE])
REQ_IMU = bytearray([REQ_H_BYTE, REQ_IMU_CMD, REQ_E_BYTE])
REQ_ALL_SENSOR = bytearray([REQ_H_BYTE, REQ_ALL_SENSOR_CMD, REQ_E_BYTE])

while(True):

    ser.write(REQ_ALL_SENSOR) 
    if(ser.in_waiting):
        HEADER = ord(ser.read());
        if(hex(HEADER) == "0xa4"):
            msg_type = ord(ser.read())
            if(hex(msg_type) == "0x2"):
                data_temp = ser.read(4);
                data = struct.unpack('<f', data_temp)            
            elif(hex(msg_type) == "0x3"):
                data_temp = ser.read(12);
                data = struct.unpack('<fff', data_temp)            
            elif(hex(msg_type) == "0x4"):
                data_temp = ser.read(16);
                data = struct.unpack('<ffff', data_temp) 
            END = ord(ser.read())

            if(hex(END) == "0xa0"):
                print(data)
    time.sleep(0.10)
