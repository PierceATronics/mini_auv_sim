import serial
import time
import struct

ser = serial.Serial('/dev/picoMaestroM', 115200)

pulse_width =1250.0 

#Convert the pulse width to the compact protocol expected by the maestro.
a = int(pulse_width * 4)
lower_bits = a & 0x7f
upper_bits = (a >> 7) & 0x7f
pulse_width_packed = struct.pack('>hh', lower_bits, upper_bits)

while(True):
    
    #iterate through each channel
    for i in [1, 3, 4, 6]:
        message = bytearray([0x84, i, pulse_width_packed[1], pulse_width_packed[3]])
        ser.write(message)

    time.sleep(1)


