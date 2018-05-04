#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import time
import sys
import traceback
from math import pi, cos, sin

com_port = "/dev/ttyUSB0"  # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200

offset = 140
xdata = [0] * 360
ydata = [0] * 360
cdata = ["black" for x in range(360)]

fig = plt.figure()
scatter = plt.scatter(xdata, ydata, color=cdata)
def update_plot(*args, **kwargs):
    scatter.set_offsets(np.c_[xdata, ydata])
    scatter.set_color(cdata)
    return scatter
ani = animation.FuncAnimation(fig, update_plot, interval=10)

def update_view( angle, data ):
    """Updates the view of a sample.

    Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
    """
    angle = -1 * (angle -105)
    if angle < 0:
        angle += 360
    angle %= 360
    data_time = time.time()

    x = data[0]
    x1= data[1]
    x2= data[2]
    x3= data[3]

    angle_rad = angle * pi / 180.0
    c = cos(angle_rad)
    s = -sin(angle_rad)

    dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # quality is on 16 bits

    dist_x = dist_mm*c
    dist_y = dist_mm*s

    # display the sample
    xdata[angle] = dist_x
    ydata[angle] = dist_y
    if not(x1 & 0x80): # is the flag for "bad data" set?
        # no, it's cool
        if not x1 & 0x40:
            # X+1:6 not set : quality is OK
            cdata[angle] = "green"
            return
    cdata[angle] = "red"

def checksum(data):
    """Compute and return the checksum as an int.

    data -- list of 20 bytes (as ints), in the order they arrived in.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )

def read_v_2_4():
    global angle, index
    init_level = 0

    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 :
                b = ser.read(1)[0]
                # start byte
                if b == 0xFA : 
                    init_level = 1
                else:
                    init_level = 0
            elif init_level == 1:
                # position index 
                b = ser.read(1)[0]
                if b >= 0xA0 and b <= 0xF9  : 
                    index = b - 0xA0
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # speed
                b_speed = [ b for b in ser.read(2)]
                
                # data
                b_data0 = [ b for b in ser.read(4)]
                b_data1 = [ b for b in ser.read(4)]
                b_data2 = [ b for b in ser.read(4)]
                b_data3 = [ b for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion... 
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3 

                # checksum  
                b_checksum = [ b for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    update_view(index * 4 + 0, b_data0)
                    update_view(index * 4 + 1, b_data1)
                    update_view(index * 4 + 2, b_data2)
                    update_view(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    update_view(index * 4 + 3, [0, 0x80, 0, 0])
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except :
            traceback.print_exc(file=sys.stdout)


import serial
ser = serial.Serial(com_port, baudrate)
thread = threading.Thread(target=read_v_2_4)
thread.daemon = True
thread.start()
plt.xlim([-3000,3000])
plt.ylim([-3000,3000])
plt.show()
