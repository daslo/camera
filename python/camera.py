#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 24 19:29:47 2019

@author: ds
"""
import serial
import numpy as np
from matplotlib import pyplot as plt
"""
149881
149881
149881


37561

11737

"""

TO = 15 # timeout for data read
DS = 37561 # number of bytes to read
Bd = 4000000 # serial baudrate

def make_usb():
    try:
        usb.close()
    except:
        pass
    try:
        usb=serial.Serial('/dev/ttyUSB0', baudrate=Bd, timeout=TO)
    except:
        usb=serial.Serial('/dev/ttyUSB1', baudrate=Bd, timeout=TO)
    return usb

def read_data(usb):
    print("START")
    usb.write(b"1")
    data = usb.read(1)
    print(data, "SYNC")
    data = b""
    
    data = usb.read(DS)
    return data

def image_37561A(data):
    Array = []
    data1 = [int(data[i]) for i in range(len(data))]
    for i in range(0,37561-313,313):
        row = data1[i:i+313]
        row1 =row[0:-1:2]
        row2 = [[i,i,i] for i in row1]
        Array.append(row2)
    Image = np.array(Array, dtype=np.uint8)
    return Image


if __name__ == "__main__":
    try:
        del Data
    except:
        pass
    USB = make_usb()
    USB.flush()
    USB.reset_input_buffer()
    USB.reset_output_buffer()
    Data = read_data(USB)
    Image = image_37561A(Data)
    plt.imshow(Image)
    USB.close()
    pass
