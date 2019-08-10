#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 10 16:54:37 2019

@author: ds
"""
import serial
import numpy as np
from matplotlib import pyplot as plt

TO = 14 # timeout for data read
DS = 1000000 # number of bytes to read
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
    
    usb.flush()
    usb.reset_input_buffer()
    usb.reset_output_buffer()
    return usb

def read_data(usb):
    print("START")
    usb.write(b"1")
    data = usb.read(1)
    print(data, "SYNC")
    data = b""
    
    data = usb.read(DS)
    return data

if __name__ == "__main__":
    try:
        del Data
    except:
        pass
    USB = make_usb()
    Data = read_data(USB)
    Data1 = [int(Data[i]) for i in range(len(Data))]
    print(len(Data))
    # Image = image_37561A(Data)
    # plt.imshow(Image)
    
    USB.close()
    pass