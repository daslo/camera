#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 24 19:29:47 2019

@author: ds
"""
import serial
import numpy as np
from matplotlib import pyplot as plt
from pyzbar.pyzbar import decode
import time


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

def read_data(usb, n):
    print("START")
    usb.write(b"1")
    data = usb.read(1)
    print(data, "SYNC")
    data = b""
    
    data = usb.read(n)
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

def image_151441A(data):
    Array = []
    data1 = [int(data[i]) for i in range(len(data))]
    for i in range(0,151441-631,631):
        row = data1[i:i+631]
        row1 =row[0:-1:2]
        row2 = [[i,i,i] for i in row1]
        Array.append(row2)
        
    Image = np.array(Array, dtype=np.uint8)
    return Image

def image_614881A(data):
    Array = []
    data1 = [int(data[i]) for i in range(len(data))]
    for i in range(0,614881-1281,1281):
        row = data1[i:i+1281]
        row1 =row[0:-1:2]
        row2 = [[i,i,i] for i in row1]
        Array.append(row2)
        
    Image = np.array(Array, dtype=np.uint8)
    return Image

def image(data, DS):
    if DS=="":
        print("No DS set!")
        pass
    elif DS=="614881A":
        Image = image_614881A(data)
    elif DS=="151441A":
        Image = image_151441A(data)
    elif DS=="37561A":
        Image = image_37561A(data)
    else:
        Image = 0
        
    return Image

TO = 15 # timeout for data read
Bd = 4000000 # serial baudrate

DS = "614881A" # number of bytes to read + version 

if __name__ == "__main__":
    try:
        del Data
    except:
        pass
    USB = make_usb()
    Data = read_data(USB, int(DS[0:-1]))
    Image = image(Data, DS)
    
    plt.imsave("Capture " + time.ctime(), Image)
    plt.imshow(Image)
    barcodes = decode(Image)
    for b in barcodes:
        print(b.data)
    pass
    
    USB.close()
    pass
