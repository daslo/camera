#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 22 17:03:48 2019

@author: ds
"""
import serial
TO = 1 # timeout for data read
Bd = 4000000 # serial baudrate

DS = "614881A" # number of bytes to read + version 

def make_usb():
    try:
        usb.close()
    except:
        pass
    
    try:
        usb=serial.Serial('/dev/ttyUSB0', baudrate=Bd, timeout=TO)
    except:
        usb=serial.Serial('/dev/ttyUSB2', baudrate=Bd, timeout=TO)
    
    usb.flush()
    usb.reset_input_buffer()
    usb.reset_output_buffer()
    return usb

if __name__ == "__main__":
    USB = make_usb()
    USB.write(b"2")
    Data = USB.read(1)
    print(chr(Data[0]))
    USB.close()
    pass
