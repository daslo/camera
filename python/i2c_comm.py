#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 22 22:03:22 2019

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
    # set_reg, reg, read, set_reg, reg, set_val, val, write, set_reg, reg, read
    for i in [b'8']:
        USB.write(i)
        Data = USB.read(1)
        print(chr(Data[0]))
    
    USB.close()
    pass
