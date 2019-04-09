#!/usr/bin/python2
# -*- coding: utf-8 -*-
__Author__ = 'csd'
__Copyright__ = 'Copyrigh 2019, PI'
__version__ = "1.0"
__data__ = "2019.04.09"

import os
import sys
import numpy as np
import hid
import ctypes

CMD_HEAD =       [0x00, 0x40, 0x00, 0x03, 0x00, 0x39, 0x1a, 0x01] # load image from flash by index
LED_RED =        [0x00, 0x40, 0x00, 0x03, 0x00, 0x07, 0x1a, 0x01] # R
LED_GREEN =      [0x00, 0x40, 0x00, 0x03, 0x00, 0x07, 0x1a, 0x02] # G
LED_BLUE =       [0x00, 0x40, 0x00, 0x03, 0x00, 0x07, 0x1a, 0x04] # B
LED_RB =         [0x00, 0x40, 0x00, 0x03, 0x00, 0x07, 0x1a, 0x05] # RB

PWM_INVERT =     [0x00, 0x40, 0x00, 0x03, 0x00, 0x05, 0x1a, 0x00]# invert
PWM_NOT_INVERT = [0x00, 0x40, 0x00, 0x03, 0x00, 0x05, 0x1a, 0x01]# not invert

POWER_STANDBY =  [0x00, 0x40, 0x00, 0x03, 0x00, 0x00, 0x02, 0x01]
VIDEO_MODE =     [0x00, 0x40, 0x00, 0x03, 0x00, 0x00, 0x02, 0x00]
LED_CURRENT =    [0x00, 0x40, 0x00, 0x05, 0x00, 0x01, 0x0b, 0xc8, 0x78, 0x7d]

class LEDStruct(ctypes.Structure):
    _fields_ = [
        ("red", ctypes.c_ubyte, 1),
        ("green", ctypes.c_ubyte, 1),
        ("blue", ctypes.c_ubyte, 1),
    ]

def listHidDevice():
    for d in hid.enumerate():
        keys = list(d.keys())
        keys.sort()
    for key in keys:
        print("%s : %s" % (key, d[key]))


class DLP4500API():
    CMD_HEAD = np.array([0x00, 0x40, 0x00, 0x03, 0x00])
    CMD_FUNC_CODE = {
        "LOAD_IMAGE": np.array([0x39, 0x1a]),
        "LED_SELECT": np.array([0x07, 0x1a]),
        "CURRENT_MODE": np.array([0x05, 0x1a]),
        "CURRENT_VALUE": np.array([0x01, 0x0b]),
        "OPERATION_MODE": np.array([0x00, 0x02]),
    }
    USB_READ_MAX_SIZE = 64

    def __init__(self, vendor_id, product_id):
        # self.write_command = []
        # self.led_selection = LEDStruct()
        self.hid_dev = hid.device()
        self.openDevice(vendor_id, product_id)

    def openDevice(self, vendor_id, product_id):
        try:
            self.hid_dev.open(vendor_id, product_id)
            print("Manufacturer: %s" % self.hid_dev.get_manufacturer_string())
            print("Product: %s" % self.hid_dev.get_product_string())
            print("Serial No: %s" % self.hid_dev.get_serial_number_string())
            print("Write the data")
        except IOError as e:
            print(e)
            print("You probably don't have the hard coded device. Update the hid.device line")
            print("in this script with one from the enumeration list output above and try again.")

    def hidWriteCmd(self, cmd):
        self.hid_dev.write(cmd)

    def hidReadCmd(self):
        read_data = self.hid_dev.read(self.USB_READ_MAX_SIZE)
        return read_data

    def preSendCmd(self, cmd1, cmd2):
        pre_command = self.CMD_HEAD.copy()
        pre_command = np.append(pre_command, cmd1)
        pre_command = np.append(pre_command, cmd2)
        return pre_command

    def loadImageFromFlash(self, imageIndex):
        write_cmd = self.preSendCmd(self.CMD_FUNC_CODE["LOAD_IMAGE"], imageIndex)
        self.hid_dev.write(write_cmd)

    def ledSelection(self, ledRed=1, ledGreen=0, ledBlue=0):
        led_value = ledRed * 1 + ledGreen * 2 + ledBlue * 4
        write_cmd = self.preSendCmd(self.CMD_FUNC_CODE["LED_SELECT"], led_value)
        self.hid_dev.write(write_cmd)

    def ledCurrentValue(self, redCurrent=40, greenCurrent=0, blueCurrent=0):
        current_3x1 = np.array([redCurrent, greenCurrent, blueCurrent])
        write_cmd = self.preSendCmd(self.CMD_FUNC_CODE["CURRENT_VALUE"], current_3x1)
        self.hid_dev.write(write_cmd)

    def ledCurrentMode(self, currentMode=1):
        write_cmd = self.preSendCmd(self.CMD_FUNC_CODE["CURRENT_MODE"], currentMode)
        self.hid_dev.write(write_cmd)

    def setOperationMode(self, operationMode=1):
        write_cmd = self.preSendCmd(self.CMD_FUNC_CODE["OPERATION_MODE"], operationMode)
        self.hid_dev.write(write_cmd)

def main():
    led_selection = LEDStruct()
    led_selection.red = 1
    led_selection.green = 1
    led_selection.blue = 1


    print(led_selection)

if __name__ == "__main__":
    main()
