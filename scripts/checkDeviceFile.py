#!/usr/bin/python
from __future__ import unicode_literals
import sys,getopt,os
import Helpers
import psutil
import glob,os,shutil
import socket
import subprocess
import pdb
import os.path
from optparse import OptionParser
import time
import sys
from functools import partial
DeviceList = []
SupportedCapabilityList = ['ROS','Display','GPIO','Power','Network','Sensor']

def print_usage():
    print "Usage Instructions"
    print "No Options: This Menu"
    print "-? This Menu"
    print "-a Check all Devices"

def check_all_devices():
    passed = Helpers.checkDeviceFileFormat()
    if(passed == False):
        print "ERROR: DeviceFile.xml formatted incorrectly. Exiting."
        return 0
    Empty_Capability_DeviceList = Helpers.ReadDeviceList('')
    if(len(Empty_Capability_DeviceList) > 0):
        print "WARN: All these Devices must have at least 1 Capability defined:"
        for i in range(0,len(Empty_Capability_DeviceList)):
            print Empty_Capability_DeviceList[i].Name
        passed = False
        return 0
    capabilities = Helpers.ReadCapabilityList()
    for i in range(0,len(capabilities)):
        found = False
        for j in range(0,len(SupportedCapabilityList)):
            if(SupportedCapabilityList[j] == capabilities[i]):
                found = True
        if(found == False):
            print "WARN: Capability: " + capabilities[i] + " Not Supported"
            passed = False
    for i in range(0,len(SupportedCapabilityList)):
        Devices = Helpers.ReadDeviceList(SupportedCapabilityList[i])
        print "---------------------"
        print "Capability: " + SupportedCapabilityList[i]
        for j in range(0,len(Devices)):
            print Devices[j].Name
    if(passed == True):
        print "All checks passed."

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?a",["help"])
    if(len(opts) == 0):
        print_usage()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-a':
            check_all_devices()
        else:
            print_usage()

if __name__ == "__main__":
    main()