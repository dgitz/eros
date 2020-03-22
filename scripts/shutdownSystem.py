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
#from colored import fg, bg, attr
DeviceList = []
class IPObject(object):
    def __init__(self,Name,Address):
        self.Name = Name
        self.Address = Address

def generate_IgnoreDeviceList():
    list = []
    list.append('dgitzdev')
    list.append('dgitzrosmaster')
    list.append('DriverStation')
    list.append('BuildServer1')
    list.append('MasterModule')

    return list
def print_usage():
    print "Usage Instructions: shutdownSystem."
    print "No Options: This Menu."
    print "-?/-h This Menu."
    print "-s/-a Shutdown all Devices."
    print "-r Reboot all devices."

def shutdown_all_devices():
    ignorelist = generate_IgnoreDeviceList()
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        if(DeviceList[i].Name == socket.gethostname()):
            print "Not shutting down my host, but restarting tasks: " + socket.gethostname()
        else:
            ignore = False
            for j in range(0,len(ignorelist)):
                if(DeviceList[i].Name == ignorelist[j]):
                    ignore = True
            if(ignore == True):
                print "Ignoring: " + DeviceList[i].Name
            else:
                print "Shutting down Device: " + DeviceList[i].Name
                sshProcess = subprocess.Popen(['ssh',"robot@" + DeviceList[i].Name], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
                sshProcess.stdin.write("sudo shutdown -h now\n")
                stdout,stderr = sshProcess.communicate()
                sshProcess.stdin.close()
    
            

def reboot_all_devices():
    ignorelist = generate_IgnoreDeviceList()
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        if(DeviceList[i].Name == socket.gethostname()):
            print "Not rebooting my host: " + socket.gethostname()
        else:
            ignore = False
            for j in range(0,len(ignorelist)):
                if(DeviceList[i].Name == ignorelist[j]):
                    ignore = True
            if(ignore == True):
                print "Ignoring: " + DeviceList[i].Name
            else:
                print "Rebooting Device: " + DeviceList[i].Name
                sshProcess = subprocess.Popen(['ssh',"robot@" + DeviceList[i].Name], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
                sshProcess.stdin.write("sudo shutdown -r now\n")
                stdout,stderr = sshProcess.communicate()
                sshProcess.stdin.close()
    

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?hars",["help"])
    if(len(opts) == 0):
        print_usage()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-h':
            print_usage()
        elif opt == '-s':
            shutdown_all_devices()
        elif opt == '-a':
            shutdown_all_devices()
        elif opt == '-r':
            reboot_all_devices()
        else:
            print_usage()

if __name__ == "__main__":
    main()
