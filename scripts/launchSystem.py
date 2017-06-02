#!/usr/bin/python
import sys,getopt,os
import Helpers
import psutil
import pdb
import subprocess
import socket
from subprocess import call
#ActiveTaskFile = '/home/robot/config/ActiveTasks'
DeviceList = []
def launch_roscore():
    print "Launching ROSCore"
    call("killall -9 roscore",shell=True,stdout=subprocess.PIPE)
    call("killall -9 rosmaster",shell=True,stdout=subprocess.PIPE)
    call("roscore &",shell=True,stdout=subprocess.PIPE)

def print_usage():
    print "Usage Instructions"

def launch_device_remote(device):
    print "Starting Remote: " + device
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("cd ~\n")
    sshProcess.stdin.write("python scripts/launchSystem.py -l\n")
    sshProcess.stdin.close()
    

def launch_device_local(hostname):
    print "Launching Local"
    subprocess.call(['/bin/bash','-c',"./scripts/launchDevice -d " + hostname + " > /dev/null & disown"])

def launch_all_devices(hostname):
    print "Launching All Devices"
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        if (DeviceList[i].Name == hostname):
            launch_device_local(hostname)
        else:
            launch_device_remote(DeviceList[i].Name)
        


def main():
    opts, args = getopt.getopt(sys.argv[1:],"?ar:l",["help"])
    if(len(opts) == 0):
        launch_roscore()
        launch_all_devices(socket.gethostname())
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-a':
            launch_roscore()
            launch_all_devices(socket.gethostname())
        elif opt == '-r':
            launch_device_remote(arg)
        elif opt == '-l':
            launch_device_local(socket.gethostname())

if __name__ == "__main__":
    main()
