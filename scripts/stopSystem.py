#!/usr/bin/python
import sys,getopt,os
from contextlib import contextmanager
import Helpers
import psutil
import pdb
import subprocess
import socket
ActiveNodesFile = '/home/robot/config/ActiveNodes'
DeviceList = []
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr

def print_usage():
    print "Usage Instructions"
    print "Using Active Nodes File: " + ActiveNodesFile
    print "No Options: Stop all Devices"
    print "-? This Menu"
    print "-a Stop all Devices"
    print "-r <device> Stop on remote device"
    print "-l Stop local Device"

def stop_device_remote(device):
    print "Stopping Remote: " + device
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("cd ~\n")
    sshProcess.stdin.write("python scripts/stopSystem.py -l\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    print stderr
    print stdout
    

def stop_device_local():
    print "Stopping Local"
    read_nodelist()

def stop_all_devices():
    print "Stop All Devices"
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        if(DeviceList[i].Name == socket.gethostname()):
            stop_device_local()
        else:
            stop_device_remote(DeviceList[i].Name)
        

def read_nodelist():
    ProcessList = []
    f = open(ActiveNodesFile, "r")
    contents = f.readlines()
    f.close()
    for i in range(0, len(contents)):
        #print contents[i]
        items = contents[i].split('\t')
        for j in range(0,len(items)):
            #print items[j]
            if (items[j] == "Node:"):
                kill_process(items[j+1])
   

def kill_process(process):
    for proc in psutil.process_iter():
        try:
            found_process=0
            #print len(proc.cmdline())
            for i in range(0, len(proc.cmdline())):
                if(proc.cmdline()[i].find(process) >= 0):
                    found_process = 1
            if(found_process == 1):
                print "Killing process: " + process + " with PID: " + str(proc.pid) 
                try: 
                    with suppress_stdout():
                        output = proc.kill()
                except:
                    print "Oops"   
        except:
            a=1

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?ar:l",["help"])
    if(len(opts) == 0):
        stop_all_devices()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-a':
            stop_all_devices()
        elif opt == '-r':
            stop_device_remote(arg)
        elif opt == '-l':
            stop_device_local()

if __name__ == "__main__":
    main()
