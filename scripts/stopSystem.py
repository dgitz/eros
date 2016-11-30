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
    

def stop_device_local(level):
    print "Stopping Local"
    read_nodelist(level)

def stop_all_devices(level):
    print "Stop All Devices"
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        if(DeviceList[i].Name == socket.gethostname()):
            stop_device_local(level)
        else:
            stop_device_remote(DeviceList[i].Name)
        

def read_nodelist(level):
    ProcessList = []
    f = open(ActiveNodesFile, "r")
    contents = f.readlines()
    f.close()
    for i in range(0, len(contents)):
        items = contents[i].split('\t')
        for j in range(0,len(items)):
            if (items[j] == "Node:"):
                hostname = socket.gethostname()
                if("ComputeModule" in hostname):
                    kill_process2(items[j+1],level)
                else:
                    kill_process(items[j+1],level) 
   
def kill_process2(process,level):
    print process
    for proc in psutil.process_iter():
        if proc.name in process:
            try:
                with suppress_stdout():
                    subprocess.call("kill " + str(level) + " " + str(proc.pid),shell=True)
            except:
                print "Oops"

def kill_process(process,level):
    for proc in psutil.process_iter():
       
        try:
            found_process=0
            for i in range(0, len(proc.cmdline())):
                if(proc.cmdline()[i].find(process) >= 0):
                    found_process = 1
            if(found_process == 1):
                print "Killing process: " + process + " with PID: " + str(proc.pid) 
                try: 
                    subprocess.call("kill " + str(level) + " " + str(proc.pid),shell=True)
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
            stop_all_devices(-2)
            stop_all_devices(-2)
        elif opt == '-r':
            stop_device_remote(arg)
        elif opt == '-l':
            stop_device_local(-2)
            stop_device_local(-2)

if __name__ == "__main__":
    main()
