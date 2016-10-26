import sys,getopt,os
import Helpers
import psutil
import pdb
ActiveTaskFile = '/home/robot/config/ActiveTasks'
DeviceList = []

def print_usage():
    print "Usage Instructions"
    print "Using Active Tasks File: " + ActiveTaskFile
    print "No Options: Stop all Devices"
    print "-? This Menu"
    print "-a Stop all Devices"
    print "-r <device> Stop on remote device"
    print "-l Stop local Device"

def stop_device_remote():
    print "Stopping Remote"

def stop_device_local():
    print "Stopping Local"
    read_tasklist()

def stop_all_devices():
    print "Stop All Devices"
    Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress

def read_tasklist():
    ProcessList = []
    f = open(ActiveTaskFile, "r")
    contents = f.readlines()
    f.close()
    for i in range(0, len(contents)):
        #print contents[i]
        items = contents[i].split('\t')
        for j in range(0,len(items)):
            if (items[j] == "Task:"):
                kill_process(items[j+1])
   

def kill_process(process):
    for proc in psutil.process_iter():
        if proc.name == process:
            print "Killing process: " + process + " with PID: " + str(proc.pid)
            proc.kill()

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?arl",["help"])
    if(len(opts) == 0):
        stop_all_devices()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-a':
            stop_all_devices()
        elif opt == '-r':
            stop_device_remote()
        elif opt == '-l':
            stop_device_local()

if __name__ == "__main__":
    main()
