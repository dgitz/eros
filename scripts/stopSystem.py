#!/usr/bin/python
import sys,getopt,os
from contextlib import contextmanager
import Helpers
import psutil
import pdb
import subprocess
import socket
import os
import os.path
from time import sleep
from subprocess import call
RootDirectory = '/home/robot/'
PackageName = "icarus_rover_v2"
ActiveTaskFile = RootDirectory + 'config/ActiveTasks'
ActiveScenarioFile = RootDirectory + 'config/ActiveScenario'
ApplicationPackage = '/home/robot/catkin_ws/src/' + PackageName + '/'
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
    print "Usage Instructions: stopSystem."
    #print "Using Active Nodes File: " + ActiveNodesFile + "."
    print "No Options: This Menu."
    print "-?/-h This Menu."
    print "-s Stop all Devices, but leaves Always On Nodes Running."
    print "-d <DeviceName> Stop this Device, but leaves Always On Nodes Running."
    print "-k Stop all Devices, including Always On Nodes."


    
def kill_all_devices():
    print "Kill All Devices"
    os.system("rosnode kill --all")
    os.system("rosnode cleanup")
def safestop_all_devices():
    print "Stop All Devices"
    DeviceList = Helpers.ReadDeviceList('ROS')
    #Determine what Nodes should always be running, don't stop these.
    f = open(RootDirectory + 'config/ActiveScenario', "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    nodelist_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/NodeList.txt"
    AlwaysOnSection = 0
    nodes_to_keep = []
    with open(nodelist_file) as f:
        for line in f:
            head,sep,tail = line.partition("#")
            head = head.replace(" ","")
            head = head.replace("\n","")
            if(head != ""):
                if "AlwaysOnNodes" in head:
                    AlwaysOnSection = 1
                elif "}" in head:
                    if(AlwaysOnSection == 1):
                        AlwaysOnSection = 0
                elif (AlwaysOnSection == 1):

                    target_device = head[head.index("device")+8:-2]
                    for device in DeviceList:
                        if(target_device == device.Name):
                            nodeconfig_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/NodeLaunch/" + head[1:head.index("=>")-1] + ".xml"
                            if(os.path.isfile(nodeconfig_file) == False):
                                print "ERROR: Node Config File: " + nodeconfig_file + " Does Not Exist.  Exiting."
                                return [-1,-1]
                            with open(nodeconfig_file) as fd:
                                for readline in fd:
                                    if("node name=" in readline):
                                        nodes_to_keep.append("/" + device.Name + readline[readline.index('node name="$(env ROS_HOSTNAME)_')+30:readline.index("pkg")-2])          
                                #    if(RunningNodeSection == 1):
                                #        launch_file.write(readline)
                                #    elif(AlwaysOnSection == 1):
                                #        launch_file_alwayson.write(readline)
                                fd.close() 
    v = subprocess.Popen(['rosnode','list'], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0)
    out,error = v.communicate()
    nodelist = out.decode('utf-8').split('\n')
    for node in nodelist:
        kill_node = True
        empty = False
        if(node == ""):
            empty = True
            kill_node = False
        elif(node == "/rosout"):
            kill_node = False
        else:
            for keep in nodes_to_keep:
                if(keep == node):
                    kill_node = False
        if(kill_node == True):
            print "Killing: " + node
            os.system("rosnode kill " + node)
        elif (empty == False):
            print "Keeping: " + node
    os.system("rosnode cleanup")
def safestop_device(device):
    print "Stopping: " + device
    DeviceList = Helpers.ReadDeviceList('ROS')
    #Determine what Nodes should always be running, don't stop these.
    f = open(RootDirectory + 'config/ActiveScenario', "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    nodelist_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/NodeList.txt"
    AlwaysOnSection = 0
    nodes_to_keep = []
    with open(nodelist_file) as f:
        for line in f:
            head,sep,tail = line.partition("#")
            head = head.replace(" ","")
            head = head.replace("\n","")
            if(head != ""):
                if "AlwaysOnNodes" in head:
                    AlwaysOnSection = 1
                elif "}" in head:
                    if(AlwaysOnSection == 1):
                        AlwaysOnSection = 0
                elif (AlwaysOnSection == 1):

                    target_device = head[head.index("device")+8:-2]
                    if(target_device == device):
                        nodeconfig_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/NodeLaunch/" + head[1:head.index("=>")-1] + ".xml"
                        if(os.path.isfile(nodeconfig_file) == False):
                            print "ERROR: Node Config File: " + nodeconfig_file + " Does Not Exist.  Exiting."
                            return [-1,-1]
                        with open(nodeconfig_file) as fd:
                            for readline in fd:
                                if("node name=" in readline):
                                    nodes_to_keep.append("/" + device + readline[readline.index('node name="$(env ROS_HOSTNAME)_')+30:readline.index("pkg")-2])          
                            fd.close() 

    v = subprocess.Popen(['rosnode','list'], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0)
    out,error = v.communicate()
    nodelist = out.decode('utf-8').split('\n')
    for node in nodelist:
        kill_node = True
        empty = False
        if(node == ""):
            empty = True
            kill_node = False
        elif(node == "/rosout"):
            kill_node = False
        elif(device not in node):
            kill_node = False
        else:
            for keep in nodes_to_keep:
                if(keep == node):
                    kill_node = False
        if(kill_node == True):
            print "Killing: " + node
            os.system("rosnode kill " + node)
        elif (empty == False):
            print "Keeping: " + node
    os.system("rosnode cleanup")
    
        

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?hskd:",["help"])
    if(len(opts) == 0):
        print_usage()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-h':
            print_usage()
        elif opt == '-s':
            safestop_all_devices()
        elif opt == '-k':
            kill_all_devices()
        elif opt == '-d':
            safestop_device(arg)
        else:
            print_usage()

if __name__ == "__main__":
    main()
