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

import time
import sys
from functools import partial
RootDirectory = '/home/robot/'
PackageName = "icarus_rover_v2"
ActiveTaskFile = RootDirectory + 'config/ActiveTasks'
ActiveScenarioFile = RootDirectory + 'config/ActiveScenario'
ApplicationPackage = '/home/robot/catkin_ws/src/' + PackageName + '/'
DeviceList = []
build=False

def print_usage():
    print "Usage Instructions"
    print "Using Active Scenario File: " + ActiveScenarioFile
    print "using Application Package: " + ApplicationPackage
    print "No Options: Sync all Devices"
    print "-? Usage Instructions (this menu)"
    print "-a Sync to all Devices"
    #print "-c <Package> Change Package from default: " +  PackageName
    #print "-p Print Current Package"
    print "-r <device> Sync to remote device"
    print "-l Sync to local Device"

def process_input(tempstr):
    print tempstr


def change_package(newpackage):
    PackageName = newpackage
    print_usage()
    command = raw_input('Now What?')
    process_input(command)
    

def print_package():
    print PackageName

def sync_local(hostname):
    print "Syncing Local: " + hostname
    #Determine what the name of the ActiveScenario is:
    f = open(ActiveScenarioFile, "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    print "Active Scenario: " + ActiveScenario

    #Remove old launch files
    shutil.rmtree(ApplicationPackage + "/launch")

    #Copy contents of /home/robot/config/scenarios/<ActiveScenario>/* to /home/robot/catkin_ws/src/icarus_rover_v2/launch/
    shutil.copytree(RootDirectory + "config/scenarios/" + ActiveScenario + "/launch",ApplicationPackage + "launch")
    
    #Sync CMakeLists.txt to the correct device
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/CMakeLists/" + hostname + ".txt"
    dest_file = ApplicationPackage + "CMakeLists.txt"
    shutil.copy(source_file,dest_file)

    #Sync package.xml to the correct device
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/package/" + hostname + ".xml"
    dest_file = ApplicationPackage + "package.xml"
    shutil.copy(source_file,dest_file)

    #Sync other config items
    for f in glob.glob(RootDirectory + "config/urdf/*"):
        shutil.copy(f,ApplicationPackage + "urdf/")

    for f in glob.glob(ApplicationPackage + "include/*"):
        shutil.copy(f,RootDirectory + "/catkin_ws/devel/include/" + PackageName) 


def test(device):
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    #sshProcess.stdin.write("export TERM=linux\n")
    sshProcess.stdin.write("cd ~/catkin_ws\n")
    sshProcess.stdin.write("source devel/setup.bash\n")
    sshProcess.stdin.write("catkin build\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
def sync_remote(device):
    f = open(ActiveScenarioFile, "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    print "Active Scenario: " + ActiveScenario
    print "Syncing Remote: " + device
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("rm " + ApplicationPackage + "launch/*\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    subprocess.call("rsync -avr " + RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/* " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -ar " + RootDirectory + "config/DeviceFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -ar " + RootDirectory + "config/TopicMap.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -ar " + RootDirectory + "config/targets/* " + "robot@" + device + ":" + RootDirectory + "config/targets/" ,shell=True) 
    subprocess.call("rsync -ar " + RootDirectory + "config/urdf/* " + "robot@" + device + ":" + RootDirectory + "config/urdf/" ,shell=True) 
    subprocess.call("rsync -ar " + RootDirectory + "scripts/* " + "robot@" + device + ":" + RootDirectory + "scripts/" ,shell=True) 

    
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/CMakeLists/" + device + ".txt"
    if(os.path.isfile(source_file) == True):
        #Sync CMakeLists.txt to the correct device
        subprocess.call("rsync -ar " + source_file + " robot@" + device + ":" + ApplicationPackage + "CMakeLists.txt" ,shell=True)
        
        #Sync package.xml to the correct device
        source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/package/" + device + ".xml"
        subprocess.call("rsync -ar " + source_file + " robot@" + device + ":" + ApplicationPackage + "package.xml" ,shell=True)

        #Sync source code and make 
        subprocess.call("rsync -ar " + ApplicationPackage + "src/* " + "robot@" + device + ":" + ApplicationPackage + "src/",shell=True)
        subprocess.call("rsync -ar " + ApplicationPackage + "util/* " + "robot@" + device + ":" + ApplicationPackage + "util/",shell=True)
        subprocess.call("rsync -ar " + ApplicationPackage + "msg/* " + "robot@" + device + ":" + ApplicationPackage + "msg/",shell=True)
        subprocess.call("rsync -ar " + ApplicationPackage + "include/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
        subprocess.call("rsync -ar " + RootDirectory + "catkin_ws/devel/include/" + PackageName + "/* " + "robot@" + device + ":" + RootDirectory + "catkin_ws/devel/include/" + PackageName,shell=True)
        subprocess.call("rsync -ar " + RootDirectory + "catkin_ws/devel/include/" + PackageName + "/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
        
        sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
        #sshProcess.stdin.write("export TERM=linux\n")
        sshProcess.stdin.write("cd ~/catkin_ws\n")
        sshProcess.stdin.write("source devel/setup.bash\n")
        sshProcess.stdin.write("catkin build\n")
        stdout,stderr = sshProcess.communicate()
        sshProcess.stdin.close()
        print stderr
        print stdout
    else:
        print "No CMakeLists.txt File found for Device: " + device 



def sync_all(hostname):
    print "Syncing All"
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        if (DeviceList[i].Name == hostname):
            sync_local(hostname)
        else:
            sync_remote(DeviceList[i].Name)

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?ac:pr:l",["help"])
    if(len(opts) == 0):
        sync_all(socket.gethostname())
    for opt,arg in opts:
        if opt == '-b':
            build=True
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-a':
            sync_all(socket.gethostname())
        elif opt == '-c':
            change_package(arg)
        elif opt == '-r':
            sync_remote(arg)
            #test(arg)
        elif opt == '-l':
            sync_local(socket.gethostname())
        elif opt == '-p':
            print_package()

if __name__ == "__main__":
    main()
