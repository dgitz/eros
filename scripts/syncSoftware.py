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
RootDirectory = '/home/robot/'
PackageName = "icarus_rover_v2"
ActiveTaskFile = RootDirectory + 'config/ActiveTasks'
ActiveScenarioFile = RootDirectory + 'config/ActiveScenario'
ApplicationPackage = '/home/robot/catkin_ws/src/' + PackageName + '/'
DeviceList = []
build=False

def process_input(tempstr):
    print tempstr


def change_package(newpackage):
    PackageName = newpackage
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

    #for f in glob.glob(ApplicationPackage + "include/*"):
    #    shutil.copy(f,RootDirectory + "/catkin_ws/devel/include/" + PackageName) 


def test(device):
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    #sshProcess.stdin.write("export TERM=linux\n")
    sshProcess.stdin.write("cd ~/catkin_ws\n")
    sshProcess.stdin.write("source devel/setup.bash\n")
    sshProcess.stdin.write("catkin build\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
def sync_remote(device,build):
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
    subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/* " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/DeviceFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt " + RootDirectory + "config/TopicMap.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/targets/* " + "robot@" + device + ":" + RootDirectory + "config/targets/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/sensors/* " + "robot@" + device + ":" + RootDirectory + "config/sensors/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/urdf/* " + "robot@" + device + ":" + RootDirectory + "config/urdf/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "scripts/* " + "robot@" + device + ":" + RootDirectory + "scripts/" ,shell=True) 

    
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/CMakeLists/" + device + ".txt"
    if(os.path.isfile(source_file) == True):
        #Sync CMakeLists.txt to the correct device
        subprocess.call("rsync -avrt " + source_file + " robot@" + device + ":" + ApplicationPackage + "CMakeLists.txt" ,shell=True)
        
        #Sync package.xml to the correct device
        source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/package/" + device + ".xml"
        subprocess.call("rsync -avrt " + source_file + " robot@" + device + ":" + ApplicationPackage + "package.xml" ,shell=True)

        #Sync source code and make 
        subprocess.call("rsync -apvrt " + ApplicationPackage + "src/* " + "robot@" + device + ":" + ApplicationPackage + "src/",shell=True)
        subprocess.call("rsync -avrt " + ApplicationPackage + "util/* " + "robot@" + device + ":" + ApplicationPackage + "util/",shell=True)
        subprocess.call("rsync -avrt " + ApplicationPackage + "msg/* " + "robot@" + device + ":" + ApplicationPackage + "msg/",shell=True)
        subprocess.call("rsync -avrlt " + ApplicationPackage + "include/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
        subprocess.call("rsync -avrlt " + RootDirectory + "catkin_ws/src/eROS/include/* robot@" + device + ":" + RootDirectory + "catkin_ws/src/eROS/include/",shell=True)
        #subprocess.call("rsync -avrlt " + RootDirectory + "catkin_ws/devel/include/" + PackageName + "/* " + "robot@" + device + ":" + RootDirectory + "catkin_ws/devel/include/" + PackageName,shell=True)
        #subprocess.call("rsync -avrt " + RootDirectory + "catkin_ws/devel/include/" + PackageName + "/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
        
        sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
        #sshProcess.stdin.write("export TERM=linux\n")
        sshProcess.stdin.write("cd ~/catkin_ws\n")
        sshProcess.stdin.write("source devel/setup.bash\n")
        if(build == "True"):
            sshProcess.stdin.write("catkin build\n")
        stdout,stderr = sshProcess.communicate()
        sshProcess.stdin.close()
        print stderr
        print stdout
    else:
        print "No CMakeLists.txt File found for Device: " + device 



def sync_all(hostname,build):
    print "Syncing All"
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        if (DeviceList[i].Name == hostname):
            sync_local(hostname)
        else:
            sync_remote(DeviceList[i].Name,build)

def main():
    parser = OptionParser("syncSoftware.py [options]")
    parser.add_option("--syncmode",dest="syncmode",default="all",help="all,remote,local [default: %default]")
    parser.add_option("--build",dest="build",default=False,help="True,False [default: %default]")
    parser.add_option("--device",dest="device",default="",help="DeviceName [default: %default]")
    (opts,args) = parser.parse_args()
    if((opts.syncmode=="remote") and (opts.device=="")):
        print "ERROR: Remote Sync must specify a device."
        return
    if(opts.syncmode == "all"):
        sync_all(socket.gethostname(),opts.build)
    elif (opts.syncmode=="remote"):
        response = os.system("ping -c 1 " + opts.device)
        if response != 0:
            print "ERROR: Remote Device" + opts.device + " is Not Reachable."
            return        
        sync_remote(opts.device,opts.build)
    elif (opts.syncmode=="local"):
        sync_local(socket.gethostname())   

if __name__ == "__main__":
    main()
