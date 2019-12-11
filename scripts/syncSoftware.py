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
BinaryPackage = '/home/robot/catkin_ws/devel/lib/'
DeviceList = []
ThirdPartyPackages = ['raspicam_node']
build=False
class IPObject(object):
    def __init__(self,Name,Address):
        self.Name = Name
        self.Address = Address

def generate_AllIPDeviceList():
    list = []
    list.append(IPObject('MasterModule','10.0.0.4'))
    list.append(IPObject('dgitzdev','10.0.0.190'))
    list.append(IPObject('dgitzrosmaster','10.0.0.111'))
    list.append(IPObject('DriverStation','10.0.0.114'))
    list.append(IPObject('ControlModule1','10.0.0.128'))
    list.append(IPObject('ControlModule2','10.0.0.110'))
    list.append(IPObject('BuildServer1','10.0.0.179'))
    return list
  
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
    if not os.path.exists("/tmp/config/"):
        os.makedirs("/tmp/config/")
    #Determine what the name of the ActiveScenario is:
    f = open(ActiveScenarioFile, "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    print "Active Scenario: " + ActiveScenario
    os.unlink(RootDirectory + "config/configdatabase.db")
    os.unlink(RootDirectory + "config/NodeList.txt")
    os.unlink(RootDirectory + "config/ControlGroup.xml")
    os.unlink(RootDirectory + "config/DeviceFile.xml")
    os.unlink(RootDirectory + "config/JoystickCalibration.xml")
    os.unlink(RootDirectory + "config/MiscConfig.xml")
    os.unlink(RootDirectory + "config/SensorLink.xml")
    os.unlink(RootDirectory + "config/SystemFile.xml")
    os.unlink(RootDirectory + "config/TopicMap.xml")
    os.unlink(RootDirectory + "config/SnapshotConfig.xml")
    if(os.path.isdir(RootDirectory + "config/scriptfiles") == True):
        shutil.rmtree(RootDirectory + "config/scriptfiles")
    os.mkdir(RootDirectory + "config/scriptfiles")
    os.system("cp -rf " + RootDirectory + "config/scenarios/" + ActiveScenario + "/scriptfiles/* " + RootDirectory + "/config/scriptfiles/")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/configdatabase.db",RootDirectory + "config/configdatabase.db")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/NodeList.txt",RootDirectory + "config/NodeList.txt")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/ControlGroup.xml",RootDirectory + "config/ControlGroup.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/DeviceFile.xml",RootDirectory + "config/DeviceFile.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/JoystickCalibration.xml",RootDirectory + "config/JoystickCalibration.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/MiscConfig.xml",RootDirectory + "config/MiscConfig.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SensorLink.xml",RootDirectory + "config/SensorLink.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SystemFile.xml",RootDirectory + "config/SystemFile.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/TopicMap.xml",RootDirectory + "config/TopicMap.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SnapshotConfig.xml",RootDirectory + "config/SnapshotConfig.xml")

    
    #Remove old launch files
    if(os.path.isdir(ApplicationPackage + "launch") == True):
        shutil.rmtree(ApplicationPackage + "launch")
    os.mkdir(ApplicationPackage + "launch")

    #Generate launch files
    [alwayson_nodecount,running_nodecount] = generate_launch(ActiveScenario,hostname)
    if((alwayson_nodecount < 0) or (running_nodecount < 0)):
        return 
    #Copy contents of /home/robot/config/scenarios/<ActiveScenario>/* to /home/robot/catkin_ws/src/icarus_rover_v2/launch/
    #shutil.copyfile(RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/" + hostname + ".launch","/tmp/config/" + hostname + ".launch")
   
    iplist = generate_AllIPDeviceList()
    out_file = open("/tmp/hosts", "w")
    out_file.write("127.0.0.1       localhost\n")
    out_file.write("127.0.0.1       " + hostname + "\n")
    out_file.write("::1     ip6-localhost ip6-loopback\n")
    out_file.write("fe00::0 ip6-localnet\n")
    out_file.write("ff00::0 ip6-mcastprefix\n")
    out_file.write("ff02::1 ip6-allnodes\n")
    out_file.write("ff02::2 ip6-allrouters\n")
    for dev in iplist:
        out_file.write(dev.Address + "\t" + dev.Name + "\n")
    out_file.close()
    subprocess.call("sudo cp /tmp/hosts /etc" ,shell=True)
    AutoLaunchList = Helpers.ReadDeviceList('AutoLaunch')
    for f in AutoLaunchList:
        update_launch = False
        sourcelaunch = ''
        if(f.Parent == hostname):
            if(f.PartNumber == '110012'):
                update_launch = True
                sourcelaunch = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Pose/auto_launch/imu_node.launch"
            elif(f.PartNumber == '810090'):
                update_launch = True
                sourcelaunch = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Pose/auto_launch/truth_node.launch"
            else:
                print "Found: " + f.Name + " to build autolaunch but PN: " + f.PartNumber + " is Not Supported"
        if(update_launch == True):
            
            shutil.copyfile(sourcelaunch,"/tmp/config/tmp.launch")
            outputlines = []
            newlaunchfilelines = []
            with open("/tmp/config/tmp.launch") as fd:
                lines = fd.readlines()
            for l in lines:
                if l.find("xml") >= 0:
                    a = 1
                elif l.find("launch") >= 0:
                    a = 1
                elif l.find("<DeviceName>") >= 0:
                    l = l.replace("<DeviceName>",f.Name)
                    outputlines.append(l)
                elif l.find("<DeviceID>") >= 0:
                    l = l.replace("<DeviceID>",str(f.ID))
                    outputlines.append(l)
                elif l.find("<PartNumber>") >= 0:
                    l = l.replace("<PartNumber>",f.PartNumber)
                    outputlines.append(l)
                else:
                    outputlines.append(l)
            with open("/tmp/config/" + hostname + ".launch") as fd:
                lines = fd.readlines()
            for l in lines:
                if l.find("</launch>") >= 0:
                    for g in outputlines:
                        newlaunchfilelines.append(str(g))
                    
                    newlaunchfilelines.append(l)
                else:
                    newlaunchfilelines.append(l)
            out_file = open("/tmp/config/" + hostname + ".launch", "w")
            for l in newlaunchfilelines:
                out_file.write(str(l))
                
            out_file.close()
    
    shutil.copyfile("/tmp/config/" + hostname + ".launch",ApplicationPackage + "launch/" + hostname + ".launch")
    shutil.copyfile("/tmp/config/" + hostname + "_AlwaysOn.launch",ApplicationPackage + "launch/" + hostname + "_AlwaysOn.launch")
    #Sync CMakeLists.txt to the correct device
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/CMakeLists/" + hostname + ".txt"
    dest_file = ApplicationPackage + "CMakeLists.txt"
    shutil.copy(source_file,dest_file)

    #Sync package.xml to the correct device
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/package/" + hostname + ".xml"
    dest_file = ApplicationPackage + "package.xml"
    shutil.copy(source_file,dest_file)

    os.system("cp -rf " + RootDirectory + "config/scenarios/" + ActiveScenario + "/models/* " + RootDirectory + "/catkin_ws/src/icarus_sim/models/")
    os.system("cp -rf " + RootDirectory + "config/scenarios/" + ActiveScenario + "/worlds/* " + RootDirectory + "/catkin_ws/src/icarus_sim/worlds/")
    os.system("cp -rf " + RootDirectory + "config/scenarios/" + ActiveScenario + "/models/* " + RootDirectory + "/.gazebo/models/")

    #Other application launch files

    #Other Packages
    os.system("cp -r ~/other_packages/json/include/nlohmann " + RootDirectory + "catkin_ws/devel/include/")

def test(device):
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    #sshProcess.stdin.write("export TERM=linux\n")
    sshProcess.stdin.write("cd ~/catkin_ws\n")
    sshProcess.stdin.write("source devel/setup.bash\n")
    sshProcess.stdin.write("catkin build\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
def sync_buildserver(device,build):
    myhost = socket.gethostname() 
    mydeviceinfo = Helpers.GetDeviceInfo(myhost)
    targetinfo = Helpers.GetDeviceInfo(device)
    f = open(ActiveScenarioFile, "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    print "Active Scenario: " + ActiveScenario
    print "Syncing Build Server: " + device
    os.unlink(RootDirectory + "config/configdatabase.db")
    os.unlink(RootDirectory + "config/NodeList.txt")
    os.unlink(RootDirectory + "config/ControlGroup.xml")
    os.unlink(RootDirectory + "config/DeviceFile.xml")
    os.unlink(RootDirectory + "config/JoystickCalibration.xml")
    os.unlink(RootDirectory + "config/MiscConfig.xml")
    os.unlink(RootDirectory + "config/SensorLink.xml")
    os.unlink(RootDirectory + "config/SystemFile.xml")
    os.unlink(RootDirectory + "config/TopicMap.xml")
    os.unlink(RootDirectory + "config/SnapshotConfig.xml")

    
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/configdatabase.db",RootDirectory + "config/configdatabase.db")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/NodeList.txt",RootDirectory + "config/NodeList.txt")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/ControlGroup.xml",RootDirectory + "config/ControlGroup.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/DeviceFile.xml",RootDirectory + "config/DeviceFile.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/JoystickCalibration.xml",RootDirectory + "config/JoystickCalibration.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/MiscConfig.xml",RootDirectory + "config/MiscConfig.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SensorLink.xml",RootDirectory + "config/SensorLink.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SystemFile.xml",RootDirectory + "config/SystemFile.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/TopicMap.xml",RootDirectory + "config/TopicMap.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SnapshotConfig.xml",RootDirectory + "config/SnapshotConfig.xml")
    
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("rm " + ApplicationPackage + "launch/*\n")
    stdout,stderr = sshProcess.communicate()
    
    sshProcess.stdin.close()
    iplist = generate_AllIPDeviceList()
    out_file = open("/tmp/hosts_" + device, "w")
    out_file.write("127.0.0.1       localhost\n")
    out_file.write("127.0.0.1       " + device + "\n")
    out_file.write("::1     ip6-localhost ip6-loopback\n")
    out_file.write("fe00::0 ip6-localnet\n")
    out_file.write("ff00::0 ip6-mcastprefix\n")
    out_file.write("ff02::1 ip6-allnodes\n")
    out_file.write("ff02::2 ip6-allrouters\n")
    for dev in iplist:
        out_file.write(dev.Address + "\t" + dev.Name + "\n")
    out_file.close()
    subprocess.call("rsync -avrt /tmp/hosts_" + device + " robot@" + device + ":/tmp/" ,shell=True) 
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("cp /tmp/hosts_" + device + " /etc/hosts\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    if not os.path.exists("/tmp/config/"):
        os.makedirs("/tmp/config/")
    [alwayson_nodecount,running_nodecount] = generate_launch(ActiveScenario,device)
    if((alwayson_nodecount < 0) or (running_nodecount < 0)):
        return 
    #subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/* " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    AutoLaunchList = Helpers.ReadDeviceList('AutoLaunch')
    for f in AutoLaunchList:
        update_launch = False
        sourcelaunch = ''
        if(f.Parent == device):
            if(f.PartNumber == '110012'):
                update_launch = True
                sourcelaunch = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Pose/auto_launch/imu_node.launch"
            elif(f.PartNumber == '810090'):
                update_launch = True
                sourcelaunch = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Pose/auto_launch/truth_node.launch"
            else:
                print "Found: " + f.Name + " to build autolaunch but PN: " + f.PartNumber + " is Not Supported"
        if(update_launch == True):
            shutil.copyfile(sourcelaunch,"/tmp/config/tmp.launch")
            outputlines = []
            newlaunchfilelines = []
            with open("/tmp/config/tmp.launch") as fd:
                lines = fd.readlines()
            for l in lines:
                if l.find("xml") >= 0:
                    a = 1
                elif l.find("launch") >= 0:
                    a = 1
                elif l.find("<DeviceName>") >= 0:
                    l = l.replace("<DeviceName>",f.Name)
                    outputlines.append(l)
                elif l.find("<DeviceID>") >= 0:
                    l = l.replace("<DeviceID>",str(f.ID))
                    outputlines.append(l)
                elif l.find("<PartNumber>") >= 0:
                    l = l.replace("<PartNumber>",f.PartNumber)
                    outputlines.append(l)
                else:
                    outputlines.append(l)
            with open("/tmp/config/" + device + ".launch") as fd:
                lines = fd.readlines()
            for l in lines:
                if l.find("</launch>") >= 0:
                    for g in outputlines:
                        newlaunchfilelines.append(str(g))
                    
                    newlaunchfilelines.append(l)
                else:
                    newlaunchfilelines.append(l)
            out_file = open("/tmp/config/" + device + ".launch", "w")
            for l in newlaunchfilelines:
                out_file.write(str(l))
                
            out_file.close()
            
    #shutil.copyfile("/tmp/config/" + hostname + ".launch",ApplicationPackage + "launch/" + hostname + ".launch")
    subprocess.call("rsync -avrt /tmp/config/" + device + ".launch " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -avrt /tmp/config/" + device + "_AlwaysOn.launch " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/configdatabase.db " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/ControlGroup.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/DeviceFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/JoystickCalibration.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/MiscConfig.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/SensorLink.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/SystemFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/TopicMap.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/SnapshotConfig.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 

    subprocess.call("rsync -avrt " + RootDirectory + "config/targets/* " + "robot@" + device + ":" + RootDirectory + "config/targets/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/sensors/* " + "robot@" + device + ":" + RootDirectory + "config/sensors/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/* " + "robot@" + device + ":" + RootDirectory + "config/scenarios/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/urdf/* " + "robot@" + device + ":" + RootDirectory + "config/urdf/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "scripts/* " + "robot@" + device + ":" + RootDirectory + "scripts/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "storage/AUDIO/output/* " + "robot@" + device + ":" + RootDirectory + "storage/AUDIO/output/" ,shell=True) 
    subprocess.call("rsync -avrt " + ActiveScenarioFile + " robot@" + device + ":" + ActiveScenarioFile ,shell=True) 
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/CMakeLists/" + device + ".txt"


    #Sync Other Packages
    exclude = " "
    if(mydeviceinfo.Architecture != targetinfo.Architecture):
        exclude = "--exclude '*.o' --exclude '*.out' --exclude 'CMakeCache.txt' --exclude 'Makefile' --exclude '*.cmake' --exclude 'CMakeFiles/*' " 
    for i in range(0,len(ThirdPartyPackages)):
        subprocess.call("rsync -avrlt " + exclude + RootDirectory + "catkin_ws/src/" + ThirdPartyPackages[i] + "/* robot@" + device + ":" + RootDirectory + "catkin_ws/src/" + ThirdPartyPackages[i] + "/",shell=True)

    if(os.path.isfile(source_file) == True):
        #Sync CMakeLists.txt to the correct device
        subprocess.call("rsync -avrt " + source_file + " robot@" + device + ":" + ApplicationPackage + "CMakeLists.txt" ,shell=True)
        
        #Sync package.xml to the correct device
        source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/package/" + device + ".xml"
        subprocess.call("rsync -avrt " + source_file + " robot@" + device + ":" + ApplicationPackage + "package.xml" ,shell=True)

        #Sync source code and make 
        subprocess.call("rsync -apvrt " + ApplicationPackage + "src/* " + "robot@" + device + ":" + ApplicationPackage + "src/",shell=True)
        subprocess.call("rsync -apvrt " + ApplicationPackage + "src_templates/* " + "robot@" + device + ":" + ApplicationPackage + "src_templates/",shell=True)
        subprocess.call("rsync -avrt " + ApplicationPackage + "util/* " + "robot@" + device + ":" + ApplicationPackage + "util/",shell=True)
        subprocess.call("rsync -avrt " + ApplicationPackage + "msg/* " + "robot@" + device + ":" + ApplicationPackage + "msg/",shell=True)
        subprocess.call("rsync -avrt " + ApplicationPackage + "srv/* " + "robot@" + device + ":" + ApplicationPackage + "srv/",shell=True)
        subprocess.call("rsync -avrlt " + ApplicationPackage + "include/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
        subprocess.call("rsync -avrlt " + RootDirectory + "catkin_ws/src/eROS/* robot@" + device + ":" + RootDirectory + "catkin_ws/src/eROS/",shell=True)
        
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
def sync_remote(device,build):
    myhost = socket.gethostname()   
    mydeviceinfo = Helpers.GetDeviceInfo(myhost)
    targetinfo = Helpers.GetDeviceInfo(device)
    f = open(ActiveScenarioFile, "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    print "Active Scenario: " + ActiveScenario
    print "Syncing Remote: " + device
    os.unlink(RootDirectory + "config/configdatabase.db")
    os.unlink(RootDirectory + "config/NodeList.txt")
    os.unlink(RootDirectory + "config/ControlGroup.xml")
    os.unlink(RootDirectory + "config/DeviceFile.xml")
    os.unlink(RootDirectory + "config/JoystickCalibration.xml")
    os.unlink(RootDirectory + "config/MiscConfig.xml")
    os.unlink(RootDirectory + "config/SensorLink.xml")
    os.unlink(RootDirectory + "config/SystemFile.xml")
    os.unlink(RootDirectory + "config/TopicMap.xml")
    os.unlink(RootDirectory + "config/SnapshotConfig.xml")

    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/configdatabase.db",RootDirectory + "config/configdatabase.db") 
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/NodeList.txt",RootDirectory + "config/NodeList.txt")    
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/ControlGroup.xml",RootDirectory + "config/ControlGroup.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/DeviceFile.xml",RootDirectory + "config/DeviceFile.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/JoystickCalibration.xml",RootDirectory + "config/JoystickCalibration.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/MiscConfig.xml",RootDirectory + "config/MiscConfig.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SensorLink.xml",RootDirectory + "config/SensorLink.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SystemFile.xml",RootDirectory + "config/SystemFile.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/TopicMap.xml",RootDirectory + "config/TopicMap.xml")
    os.symlink(RootDirectory + "config/scenarios/" + ActiveScenario + "/SnapshotConfig.xml",RootDirectory + "config/SnapshotConfig.xml")
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("rm " + ApplicationPackage + "launch/*\n")
    stdout,stderr = sshProcess.communicate()
    
    sshProcess.stdin.close()
    if not os.path.exists("/tmp/config/"):
        os.makedirs("/tmp/config/")
    [alwayson_nodecount,running_nodecount] = generate_launch(ActiveScenario,device)
    if((alwayson_nodecount < 0) or (running_nodecount < 0)):
        return 
    #subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/* " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    iplist = generate_AllIPDeviceList()
    out_file = open("/tmp/hosts_" + device, "w")
    out_file.write("127.0.0.1       localhost\n")
    out_file.write("127.0.0.1       " + device + "\n")
    out_file.write("::1     ip6-localhost ip6-loopback\n")
    out_file.write("fe00::0 ip6-localnet\n")
    out_file.write("ff00::0 ip6-mcastprefix\n")
    out_file.write("ff02::1 ip6-allnodes\n")
    out_file.write("ff02::2 ip6-allrouters\n")
    for dev in iplist:
        out_file.write(dev.Address + "\t" + dev.Name + "\n")
    out_file.close()
    subprocess.call("rsync -avrt /tmp/hosts_" + device + " robot@" + device + ":/tmp/" ,shell=True) 
    sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    sshProcess.stdin.write("cp /tmp/hosts_" + device + " /etc/hosts\n")
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    AutoLaunchList = Helpers.ReadDeviceList('AutoLaunch')
    for f in AutoLaunchList:
        update_launch = False
        sourcelaunch = ''
        if(f.Parent == device):
            if(f.PartNumber == '110012'):
                update_launch = True
                sourcelaunch = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Pose/auto_launch/imu_node.launch"
            elif(f.PartNumber == '810090'):
                update_launch = True
                sourcelaunch = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Pose/auto_launch/truth_node.launch"
            else:
                print "Found: " + f.Name + " to build autolaunch but PN: " + f.PartNumber + " is Not Supported"
        if(update_launch == True):
            shutil.copyfile(sourcelaunch,"/tmp/config/tmp.launch")
            outputlines = []
            newlaunchfilelines = []
            with open("/tmp/config/tmp.launch") as fd:
                lines = fd.readlines()
            for l in lines:
                if l.find("xml") >= 0:
                    a = 1
                elif l.find("launch") >= 0:
                    a = 1
                elif l.find("<DeviceName>") >= 0:
                    l = l.replace("<DeviceName>",f.Name)
                    outputlines.append(l)
                elif l.find("<DeviceID>") >= 0:
                    l = l.replace("<DeviceID>",str(f.ID))
                    outputlines.append(l)
                elif l.find("<PartNumber>") >= 0:
                    l = l.replace("<PartNumber>",f.PartNumber)
                    outputlines.append(l)
                else:
                    outputlines.append(l)
            with open("/tmp/config/" + device + ".launch") as fd:
                lines = fd.readlines()
            for l in lines:
                if l.find("</launch>") >= 0:
                    for g in outputlines:
                        newlaunchfilelines.append(str(g))
                    
                    newlaunchfilelines.append(l)
                else:
                    newlaunchfilelines.append(l)
            out_file = open("/tmp/config/" + device + ".launch", "w")
            for l in newlaunchfilelines:
                out_file.write(str(l))
                
            out_file.close()

    subprocess.call("rsync -avrt /tmp/config/" + device + ".launch " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -avrt /tmp/config/" + device + "_AlwaysOn.launch " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/ControlGroup.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/DeviceFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/JoystickCalibration.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/MiscConfig.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/SensorLink.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/SystemFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/TopicMap.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt --copy-links " + RootDirectory + "config/SnapshotConfig.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/scriptfiles/* " + "robot@" + device + ":" + RootDirectory + "config/scriptfiles/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/targets/* " + "robot@" + device + ":" + RootDirectory + "config/targets/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/sensors/* " + "robot@" + device + ":" + RootDirectory + "config/sensors/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/* " + "robot@" + device + ":" + RootDirectory + "config/scenarios/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/urdf/* " + "robot@" + device + ":" + RootDirectory + "config/urdf/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "scripts/* " + "robot@" + device + ":" + RootDirectory + "scripts/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "storage/AUDIO/output/* " + "robot@" + device + ":" + RootDirectory + "storage/AUDIO/output/" ,shell=True) 
    subprocess.call("rsync -avrt " + ActiveScenarioFile + " robot@" + device + ":" + ActiveScenarioFile ,shell=True) 
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/CMakeLists/" + targetinfo.DeviceType + ".txt"
    if(os.path.isfile(source_file) == True):
        #Sync CMakeLists.txt to the correct device
        subprocess.call("rsync -avrt " + source_file + " robot@" + device + ":" + ApplicationPackage + "CMakeLists.txt" ,shell=True)
    else:
        print "No CMakeLists.txt File found for Device: " + device 
    #Sync package.xml to the correct device
    source_file = RootDirectory + "config/scenarios/" + ActiveScenario + "/package/" + device + ".xml"
    subprocess.call("rsync -avrt " + source_file + " robot@" + device + ":" + ApplicationPackage + "package.xml" ,shell=True)

    #Sync source code and make 
    exclude = ""
    if(mydeviceinfo.Architecture != targetinfo.Architecture):
        exclude = "--exclude '*.o' --exclude '*.out' --exclude 'CMakeCache.txt' --exclude 'Makefile' --exclude '*.cmake' --exclude 'CMakeFiles/*' " 
    subprocess.call("rsync -avprt " + exclude + ApplicationPackage + "src/* " + "robot@" + device + ":" + ApplicationPackage + "src/",shell=True)
    subprocess.call("rsync -avprt " + exclude + ApplicationPackage + "src_templates/* " + "robot@" + device + ":" + ApplicationPackage + "src_templates/",shell=True)
    subprocess.call("rsync -avrt " + exclude + ApplicationPackage + "util/* " + "robot@" + device + ":" + ApplicationPackage + "util/",shell=True)
    subprocess.call("rsync -avrt " + exclude + ApplicationPackage + "msg/* " + "robot@" + device + ":" + ApplicationPackage + "msg/",shell=True)
    subprocess.call("rsync -avrt " + exclude + ApplicationPackage + "srv/* " + "robot@" + device + ":" + ApplicationPackage + "srv/",shell=True)
    subprocess.call("rsync -avrlt " + exclude + ApplicationPackage + "include/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
    subprocess.call("rsync -avrlt " + exclude + RootDirectory + "catkin_ws/src/eROS/* robot@" + device + ":" + RootDirectory + "catkin_ws/src/eROS/",shell=True)
    
    
    if(targetinfo.Architecture == 'x86_64'):
        #subprocess.call("rsync -avrlt " + exclude + RootDirectory + "catkin_ws/src/icarus_sim/* robot@" + device + ":" + RootDirectory + "catkin_ws/src/icarus_sim/",shell=True)
        subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/models/* " + "robot@" + device + ":" + RootDirectory + "/.gazebo/models/" ,shell=True) 
        subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/models/* " + "robot@" + device + ":" + RootDirectory + "/catkin_ws/src/icarus_sim/models/" ,shell=True) 
        subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/worlds/* " + "robot@" + device + ":" + RootDirectory + "/catkin_ws/src/icarus_sim/worlds/" ,shell=True) 

    #subprocess.call("rsync -avrlt " + RootDirectory + "catkin_ws/devel/include/" + PackageName + "/* " + "robot@" + device + ":" + RootDirectory + "catkin_ws/devel/include/" + PackageName,shell=True)
    #subprocess.call("rsync -avrt " + RootDirectory + "catkin_ws/devel/include/" + PackageName + "/* " + "robot@" + device + ":" + ApplicationPackage + "include/",shell=True)
    
    if((mydeviceinfo.Architecture == targetinfo.Architecture) and (mydeviceinfo.Architecture != 'x86_64')):
        print "Syncing Binaries"
        subprocess.call("rsync -apvrt " + ApplicationPackage + "package.xml " + "robot@" + device + ":" + ApplicationPackage + "",shell=True)
        subprocess.call("rsync -avt " + BinaryPackage + "*.so " + "robot@" + device + ":" + BinaryPackage + "",shell=True)
        subprocess.call("rsync -avt " + BinaryPackage + "*.a " + "robot@" + device + ":" + BinaryPackage + "",shell=True)
        subprocess.call("ssh robot@" + device + " & mkdir " + BinaryPackage + "icarus_rover_v2",shell=True)
        subprocess.call("rsync -avt " + BinaryPackage + "icarus_rover_v2/* " + "robot@" + device + ":" + BinaryPackage + "icarus_rover_v2/",shell=True)
        for i in range(0,len(ThirdPartyPackages)):
            subprocess.call("rsync -avt " + BinaryPackage + ThirdPartyPackages[i] +"/* " + "robot@" + device + ":" + BinaryPackage + ThirdPartyPackages[i] + "/",shell=True)
    for i in range(0,len(ThirdPartyPackages)):
        subprocess.call("rsync -avt " + RootDirectory + "catkin_ws/src/" + ThirdPartyPackages[i] + "/launch/* " + "robot@" + device + ":" + RootDirectory + "/catkin_ws/src/" + ThirdPartyPackages[i] + "/launch/",shell=True)
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
    
def sync_display(device):
    f = open(ActiveScenarioFile, "r")
    contents = f.readlines()
    f.close()
    ActiveScenario = "".join(contents[0].split())
    print "Active Scenario: " + ActiveScenario
    print "Syncing Display: " + device
    #sshProcess = subprocess.Popen(['ssh',"robot@" + device], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    #sshProcess.stdin.write("rm " + ApplicationPackage + "launch/*\n")
    #stdout,stderr = sshProcess.communicate()
    
    #sshProcess.stdin.close()
    subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/* " + "robot@" + device + ":" + RootDirectory + "config/scenarios/" + ActiveScenario + "/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/launch/* " + "robot@" + device + ":" + ApplicationPackage + "launch/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/DeviceFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/SystemFile.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/MiscConfig.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt " + RootDirectory + "config/TopicMap.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/SnapshotConfig.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/targets/* " + "robot@" + device + ":" + RootDirectory + "config/targets/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/scenarios/" + ActiveScenario + "/sensors/* " + "robot@" + device + ":" + RootDirectory + "config/sensors/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/urdf/* " + "robot@" + device + ":" + RootDirectory + "config/urdf/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "scripts/* " + "robot@" + device + ":" + RootDirectory + "scripts/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "config/ControlGroup.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True)  
    subprocess.call("rsync -avrt " + RootDirectory + "config/TopicMap.xml " + "robot@" + device + ":" + RootDirectory + "config/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "executable/* " + "robot@" + device + ":" + RootDirectory + "executable/" ,shell=True) 
    subprocess.call("rsync -avrt " + RootDirectory + "catkin_ws/src/eROS/include/*.h " + "robot@" + device + ":" + RootDirectory + "catkin_ws/src/eROS/include/",shell=True)
    subprocess.call("rsync -avrt " + RootDirectory + "catkin_ws/src/icarus_rover_v2/include/*.h " + "robot@" + device + ":" + RootDirectory + "catkin_ws/src/icarus_rover_v2/include/",shell=True)
    subprocess.call("rsync -avrt " + RootDirectory + "Dropbox/ICARUS/RoverV2/SOFTWARE/gui/* " + "robot@" + device + ":" + RootDirectory + "Dropbox/ICARUS/RoverV2/SOFTWARE/gui/" ,shell=True) 
    
def sync_all(hostname,build):
    print "Syncing All"
    DeviceList = Helpers.ReadDeviceList('ROS')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        if (DeviceList[i].Name == hostname):
            sync_local(hostname)
        else:
            if((DeviceList[i].Name != "dgitzrosmaster") and
               (DeviceList[i].Name != "dgitzdev")):
                sync_remote(DeviceList[i].Name,build)
    DeviceList = Helpers.ReadDeviceList('Display')
    for i in range(0,len(DeviceList)):
        print DeviceList[i].Name + ":" + DeviceList[i].IPAddress
        sync_display(DeviceList[i].Name)
def generate_launch(scenario,device):
    nodelist_file = RootDirectory + "config/scenarios/" + scenario + "/NodeList.txt"
    running_node_count = 0
    alwayson_node_count = 0
    AlwaysOnSection = 0
    RunningNodeSection = 0
    launch_file = open("/tmp/config/" + device + ".launch","w+")
    launch_file.write("<!--xml-->\r\n")
    launch_file.write("<launch>\r\n")
    launch_file_alwayson = open("/tmp/config/" + device + "_AlwaysOn.launch","w+")
    launch_file_alwayson.write("<!--xml-->\r\n")
    launch_file_alwayson.write("<launch>\r\n")
    with open(nodelist_file) as f:
        for line in f:
            head,sep,tail = line.partition("#")
            head = head.replace(" ","")
            head = head.replace("\n","")
            if(head != ""):
                if "AlwaysOnNodes" in head:
                    AlwaysOnSection = 1
                elif "RunningNodes" in head:
                    RunningNodeSection = 1
                elif "}" in head:
                    if(AlwaysOnSection == 1):
                        AlwaysOnSection = 0
                    elif(RunningNodeSection == 1):
                        RunningNodeSection = 0
                else:
                    target_device = head[head.index("device")+8:-2]
                    if(target_device == device):
                        nodeconfig_file = RootDirectory + "config/scenarios/" + scenario + "/launch/NodeLaunch/" + head[1:head.index("=>")-1] + ".xml"
                        if(RunningNodeSection == 1):
                                    running_node_count = running_node_count + 1
                        elif(AlwaysOnSection == 1):
                                    alwayson_node_count = alwayson_node_count + 1
                        if(os.path.isfile(nodeconfig_file) == False):
                            print "ERROR: Node Config File: " + nodeconfig_file + " Does Not Exist.  Exiting."
                            return [-1,-1]
                        with open(nodeconfig_file) as fd:
                            for readline in fd:
                                if(RunningNodeSection == 1):
                                    launch_file.write(readline)
                                elif(AlwaysOnSection == 1):
                                    launch_file_alwayson.write(readline)
                        fd.close()

    launch_file_alwayson.write("</launch>\r\n")                       
    launch_file.write("</launch>\r\n")
    launch_file.close()
    launch_file_alwayson.close() 
    return [alwayson_node_count,running_node_count]
def main():
    parser = OptionParser("syncSoftware.py [options]")
    parser.add_option("-s","--syncmode",dest="syncmode",default="all",help="all,remote,buildserver,local,display [default: %default]")
    parser.add_option("-b","--build",dest="build",default=False,help="True,False [default: %default]")
    parser.add_option("-d","--device",dest="device",default="",help="DeviceName [default: %default]")
    (opts,args) = parser.parse_args()
    if((opts.syncmode=="remote") and (opts.device=="")):
        print "ERROR: Remote Sync must specify a device."
        return
    if(opts.syncmode == "all"):
        sync_all(socket.gethostname(),opts.build)
    elif (opts.syncmode=="remote"):
        response = os.system("ping -c 1 " + opts.device)
        if response != 0:
            print "ERROR: Remote: " + opts.device + " is Not Reachable."
            return 
        sync_remote(opts.device,opts.build)
    elif (opts.syncmode=="buildserver"):
        response = os.system("ping -c 1 " + opts.device)
        if response != 0:
            print "ERROR: Build Server: " + opts.device + " is Not Reachable."
            return        
        sync_buildserver(opts.device,opts.build)
    elif (opts.syncmode=="display"):
        response = os.system("ping -c 1 " + opts.device)
        if response != 0:
            print "ERROR: Remote Display: " + opts.device + " is Not Reachable."
            return        
        sync_display(opts.device)
    elif (opts.syncmode=="local"):
        sync_local(socket.gethostname())   

if __name__ == "__main__":
    main()
