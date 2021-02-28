# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb

# Defines
CRED = '\33[31m'
CYELLOW = '\33[33m'
CGREEN = '\33[32m'
CBLUE = '\33[34m'
CEND = '\033[0m'
# Class Objects
class FolderToSync(object):
    def __init__(self,Name,Type,Path):
        self.Name = Name
        self.Type = Type
        self.Path = Path
class Device(object):
    def __init__(self,Name,Architecture,Catkin_Dir):
        self.Name = Name
        self.Architecture = Architecture
        self.Catkin_Dir = Catkin_Dir
# Functions
def generate_FolderSync():
    list = []
    list.append(FolderToSync('eROS','Source','/home/robot/catkin_ws/src/eROS/'))
    list.append(FolderToSync('eROS_config','Config','/home/robot/catkin_ws/src/eROS/launch/'))
    list.append(FolderToSync('main_config','Config','/home/robot/config/'))
    list.append(FolderToSync('main_devel','Binary','/home/robot/catkin_ws/devel/'))
    list.append(FolderToSync('main_build','Binary','/home/robot/catkin_ws/build/'))
    return list

def generate_Devices():
    list = []
    list.append(Device('BuildServer1','armv7l','/home/robot/catkin_ws'))
    list.append(Device('GPUModule1','aarch64','/home/robot/catkin_ws'))
    return list

def sync_remote_to_remote(origin,remote):
    print(CGREEN + "Sync From: " + origin + " To: " + remote + " Started..." + CEND)
    folders = generate_FolderSync()
    sshProcess = subprocess.Popen(['ssh',"robot@" + origin], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    for folder in folders:
        if ((folder.Type == 'Binary') or (folder.Type == 'Config')):
            print("Syncing: " + folder.Name + " to Remote: " + remote)
            tempstr = "rsync -avrt " + folder.Path + "* robot@" + remote + ":" + folder.Path + "\n"
            sshProcess.stdin.write(tempstr)
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    print(CGREEN + "Sync From: " + origin + " To: " + remote + " Finished." + CEND)

def sync_buildserver(device_name,build):
    print(CGREEN + "Sync Started to: " + device_name + CEND)
    folders = generate_FolderSync()
    for folder in folders:
        if(folder.Type == 'Source'):
            subprocess.call("rsync -avrt " + folder.Path + "/* robot@" + device_name + ":" + folder.Path,shell=True)
    print(CGREEN + "Sync Completed to: " + device_name  + CEND)
    devices = generate_Devices()
    if(build == True):
        for device in devices:
            if(device.Name == device_name):
                print(CGREEN + "Building on Device: " + device.Name + "..." + CEND)
                tempstr = "ssh robot@" + device.Name + " \"cd " + device.Catkin_Dir + "; source devel/setup.bash; catkin_make > /dev/null\""
                ret = subprocess.call(tempstr,shell=True)
                if(ret != 0): 
                    print(CRED + "BUILD FAILED ON TARGET!!!" + CEND)
                    return
                else:
                    print(CGREEN + "Build Completed on: " + device.Name + CEND)
    
        


# Main
def main():
    parser = OptionParser("syncSoftware.py [options]")
    parser.add_option("-s","--syncmode",dest="syncmode",default="all",help="all,remote,buildserver,buildserver_target, [default: %default]")
    parser.add_option("-b","--build",dest="build",default=False,help="True,False [default: %default]",type="int")
    parser.add_option("-d","--devices",dest="devices",default="",help="DeviceName1,DeviceName2,... [default: %default]")
    (opts,args) = parser.parse_args()
    if (opts.syncmode=="buildserver"):
        devices = opts.devices.split(",")
        if(len(devices) < 1):
            print(CRED,"No Devices Specified.")
            return
        for device in devices:
            response = os.system("ping -c 1 " + device)
            if response != 0:
                print("ERROR: Build Server: " + device + " is Not Reachable.")
                return        
            sync_buildserver(device,opts.build)
    elif(opts.syncmode == "buildserver_target"):
        devices = opts.devices.split(",")
        if(len(devices) < 2):
            print(CRED,"Not enough Devices Specified.")
            return
        build_server = devices[0]
        remotes = devices[1:]
        response = os.system("ping -c 1 " + build_server)
        if response != 0:
            print("ERROR: Build Server: " +build_server + " is Not Reachable.")
            return        
        sync_buildserver(build_server,opts.build)

        for device in remotes:
            sync_remote_to_remote(build_server,device)
        print(CGREEN + 'Remote Sync Finished' + CEND)

if __name__ == "__main__":
    main()