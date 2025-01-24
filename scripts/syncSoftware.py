# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb
from util.Helpers import *

# Defines
CRED = '\33[31m'
CYELLOW = '\33[33m'
CGREEN = '\33[32m'
CBLUE = '\33[34m'
CEND = '\033[0m'

def sync_remote_to_remote(syncconfig_file,origin,remote):
    print(CGREEN + "Sync From: " + origin + " To: " + remote + " Started..." + CEND)
    folders = Helpers.ReadSyncConfig(syncconfig_file)
    sshProcess = subprocess.Popen(['ssh',"robot@" + origin], stdin=subprocess.PIPE, stdout = subprocess.PIPE, universal_newlines=True,bufsize=0) 
    for folder in folders:
        if ((folder.Type == 'Binary') or (folder.Type == 'Config')):
            print("Syncing: " + folder.Name + " to Remote: " + remote)
            tempstr = "rsync -iart " + folder.Directory + "* robot@" + remote + ":" + folder.Directory + "| grep '^<' | awk '{ print $2 }'\n"
            sshProcess.stdin.write(tempstr)
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    print(CGREEN + "Sync From: " + origin + " To: " + remote + " Finished." + CEND)

def sync_remote(syncconfig_file,device_name):
    print(CGREEN + "Sync Started to: " + device_name + CEND)
    syncconfig_file = "/home/davidgitz/config/SyncConfig.xml" # Fix this
    folders = ReadSyncConfig(syncconfig_file)
    for folder in folders:
        if((folder.Type == 'Source') or (folder.Type == 'Config')):            
            subprocess.call("rsync -iart " + folder.Directory + "/* robot@" + device_name + ":" + folder.Directory + "| grep '^<' | awk '{ print $2 }'",shell=True)
    print(CGREEN + "Sync Completed to: " + device_name  + CEND)
    

def sync_buildserver(devicelist_file,syncconfig_file,device_name,build):
    print(CGREEN + "Sync Started to: " + device_name + CEND)
    folders = Helpers.ReadSyncConfig(syncconfig_file)
    devices = Helpers.ReadDeviceList(devicelist_file,'ROS')
    myDevice = [] 
    found = False
    for device in devices:
        if device.Name == device_name:
            found = True
            myDevice = device
    if(found == False):
        print(CRED + "Could NOT Find Device in DeviceList." + CEND)
        return
    for folder in folders:
        if((folder.Type == 'Source') or (folder.Type == 'Config')):
            sync_this = False
            for arch in folder.Architectures:
                if arch == myDevice.Architecture:
                    sync_this = True
            if(sync_this == True):
                subprocess.call("rsync -iart " + folder.Directory + "/* robot@" + device_name + ":" + folder.Directory + "| grep '^<' | awk '{ print $2 }'",shell=True)
            else:
                print(CYELLOW + "Architecture Mismatch, Not Syncing " + folder.Type + " Folder: " + folder.Directory + CEND)
    print(CGREEN + "Sync Completed to: " + device_name  + CEND)
    
    build_attempted = False
    if(build == True):
        if(len(devices) == 0):
            print(CRED + "NO DEVICES FOUND!" + CEND)
        for device in devices:
            if(device.Name == device_name):
                build_attempted = True
                print(CGREEN + "Building on Device: " + device.Name + "..." + CEND)
                tempstr = "ssh robot@" + device.Name + " \"cd " + device.CatkinWS + "; source devel/setup.bash; catkin_make"
                if(device.Jobs < 0):
                    tempstr = tempstr + " > /dev/null\""
                else:
                    tempstr = tempstr + " -j" + str(device.Jobs) +" > /dev/null\""
                ret = subprocess.call(tempstr,shell=True)
                if(ret != 0): 
                    print(CRED + "BUILD FAILED ON TARGET!!!" + CEND)
                    return
                else:
                    print(CGREEN + "Build Completed on: " + device.Name + CEND)
        if(build_attempted == False):
            print(CYELLOW + "Build never attempted.  Check your configuration." + CEND)
    
        


# Main
def main():
    parser = OptionParser("syncSoftware.py [options]")
    parser.add_option("-s","--syncmode",dest="syncmode",default="all",help="all,buildserver,buildserver_target, [default: %default]")
    parser.add_option("-b","--build",dest="build",default=0,help="1,0 [default: %default]",type="int")
    parser.add_option("-d","--devices",dest="devices",default="",help="DeviceName1,DeviceName2,... [default: %default]")
    parser.add_option("-c","--config_dir",dest="config_dir",default="~/config/",help="Location where Config dir should be found. default=~/config")
    (opts,args) = parser.parse_args()
    if(opts.syncmode == "all"):
        print(CRED + "WIP!!!" + CEND)
        devices = opts.devices.split(",")
        remotes = devices
        for device in remotes:
            sync_remote(opts.config_dir + "SyncConfig.xml",device)
        print(CGREEN + 'Remote Sync Finished' + CEND)
    elif (opts.syncmode=="buildserver"):
        devices = opts.devices.split(",")
        if(len(devices) < 1):
            print(CRED,"No Devices Specified.")
            return
        for device in devices:
            response = os.system("ping -c 1 " + device)
            if response != 0:
                print("ERROR: Build Server: " + device + " is Not Reachable.")
                return        
            sync_buildserver(opts.config_dir + "DeviceList.json",opts.config_dir + "SyncArtifactConfig.xml",device,opts.build)
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
        sync_buildserver(opts.config_dir + "DeviceList.json",opts.config_dir + "SyncArtifactConfig.xml",build_server,opts.build)

        for device in remotes:
            sync_remote_to_remote(opts.config_dir + "SyncConfig.xml",build_server,device)
        print(CGREEN + 'Remote Sync Finished' + CEND)
    else:
        print(CRED + 'Sync Mode: ' + opts.syncmode + ' Not Supported.' + CEND)

if __name__ == "__main__":
    main()
