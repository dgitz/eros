# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb
import Helpers

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
            tempstr = "rsync -avrt " + folder.Directory + "* robot@" + remote + ":" + folder.Directory + "\n"
            sshProcess.stdin.write(tempstr)
    stdout,stderr = sshProcess.communicate()
    sshProcess.stdin.close()
    print(CGREEN + "Sync From: " + origin + " To: " + remote + " Finished." + CEND)

def sync_buildserver(devicelist_file,syncconfig_file,device_name,build):
    print(CGREEN + "Sync Started to: " + device_name + CEND)
    folders = Helpers.ReadSyncConfig(syncconfig_file)
    for folder in folders:
        if((folder.Type == 'Source') or (folder.Type == 'Config')):
            subprocess.call("rsync -avrt " + folder.Directory + "/* robot@" + device_name + ":" + folder.Directory,shell=True)
    print(CGREEN + "Sync Completed to: " + device_name  + CEND)
    devices = Helpers.ReadDeviceList(devicelist_file,'ROS')
    if(build == True):
        for device in devices:
            if(device.Name == device_name):
                print(CGREEN + "Building on Device: " + device.Name + "..." + CEND)
                tempstr = "ssh robot@" + device.Name + " \"cd " + device.CatkinWS + "; source devel/setup.bash; catkin_make"
                if(device.Jobs == 0):
                    tempstr = tempstr + " > /dev/null\""
                else:
                    tempstr = tempstr + " -j" + str(device.Jobs) +" > /dev/null\""
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
    parser.add_option("-b","--build",dest="build",default=0,help="1,0 [default: %default]",type="int")
    parser.add_option("-d","--devices",dest="devices",default="",help="DeviceName1,DeviceName2,... [default: %default]")
    parser.add_option("-c","--config_dir",dest="config_dir",default="/home/robot/config/",help="Location where Config dir should be found. default=/home/robot/config")
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
            sync_buildserver(opts.config_dir + "DeviceList.xml",opts.config_dir + "SyncConfig.xml",device,opts.build)
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
        sync_buildserver(opts.config_dir + "DeviceList.xml",opts.config_dir + "SyncConfig.xml",build_server,opts.build)

        for device in remotes:
            sync_remote_to_remote(opts.config_dir + "SyncConfig.xml",build_server,device)
        print(CGREEN + 'Remote Sync Finished' + CEND)

if __name__ == "__main__":
    main()