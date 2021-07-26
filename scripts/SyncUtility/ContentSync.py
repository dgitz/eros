import sys
import os
import subprocess
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../../include/eros/")
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../ArchitectureUtility/")
from eROS_Definitions import *
from ContentSyncDefinition import ContentSyncType
from ContentSyncDefinition import FolderType
from ArchitectureType import ArchitectureType
from Architecture import Architecture
import xml.etree.ElementTree as ET

import pdb
class Folder(object):
    def __init__(self,Name='',Type='',Path=''):
        self.Name = Name
        self.Type = Type
        self.Directory = Path
class Device(object):
    def __init__(self,Name='',User='',ChildDevices=[]):
        self.Name = Name
        self.User = User
        self.ChildDevices = ChildDevices

class ContentSync():
    __initialized = False
    def __init__(self):
        self.__initialized = True
    @staticmethod
    def convert_ContentMode(content_mode_str):
        if(content_mode_str == "manual"):
            return ContentSyncType.MANUAL
        elif(content_mode_str == "auto"):
            return ContentSyncType.AUTOMATIC
        else:
            print(COLOR_RED + "ERROR Content Mode : " + content_mode_str + " Not supported." + COLOR_END)
            return ContentSyncType.UNKNOWN
    @staticmethod
    def convert_FolderType(folder_type_str):
        if(folder_type_str == "generic"):
            return FolderType.GENERIC
        elif(folder_type_str == "config"):
            return FolderType.CONFIG
        elif(folder_type_str == "source"):
            return FolderType.SOURCE
        elif(folder_type_str == "binary"):
            return FolderType.BINARY
        else:
            return FolderType.GENERIC
    def sync(self,devices,folders,clean,dryrun,verbose):
        arch_detector = Architecture()
        host_arch = arch_detector.get_architecture('localhost')
        for device in devices:
            target = device.Name
            username = device.User    
            target_arch = arch_detector.get_architecture(target,username)
            drop_binaries = False
            if(arch_detector.check_architecture_compatability(host_arch,target_arch) == False): 
                drop_binaries = True
            folder_list = []
            for f in folders:
                folder_list.append(f.Directory)
            if(self.__transfer(username,target,folder_list,drop_binaries,clean,dryrun,verbose) == False):
                print COLOR_YELLOW + "WARN: Sync Failed to Target: " + target + COLOR_END
    def read_config_cli(self,folder_list,username,target):
        FolderList = []
        DeviceTreeList = []
        for f in folder_list:
            newFolder = Folder(f,'generic',f)
            FolderList.append(newFolder)
        newDevice = Device(target,username)
        DeviceTreeList.append(newDevice)
        return FolderList,DeviceTreeList
    def read_config_file(self,file_path):
        FolderList = []
        DeviceTreeList = []
        tree = ET.parse(file_path)
        root = tree.getroot()
        for List in root:
            for item in List:
                if(List.tag == 'SyncDevices'):
                    newDevice = Device(item.attrib["name"])
                    for child_item in item:
                        childDevice = Device(child_item.attrib["user"],child_item.attrib["name"])
                        newDevice.ChildDevices.append(childDevice)
                    DeviceTreeList.append(newDevice)
                elif(List.tag == 'Folders'):
                    newFolder = Folder(item.attrib["name"],item.attrib["type"],item.attrib["directory"])
                    FolderList.append(newFolder)
        return FolderList,DeviceTreeList
    def __transfer(self,username,target,folder_list,drop_binaries,clean,dryrun,verbose):
        if(dryrun):
            print COLOR_YELLOW + "WARN: Enabling Dry-Run Mode!!!" + COLOR_END
        else:
            if(clean):
                print COLOR_YELLOW + "WARN: Cleaning Target Folders!" + COLOR_END
                for folder in folder_list:
                    cmd = "ssh " + username + "@" + target + " \ rm -r -f " + folder# + " 2>/dev/null "
                    subprocess.check_output(cmd,shell=True)
  
        for folder in folder_list:
            cmd = "rsync -iart"
            if(dryrun):
                cmd += "n"
            if((verbose == VerbosityLevel.VERYVERBOSE)):
                cmd += "v"
            else:
                if(dryrun == False):
                    cmd += "q"
            cmd += " --exclude *git/* "
            if(drop_binaries == True):
                cmd += " --exclude */CMakeFiles/* "
                cmd += " --exclude /CMakeFiles/* "
                cmd += " --exclude */catkin_generated/* "
                cmd += " --exclude /catkin_generated/* "
                cmd += " --exclude *.make "
                cmd += " --exclude *.cmake "
                cmd += " --exclude *.o "
                cmd += " --exclude *.so "
                cmd += " --exclude Makefile "
                cmd += " --exclude *context.py "


            cmd += " " + folder + " " + username + "@" + target + ":" + folder + "| grep '^<' | awk '{ print $2 }'\n"
            subprocess.call(cmd,shell=True)
            print COLOR_GREEN + " Sync'd Folder: " + folder + COLOR_END
        return True