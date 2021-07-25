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

import pdb
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
    def manual_sync(self,username,target,folder_type_list,folder_list,clean,dryrun,verbose):
        arch_detector = Architecture()
        host_arch = arch_detector.get_architecture('localhost')
        target_arch = arch_detector.get_architecture(target,username)
        drop_binaries = False
        if(arch_detector.check_architecture_compatability(host_arch,target_arch) == False): 
            print COLOR_YELLOW + "WARN: Host Arch: " + Architecture.convert_ArchStr(host_arch) + " Is not Directly Compatible Target Arch: " + Architecture.convert_ArchStr(target_arch) + COLOR_END
            drop_binaries = True
        return self.__sync(username,target,folder_type_list,folder_list,drop_binaries,clean,dryrun,verbose)
    def __sync(self,username,target,folder_type_list,folder_list,drop_binaries,clean,dryrun,verbose):
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