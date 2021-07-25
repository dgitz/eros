import sys
import os
import subprocess
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../../include/eros/")
from eROS_Definitions import *
from ContentSyncDefinition import ContentSyncType
from ContentSyncDefinition import FolderType
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
    @staticmethod
    def sync(username,target,folder_type_list,folder_list,clean,dryrun,verbose):
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
                cmd += "q"
            cmd += " " + folder + " " + username + "@" + target + ":" + folder + "| grep '^<' | awk '{ print $2 }'\n"
            subprocess.call(cmd,shell=True)
            if(verbose == VerbosityLevel.VERBOSE):
                print COLOR_GREEN + " Sync'd Folder: " + folder + COLOR_END
        return True