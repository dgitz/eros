#!/usr/bin/env python
from enum import Enum
import subprocess
import pdb
class ArchitectureType(Enum):
    UNKNOWN = 0
    X86_64 =1 
    ARMV7L = 2
    AARCH64 = 3
    END_OF_LIST= 4

class Architecture():

    __initialized = False
    def __init__(self):
        self.__initialized = True
    def is_initialized(self):
        return self.__initialized
    def pretty(self):
        pass
    def get_architecture(self,hostname,user=''):
        arch1 = self.architecture_lookup_cmd1(hostname,user)
        arch = arch1
        return  arch
    def architecture_lookup_cmd1(self,hostname,user=''):
        cmd1 = "uname -m"
        if(hostname == "localhost"):
            cmd_res = subprocess.check_output(cmd1,shell=True)
            arch = self.convert_architecture(cmd_res)
            if(arch == ArchitectureType.UNKNOWN):
                print "ERROR: Architecture not able to process with command: " + cmd1 + " and response: " + cmd_res
            return arch
        else:
            try:
                cmd = "ssh " + user + "@" + hostname + " 2>/dev/null " + cmd1
                cmd_res = subprocess.check_output(cmd,shell=True)
                arch = self.convert_architecture(cmd_res)
                if(arch == ArchitectureType.UNKNOWN):
                    print "ERROR: Architecture not able to process with command: " + cmd + " and response: " + cmd_res
                return arch
            except:
                print "ERROR: Not able to connect to Host: " + hostname + " with user: " + user
                return ArchitectureType.UNKNOWN
    def convert_architecture(self,arch_str):
        if('x86_64' in arch_str):
            return ArchitectureType.X86_64
        elif('armv7l' in arch_str):
            return ArchitectureType.ARMV7L
        elif('aarch64' in arch_str):
            return ArchitectureType.AARCH64
        else:
            return ArchitectureType.UNKNOWN


    