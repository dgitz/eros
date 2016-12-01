#!/usr/bin/python
import sys,getopt,os
from contextlib import contextmanager
import Helpers
import psutil
import pdb
import subprocess
import time
ActiveNodesFile = '/home/robot/config/ActiveNodes'
TemplateHeaderFile = '/home/robot/catkin_ws/src/icarus_rover_v2/src_templates/sample_node.h'
TemplateCppFile = '/home/robot/catkin_ws/src/icarus_rover_v2/src_templates/sample_node.cpp'
CodeDirectory = '/home/robot/catkin_ws/src/icarus_rover_v2/src/'

def print_usage():
    print "Usage Instructions"
    print "Using Template Header File: \r\n" + TemplateHeaderFile + "\r\n\r\nUsing Template Cpp File: \r\n" + TemplateCppFile + "\r\n\r\nUpdating all Nodes in: \r\n" + CodeDirectory + "\r\n"
    print "No Options: Update all Source Code"
    print "-? This Menu"
    print "-a Update all Source Code"

def update_all():
    for dirName,subdirList,fileList in os.walk(CodeDirectory):
        for fname in fileList:
            if(("node" in fname) and ("~" not in fname) and ("process" not in fname) and ("orig" not in fname)):
                if(".h" in fname):
                    subprocess.call("meld " + TemplateHeaderFile + " " + dirName + "/" + fname,shell=True)
                elif(".cpp" in fname):
                    subprocess.call("meld " + TemplateCppFile + " " +  dirName + "/" + fname,shell=True)
                time.sleep(0.25)
    

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?a",["help"])
    if(len(opts) == 0):
        update_all()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-a':
            update_all()

if __name__ == "__main__":
    main()
