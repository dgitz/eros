#!/usr/bin/python
import sys
import pdb
import os 
from os import walk
import xml.etree.ElementTree as ET
def check_all():
    unittest_dir="/home/robot/catkin_ws/build/test_results/icarus_rover_v2"
    f = []
    all_ok = True
    failed_list = []
    passed_list = []
    unittest_count = 0
    for (dirpath, dirnames, filenames) in walk(unittest_dir):
        f.extend(filenames)
        for file in f:
            unittest_count = unittest_count + 1
            if(check_unittestfile(dirpath + "/" + file) == False):
                print "Test:" + file + " FAILED."
                all_ok = False
                failed_list.append(dirpath + "/" + file)
            else:
                passed_list.append(dirpath + "/" + file)
        break
    if(unittest_count == 0):
        print "NO UNIT TESTS RAN"
        all_ok = False
    return [all_ok,failed_list,passed_list];
def check_unittestfile(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    if((int(root.attrib['failures']) > 0) or (int(root.attrib['errors']) > 0)):
        return False
    else:
        return True

