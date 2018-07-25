import os.path
import sys
from datetime import datetime
import pdb
def print_usage():
    print "Usage Instructions"
    print "Generate eROS_Definitions.py from header: python generate_erosdefinitions.py -p <Directory>, where Directory is the location where eROS_Definitions.h exists"
    print "Generate eROS_Definitions.m from header: python generate_erosdefinitions.py -m <Directory>, where Directory is the location where eROS_Definitions.h exists"

def generate_file_python(directory):
    header_file = directory + "/eROS_Definitions.h"
    if(os.path.isfile(header_file) == False):
        print "<Directory> should have eROS_Definitions.h"
        sys.exit(0)
    output_path = directory + "/eROS_Definitions.py"
    output_file = open(output_path,'w+')
    output_file.write('#/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    output_file.write('#/***Created on:')
    output_file.write( str(datetime.now()))
    output_file.write('***/\r\n')
    lines = tuple(open(header_file, 'r'))
    for line in lines:
        if(("define" in line) and ("__EROS_DEFINITIONS_INCLUDED__" not in line)):
            items = line.split()
            if(items[0] == "#define"):
                output_file.write(items[1] + ' = ' + items[2] + '\r\n')
        elif(line.startswith("//")):
            output_file.write('\r\n' + line.replace("//","#"))
            
def generate_file_matlab(directory):
    header_file = directory + "/eROS_Definitions.h"
    if(os.path.isfile(header_file) == False):
        print "<Directory> should have eROS_Definitions.h"
        sys.exit(0)
    output_path = directory + "/eROS_Definitions.m"
    output_file = open(output_path,'w+')
    output_file.write('%***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    output_file.write('%***Created on:')
    output_file.write('%' + str(datetime.now()))
    output_file.write('%***/\r\n')
    lines = tuple(open(header_file, 'r'))
    in_tag = 0
    tag_name = "";
    for line in lines:
        if("//TAG: Start" in line):
            tag_name = line[12:].strip()
            output_file.write("global " + tag_name + "\r\n")
            in_tag = 1
        elif("//TAG: End" in line):
            output_file.write("\r\n\r\n")
            in_tag = 0
        elif(in_tag == 1):
            items = line.split()
            if(len(items) >= 3):
                output_file.write(tag_name + "." + items[1] + " = " + items[2] + ";\r\n")
                    
                    
                
if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-p"):
    generate_file_python(sys.argv[2])
elif (sys.argv[1] == "-m"):
    generate_file_matlab(sys.argv[2])
