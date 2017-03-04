import os.path
import sys
from datetime import datetime
import pdb
def print_usage():
    print "Usage Instructions"
    print "Generate eROS_Definitions.py from header: ./generate_erosdefinitions.py -g <Directory>, where Directory is the location where eROS_Definitions.h exists"
def generate_file(directory):
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
if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-g"):
    generate_file(sys.argv[2])
