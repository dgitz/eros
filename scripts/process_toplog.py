#Author: David Gitz, gitz_david@cat.com
#Purpose: Reads in a series of top logs and converts CPU usage of every process into one csv file
import sys
import time
import pdb
class Process(object):
    def __init__(self,PID,Name,RAM_kB=None,CPU_Perc=None):
        self.PID = PID
        self.Name = Name
        self.RAM_kB = []
        self.CPU_Perc = []

def print_usage():
    print "Usage Instructions"
    print "Process Log: -p <directory containing top_log.txt file"

def process_log(top_log,out_dir):
    line_counter = 0
    new_entry = 0
    entry_counter = 0
    ProcessList = []
    with open(top_log) as f:
        PID_Column = -1
        RSS_Column = -1
        CPU_Column = -1
        COMMAND_Column = -1
        initialized = 0
        for line in f:
            line_counter = line_counter + 1
            print "Reading Line: ",line_counter
            items = line.split()
            if '<' in items:
                items.remove('<')
            if(initialized == 0):
                for i in range(len(items)):
                    if(items[i] == "PID"):
                        PID_Column = i
                    elif(items[i] == "RSS"):
                        RSS_Column = i
                    elif(items[i] == "COMMAND"):
                        COMMAND_Column = i
                    elif(items[i] == "%CPU"):
                        CPU_Column = i
                if((COMMAND_Column > -1) and (PID_Column > -1)):
                    initialized = 1
                if((initialized == 0) and (line_counter > 100)):
                    print "Couldn't get column names.  Exiting."
                    sys.exit(0)             
            else:
                if(len(items) > 0):
                    if(items[0] == "PID"):
                        entry_counter = entry_counter +1
                    elif(items[0].isdigit()):
                        #print "Process entry"
                        if ((items[COMMAND_Column].startswith("ps") == 0) and (items[COMMAND_Column].startswith("sh") == 0) and (items[COMMAND_Column].startswith("grep") == 0) and (items[COMMAND_Column].startswith("tail") == 0) and (items[COMMAND_Column].startswith("top") == 0)):
                        #if (("ps" not in items[5]) and ("sh" not in items[5]) and ("grep" not in items[5]) and ("tail" not in items[5]) and ("top" not in items[5])):
                            #print items
                            add_new_process = 1
                            for i in range(len(ProcessList)):
                                p = ProcessList[i]
                                if((p.PID == items[PID_Column]) and (p.Name == items[COMMAND_Column])): #Should check name of process too
                                    add_new_process = 0
                                    temp = items[CPU_Column]
                                    lastchar = items[CPU_Column][len(temp)-1]
                                    cpu = 0
                                    if(items[COMMAND_Column].startswith("0")):
                                        pdb.set_trace()
                                    if (lastchar.isdigit() == 1):
                                        cpu = temp
                                    elif (lastchar == "%"):
                                        cpu = float(temp[:-1])
                                    else:
                                        print "CPU units: ",lastchar, " are not supported.  Exiting"
                                        sys.exit(0)
                                    ProcessList[i].CPU_Perc.append(cpu)
                            if(add_new_process == 1):
                                ProcessList.append(Process(items[PID_Column],items[COMMAND_Column]))
                                for j in range(entry_counter-1):
                                    ProcessList[len(ProcessList)-1].CPU_Perc.append(0)
                                temp = items[CPU_Column]
                                lastchar = items[CPU_Column][len(temp)-1]
                                cpu = 0
                                if (lastchar.isdigit() == 1):
                                    cpu = temp
                                elif (lastchar == "%"):
                                    cpu = float(temp[:-1])
                                else:
                                    print "CPU units: ",lastchar, " are not supported.  Exiting"
                                    sys.exit(0)
                                ProcessList[len(ProcessList)-1].CPU_Perc.append(cpu)
        
           
            #time.sleep(0.1)  #For dev only
    for i in range(len(ProcessList)):
        p = ProcessList[i]
        to_fill = entry_counter - len(p.CPU_Perc)
        if (to_fill > 0):
            for j in range(to_fill):
                ProcessList[i].CPU_Perc.append(0)
    for i in range(len(ProcessList)):
        print i,len(ProcessList[i].CPU_Perc)
    path = out_dir + "top_cpu_used.csv"
    outfile = open(path,'w')
    tempstr = "Index,"
    for p in ProcessList:
        tempstr = tempstr + p.Name + "_" + p.PID + ","
    tempstr = tempstr + "\n"
    outfile.write(tempstr)
    tempstr = ""
    for i in range(entry_counter):
        tempstr = str(i) + ","
        for p in ProcessList:
            tempstr = tempstr + str(p.CPU_Perc[i]) + ","
        tempstr = tempstr + "\n"
        #print tempstr
        outfile.write(tempstr)   
    print "Process Count: ",len(ProcessList)   

if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-p"):
    in_file = sys.argv[2] + "/top_log.txt"
    process_log(in_file,sys.argv[2])

        
