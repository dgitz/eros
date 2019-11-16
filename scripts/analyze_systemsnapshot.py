#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys,getopt,os
import subprocess
import os,shutil
import time
import matplotlib.pyplot as plt 
from os import walk
import pdb
FileType_UNKNOWN = 0
FileType_RESOURCEUSED = 1
FileType_RESOURCEAVAILABLE = 2
FileType_LOADFACTOR = 3
FileType_UPTIME = 4

DataType_UNKNOWN = 0
DataType_CPUPERC = 1
DataType_RAMPERC = 2
DataType_DISKPERC = 3
DataType_LOADFACTOR_1MIN = 4
DataType_LOADFACTOR_5MIN = 5
DataType_LOADFACTOR_15MIN = 6

class Signal():
    def __init__(self,filetype=FileType_UNKNOWN,datatype=DataType_UNKNOWN,name=''):
        self.filetype = filetype
        self.name = name
        self.datatype = datatype
        self.tov = []
        self.status = []
        self.value = []
        self.type = []
        self.rms = []

def print_usage():
    print "Usage Instructions: analyze_systmesnapshot."
    print "\tNo Options: This Menu."
    print "\t-?/-h This Menu."
    print "\t-a <bag_directory> Analyze Single BAG->CSV Extracted Directory."
    print "\t-s <SystemSnap Directory> Analyze a SystemSnap Extracted Directory."

def load_alldata(directory):
    signals = []
    tempstr = "\nLoading Directory: " + directory
    files_loaded = []
    for (dirpath, dirnames, filenames) in walk(directory):
        for f in filenames:
            if (f[len(f)-4:len(f)] == '.csv'):
                if ('resource.csv' in f):
                    signals.append(load_csv(FileType_RESOURCEUSED,dirpath,f))
                    files_loaded.append(f)
                elif ('resource_available.csv' in f):
                    signals.append(load_csv(FileType_RESOURCEAVAILABLE,dirpath,f))
                    files_loaded.append(f)
                elif ('loadfactor.csv' in f):
                    signals.append(load_csv(FileType_LOADFACTOR,dirpath,f))
                    files_loaded.append(f)
                #elif ('uptime.csv' in f):
                #    print "Loading Uptime file: " + f
                else:
                    tempstr += "\nWARN: Not Loading: " + f
    if(len(signals) == 0):
        tempstr += "\nWARN: No Signals loaded for this log."
    for f in files_loaded:
        tempstr+="\nLoaded file: " + f
    return signals,tempstr
def process_singlebagdirectory(csv_directory):
    tempstr = ""
    signals,tempstr = load_alldata(csv_directory)
    print tempstr
    start_time,signals = sync_timebase(signals)
    drawgraphs(signals,True)
def process_systemsnapdirectory(systemsnap_directory):
    if(os.path.isdir(systemsnap_directory + "/output") == True):
        shutil.rmtree(systemsnap_directory + "/output")
    os.mkdir(systemsnap_directory + "/output")
    fd = open(systemsnap_directory + "/output/analysis.txt","w+")
    for (dirpath, dirnames, filenames) in walk(systemsnap_directory):
        for directory in dirnames:
            if(directory[len(directory)-4:len(directory)] == '_CSV'):
                print "Reading Directory: " + directory
                signals,tempstr = load_alldata(dirpath + "/" + directory + "/")
                fd.write(tempstr)
                if(len(signals) == 0):
                    continue
                start_time,signals = sync_timebase(signals)
                figs = drawgraphs(signals,False)
                os.mkdir(systemsnap_directory + "/output/" + directory)
                fd.write("\nSaving Figures...")
                print "Saving Figures"
                for fig in figs:
                    title = fig._suptitle.get_text() 
                    fig.savefig(systemsnap_directory + "/output/" + directory + "/" + title + ".png",bbox_inches='tight',dpi=600)
                fd.write("\nFigures Saved.")
        break
    fd.close()
def drawgraphs(signals,show_figures):
    figlist = []
    cpuused_perc_signals = get_signalsbytype(signals,FileType_RESOURCEUSED,DataType_CPUPERC)
    if(len(cpuused_perc_signals) > 0):
        title = 'CPU Used by Node (%)'
        fig = plt.figure()
        plt1 = fig.add_subplot(111)
        for i in range(0,len(cpuused_perc_signals)):
            plt1.plot(cpuused_perc_signals[i].tov,cpuused_perc_signals[i].value,label=cpuused_perc_signals[i].name)      
        plt1.legend(prop={'size':10})    
        plt.xlabel('Time (sec)')
        plt.ylabel('Percent')
        plt.suptitle(title)
        plt.ylim(bottom=0,top=100)
        plt.xlim(left=0) 
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        print "WARN: No CPU Used Perc Signals Available."
    ramused_perc_signals = get_signalsbytype(signals,FileType_RESOURCEUSED,DataType_RAMPERC)
    if(len(ramused_perc_signals) > 0):
        title = 'RAM Used by Node (%)'
        fig = plt.figure()
        plt1 = fig.add_subplot(111)
        for i in range(0,len(ramused_perc_signals)):
            plt1.plot(ramused_perc_signals[i].tov,ramused_perc_signals[i].value,label=ramused_perc_signals[i].name)      
        plt1.legend(prop={'size':10})        
        plt.xlabel('Time (sec)')
        plt.ylabel('Percent')
        plt.suptitle(title)
        plt.ylim(bottom=0,top=100)
        plt.xlim(left=0) 
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        print "WARN: No RAM Used Perc Signals Available."
    diskused_perc_signals = get_signalsbytype(signals,FileType_RESOURCEUSED,DataType_DISKPERC)
    if(len(diskused_perc_signals) > 0):
        title = 'DISK Used by Node (%)'
        fig = plt.figure()
        plt1 = fig.add_subplot(111)
        for i in range(0,len(diskused_perc_signals)):
            plt1.plot(diskused_perc_signals[i].tov,diskused_perc_signals[i].value,label=diskused_perc_signals[i].name)      
        plt1.legend(prop={'size':10})          
        plt.xlabel('Time (sec)')
        plt.ylabel('Percent')
        plt.suptitle(title)
        plt.ylim(bottom=0,top=100)
        plt.xlim(left=0) 
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        print "WARN: No DISK Used Perc Signals Available."
    loadfactor_1min_signals = get_signalsbytype(signals,FileType_LOADFACTOR,DataType_LOADFACTOR_1MIN)
    loadfactor_5min_signals = get_signalsbytype(signals,FileType_LOADFACTOR,DataType_LOADFACTOR_5MIN)
    loadfactor_15min_signals = get_signalsbytype(signals,FileType_LOADFACTOR,DataType_LOADFACTOR_15MIN)
    if(len(loadfactor_1min_signals) > 0):
        title = 'Scaled Load Factor by Device'
        fig = plt.figure()        
        for j in range(0,len(loadfactor_1min_signals)):
            subplot_str = str(len(loadfactor_1min_signals)) + '1' + str(j+1)
            plt1 = fig.add_subplot(subplot_str)
            for i in range(0,len(loadfactor_1min_signals)):
                plt1.plot(loadfactor_1min_signals[i].tov,loadfactor_1min_signals[i].value,label='1 Min')  
                plt1.plot(loadfactor_5min_signals[i].tov,loadfactor_5min_signals[i].value,label='5 Min')
                plt1.plot(loadfactor_15min_signals[i].tov,loadfactor_15min_signals[i].value,label='15 Min')
            plt1.legend(prop={'size':10})  
            plt1.title.set_text(loadfactor_1min_signals[i].name) 
            plt.xlim(left=0)      
        plt.xlabel('Time (sec)')
        plt.ylabel('Factor')
        plt.suptitle(title)
        plt.xlim(left=0) 
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        print "WARN: No Load Factor Signals Available."
    cpuav_perc_signals = get_signalsbytype(signals,FileType_RESOURCEAVAILABLE,DataType_CPUPERC)
    ramav_perc_signals = get_signalsbytype(signals,FileType_RESOURCEAVAILABLE,DataType_RAMPERC)
    diskav_perc_signals = get_signalsbytype(signals,FileType_RESOURCEAVAILABLE,DataType_DISKPERC)
    if(len(cpuav_perc_signals) > 0):
        title = 'Resource Available by Device'
        fig = plt.figure() 
        plt1 = fig.add_subplot(311)
        for i in range(0,len(cpuav_perc_signals)):
            plt1.plot(cpuav_perc_signals[i].tov,cpuav_perc_signals[i].value,label=cpuav_perc_signals[i].name)
        plt1.legend(prop={'size':10})  
        plt1.set_ylabel('Percent')
        plt1.title.set_text('CPU Available')
        plt.ylim(bottom=0,top=100) 
        plt.xlim(left=0) 
        plt2 = fig.add_subplot(312)
        for i in range(0,len(ramav_perc_signals)):
            plt2.plot(ramav_perc_signals[i].tov,ramav_perc_signals[i].value,label=ramav_perc_signals[i].name)
        plt2.title.set_text('RAM Available')
        plt2.set_ylabel('Percent') 
        plt2.legend(prop={'size':10})
        plt.ylim(bottom=0,top=100) 
        plt.xlim(left=0) 
        plt3 = fig.add_subplot(313)
        for i in range(0,len(diskav_perc_signals)):
            plt3.plot(diskav_perc_signals[i].tov,diskav_perc_signals[i].value,label=diskav_perc_signals[i].name)
        plt3.title.set_text('DISK Available') 
        plt3.set_ylabel('Percent')
        plt3.legend(prop={'size':10}) 
        plt.ylim(bottom=0,top=100)
        plt.xlim(left=0) 
        plt.xlabel('Time (sec)')
        plt.suptitle(title)
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        print "WARN: No Resource Available Signals Available."
    if(show_figures == 1):
        plt.show()
    return figlist
    
def sync_timebase(signals):
    start_time = signals[0][0].tov[0]
    for i in range(0,len(signals)):
        for j in range(0,len(signals[i])):
            new_tov = []
            for k in range(0,len(signals[i][j].tov)):
                new_tov.append(signals[i][j].tov[k]-start_time)
            signals[i][j].tov = new_tov
    return start_time,signals

def get_signalsbytype(signals,filetype,datatype):
    out_signals = []
    for i in range(0,len(signals)):
        for j in range(0,len(signals[i])):
            if((signals[i][j].filetype == filetype) and (signals[i][j].datatype == datatype)):
                out_signals.append(signals[i][j])
    return out_signals

def main():
    if len(sys.argv) == 1:
        print_usage()
        sys.exit(0)
    elif (sys.argv[1] == "-h"):
        print_usage()
        sys.exit(0)
    elif (sys.argv[1] == "-a"):
        process_singlebagdirectory(sys.argv[2])
    elif (sys.argv[1] == "-s"):
        process_systemsnapdirectory(sys.argv[2])
    else:
        print_usage()
        sys.exit(0)

def load_csv(filetype,directory,f):
    signals = []
    if(filetype == FileType_UNKNOWN):
        a = 1 #Do Nothing
    elif(filetype == FileType_RESOURCEUSED):
        nodename = f[0:len(f)-13]
        signal_cpuused_perc = Signal(FileType_RESOURCEUSED,DataType_CPUPERC,nodename)
        signal_ramused_perc = Signal(FileType_RESOURCEUSED,DataType_RAMPERC,nodename)
        signal_diskused_perc = Signal(FileType_RESOURCEUSED,DataType_DISKPERC,nodename)
        TIMESTAMP_SEC_COL = 1
        TIMESTAMP_NSEC_COL = 2
        CPUPERC_COL = 6
        RAMPERC_COL = 7
        DISKPERC_COL = 8
        tov = []
        cpu = []
        ram = []
        disk = []
        with open (directory + f) as fd:
            content = fd.readlines()
            counter = 0
            for line in content:
                data = line.split(",")
                if(counter == 0):
                    a = 1 # Do Nothing
                else:
                    sec = float(data[TIMESTAMP_SEC_COL])
                    nsec = float(data[TIMESTAMP_NSEC_COL])
                    tov.append(sec + nsec/1000000000.0)
                    cpu.append(float(data[CPUPERC_COL]))
                    ram.append(float(data[RAMPERC_COL]))
                    disk.append(float(data[DISKPERC_COL]))
                    
                counter=counter+1
        signal_cpuused_perc.tov = tov
        signal_cpuused_perc.value = cpu
        signal_ramused_perc.tov = tov
        signal_ramused_perc.value = ram
        signal_diskused_perc.tov = tov
        signal_diskused_perc.value = disk
        signals.append(signal_cpuused_perc)
        signals.append(signal_ramused_perc)
        signals.append(signal_diskused_perc)
    elif(filetype == FileType_LOADFACTOR):
        devicename = f[0:len(f)-15]
        signal_1min = Signal(FileType_LOADFACTOR,DataType_LOADFACTOR_1MIN,devicename)
        signal_5min = Signal(FileType_LOADFACTOR,DataType_LOADFACTOR_5MIN,devicename)
        signal_15min = Signal(FileType_LOADFACTOR,DataType_LOADFACTOR_15MIN,devicename)
        TIMESTAMP_SEC_COL = 1
        TIMESTAMP_NSEC_COL = 2
        LOADFACTOR_1MIN_COL = 5
        LOADFACTOR_5MIN_COL = 6
        LOADFACTOR_15MIN_COL = 7
        tov = []
        min1 = []
        min5 = []
        min15 = []
        with open (directory + f) as fd:
            content = fd.readlines()
            counter = 0
            for line in content:
                data = line.split(",")
                if(counter == 0):
                    a = 1 # Do Nothing
                else:
                    sec = float(data[TIMESTAMP_SEC_COL])
                    nsec = float(data[TIMESTAMP_NSEC_COL])
                    tov.append(sec + nsec/1000000000.0)
                    min1.append(float(data[LOADFACTOR_1MIN_COL]))
                    min5.append(float(data[LOADFACTOR_5MIN_COL]))
                    min15.append(float(data[LOADFACTOR_15MIN_COL].strip()[0:-1]))
                    
                counter=counter+1
        signal_1min.tov = tov
        signal_1min.value = min1
        signal_5min.tov = tov
        signal_5min.value = min5
        signal_15min.tov = tov
        signal_15min.value = min15
        signals.append(signal_1min)
        signals.append(signal_5min)
        signals.append(signal_15min)
    elif(filetype == FileType_RESOURCEAVAILABLE):
        devicename = f[0:len(f)-23]
        signal_cpuav_perc = Signal(FileType_RESOURCEAVAILABLE,DataType_CPUPERC,devicename)
        signal_ramav_perc = Signal(FileType_RESOURCEAVAILABLE,DataType_RAMPERC,devicename)
        signal_diskav_perc = Signal(FileType_RESOURCEAVAILABLE,DataType_DISKPERC,devicename)
        TIMESTAMP_SEC_COL = 1
        TIMESTAMP_NSEC_COL = 2
        CPUPERC_COL = 6
        RAMPERC_COL = 7
        DISKPERC_COL = 8
        tov = []
        cpu = []
        ram = []
        disk = []
        with open (directory + f) as fd:
            content = fd.readlines()
            counter = 0
            for line in content:
                data = line.split(",")
                if(counter == 0):
                    a = 1 # Do Nothing
                else:
                    sec = float(data[TIMESTAMP_SEC_COL])
                    nsec = float(data[TIMESTAMP_NSEC_COL])
                    tov.append(sec + nsec/1000000000.0)
                    cpu.append(float(data[CPUPERC_COL]))
                    ram.append(float(data[RAMPERC_COL]))
                    disk.append(float(data[DISKPERC_COL]))
                    
                counter=counter+1
        signal_cpuav_perc.tov = tov
        signal_cpuav_perc.value = cpu
        signal_ramav_perc.tov = tov
        signal_ramav_perc.value = ram
        signal_diskav_perc.tov = tov
        signal_diskav_perc.value = disk
        signals.append(signal_cpuav_perc)
        signals.append(signal_ramav_perc)
        signals.append(signal_diskav_perc)
    else:
        a = 1 #Do Nothing
    return signals

if __name__ == "__main__":
    main()
