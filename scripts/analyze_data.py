#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys,getopt,os
import subprocess
import os,shutil
import time
import copy
import matplotlib.pyplot as plt 
import datetime
import math
import eROS_Definitions as eros
from optparse import OptionParser
from os import walk
import pdb
FileType_UNKNOWN = 0
FileType_RESOURCEUSED = 1
FileType_RESOURCEAVAILABLE = 2
FileType_LOADFACTOR = 3
FileType_UPTIME = 4
FileType_DIAGNOSTIC = 5
FileType_ARMEDSTATE = 6


DataType_UNKNOWN = 0
DataType_CPUPERC = 1
DataType_RAMPERC = 2
DataType_RAMMB = 3
DataType_DISKPERC = 4
DataType_LOADFACTOR_1MIN = 5
DataType_LOADFACTOR_5MIN = 6
DataType_LOADFACTOR_15MIN = 7
DataType_DIAGNOSTICLEVEL = 8
DataType_ARMEDSTATE = 9
class Signal():
    def __init__(self,filetype=FileType_UNKNOWN,datatype=DataType_UNKNOWN,name='',node_name=''):
        self.filetype = filetype
        self.name = name
        self.node_name = node_name
        self.datatype = datatype
        self.tov = []
        self.status = []
        self.value = []
        self.type = []
        self.rms = []
class Message():
    def __init__(self,name,count,data_type):
        self.name = name
        self.count = count
        self.data_type = data_type

def convert_datetime(timestamp):
    tov = 0
    error = 0
    timestamp = timestamp.replace(':','/')
    v = timestamp.split('/')
    dsec = 0
    try:
        dsec = float(v[5])
    except:
        error = 1
        return tov,error
    sec = int(math.floor(dsec))
    msec = int(round(1000.0*(dsec-sec)))
    t = datetime.datetime(int(v[0]),int(v[1]),int(v[2]),int(v[3]),int(v[4]),sec,msec)
    t_tov = (t-datetime.datetime(1970,1,1)).total_seconds()
    return t_tov,error
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
                elif ('diagnostic.csv' in f):
                    signals.append(load_csv(FileType_DIAGNOSTIC,dirpath,f))
                    files_loaded.append(f)
                elif ('armed_state.csv' in f):
                    signals.append(load_csv(FileType_ARMEDSTATE,dirpath,f))
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
def analyze(data_directory,output_directory,extra_signal_list,show_graphics):
    signals = []
    messages = []
    log_directories = []
    log_duration = 0
    total_duration = 0
    for (dirpath, dirnames, filenames) in walk(data_directory):
        dirnames.sort()
        for directory in dirnames:
            log_directories.append(directory)
            print "Loading Directory: " + directory
            new_signals,tempstr = load_alldata(dirpath + "/" + directory + "/")
            flattened_signals = flatten(new_signals)
            for ii in range(0,len(flattened_signals)):
                found_me = False
                for jj in range(0,len(signals)):
                    if(flattened_signals[ii].name == signals[jj].name):
                        found_me = True
                        dt = flattened_signals[ii].tov[0]-signals[jj].tov[-1]
                        if(dt < 0.0):
                            print "ERROR: Current Directory has a timestamp less than the previous directory."
                        signals[jj].tov.extend(flattened_signals[ii].tov)
                        signals[jj].value.extend(flattened_signals[ii].value)
                        signals[jj].status.extend(flattened_signals[ii].status)
                        signals[jj].type.extend(flattened_signals[ii].type)
                        signals[jj].rms.extend(flattened_signals[ii].rms)
                if(found_me == False):
                    signals.append(flattened_signals[ii])
            log_duration = log_duration + (flattened_signals[0].tov[-1] - flattened_signals[0].tov[0])
            print "Loading Bag Info"
            with open (dirpath + "/" + directory + "/bag_info.txt") as fd:
                content = fd.readlines()
                counter = 0
                in_messages = False
                for line in content:
                    if("topics:" in line):
                        in_messages = True
                    if(in_messages):
                        data = line.split()
                        message = []
                        if(data[0] == "topics:"):
                            message = Message(data[1],int(data[2]),data[5])
                        else:
                            message = Message(data[0],int(data[1]),data[4])
                        found_me = False
                        for i in range(0,len(messages)):
                            if(messages[i].name == message.name):
                                found_me = True
                                messages[i].count = messages[i].count + message.count
                        if(found_me == False):
                            messages.append(message)
                    counter = counter + 1
    extra_signals = []
    list_sig = extra_signal_list.split(',')
    for i in range(0,len(list_sig)):
        for j in range(0,len(signals)):
            if(list_sig[i] in signals[j].name):
                extra_signals.append(signals[j])
    #Find Earliest start time
    stop_time = 0  # A small start time
    start_time = signals[0].tov[0]*1000000.0  # A really big start time
    for i in range(0,len(signals)):
        if signals[i].tov[0] < start_time:
            start_time = signals[i].tov[0]
        if signals[i].tov[-1] > stop_time:
            stop_time = signals[i].tov[-1]
    for i in range(0,len(signals)):
        signals[i].tov = [x - start_time for x in signals[i].tov]  
    print "Analyzing Data"    
    fd = open(output_directory + "/analysis.txt","w+")
    for i in range(0,len(log_directories)):
        fd.write("Log Directory: " + log_directories[i] + "\n")
    fd.write("Log Start Time: " + str(start_time) + " sec\n")
    fd.write("Log Stop Time: " + str(stop_time) + " sec\n")
    fd.write("Log Duration: " + str(log_duration) + " sec\n")
    fd.write("Total Duration: " + str(stop_time-start_time) + " sec\n")
    # Signal Type Specific Analysis
    fd.write("RAM Usage Analysis\n")
    ramused_MB_signals = get_signalsbytype(signals,FileType_RESOURCEUSED,DataType_RAMMB)
    potential_memoryleak_signals = []
    bin_count = 10
    perc_increase = 5.0
    for i in range(0,len(ramused_MB_signals)):
        v = []
        bin_t = (ramused_MB_signals[i].tov[-1]-ramused_MB_signals[i].tov[0])/bin_count
        t = ramused_MB_signals[i].tov[0]
        for j in range(0,len(ramused_MB_signals[i].tov)):
            dt = ramused_MB_signals[i].tov[j]-t
            if(dt > bin_t):
                v.append(ramused_MB_signals[i].value[j])
                t = ramused_MB_signals[i].tov[j]
        count = 0
        for j in range(1,len(v)):
            if(v[j] > ((1+perc_increase/100.0)*v[0])):
                count = count + 1
        if(count > (bin_count/2)):
            sig = ramused_MB_signals[i]
            potential_memoryleak_signals.append(sig)
    if(len(potential_memoryleak_signals) > 0):
        tempstr = "[WARN]: Potential Memory Leak Signals:"
        print tempstr
        fd.write(tempstr + "\n")
        for i in range(0,len(potential_memoryleak_signals)):
            tempstr = "[" + str(i+1) + "/" + str(len(potential_memoryleak_signals)) + "] Name: " + potential_memoryleak_signals[i].name
            print tempstr
            fd.write("\t" + tempstr + "\n")
    fd.write("Armed State Analysis\n")
    armed_states = []
    armed_state_count = 0
    for i in range(0,eros.ARMEDSTATUS_ARMING+1):
        armed_states.append(0)
    for i in range(0,len(signals)):
        if((signals[i].filetype == FileType_ARMEDSTATE) and (signals[i].datatype == DataType_ARMEDSTATE)):
            for j in range(0,len(signals[i].tov)):
                armed_states[signals[i].value[j]] = armed_states[signals[i].value[j]] + 1
                armed_state_count = armed_state_count + 1
    for i in range(0,len(armed_states)):
        if(armed_states[i] > 0):
            fd.write("In Armed State: " + str(i) + " For " + str(100.0*armed_states[i]/armed_state_count) + "%\n")
    fd.write("Diagnostic Analysis\n")
    total_warn_count = 0
    total_error_count = 0
    total_fatal_count = 0
    for i in range(0,len(signals)):
        if((signals[i].filetype == FileType_DIAGNOSTIC) and (signals[i].datatype == DataType_DIAGNOSTICLEVEL)):
            warn_count = 0
            error_count = 0
            fatal_count = 0            
            for j in range(len(signals[i].tov)):
                if(signals[i].value[j] == eros.WARN):
                    warn_count = warn_count + 1
                elif(signals[i].value[j] == eros.ERROR):
                    error_count = error_count + 1
                elif(signals[i].value[j] == eros.FATAL):
                    fatal_count = fatal_count + 1
            if(warn_count > 0):
                fd.write("[WARN]: Signal: " + signals[i].name + " Warn Count=" + str(warn_count) + "\n")
            if(error_count > 0):
                fd.write("[ERROR]: Signal: " + signals[i].name + " Error Count=" + str(error_count) + "\n")
            if(fatal_count > 0):
                fd.write("[FATAL]: Signal: " + signals[i].name + " Fatal Count=" + str(fatal_count) + "\n")
            total_warn_count = total_warn_count + warn_count
            total_error_count = total_error_count + error_count
            total_fatal_count = total_fatal_count + fatal_count
    fd.write("Warn Count: " + str(total_warn_count) + "\n")
    fd.write("Error Count: " + str(total_error_count) + "\n")
    fd.write("Fatal Count: " + str(total_fatal_count) + "\n")
    fd.write("Contains Signals:\n")
    for i in range(0,len(signals)):
        fd.write("\t[" + str(i+1) + "/" + str(len(signals)) + "] " + signals[i].name + "\n")
    fd.write("Contains Messages:\n")
    for i in range(0,len(messages)):
        rate = messages[i].count/log_duration
        fd.write("\t[" + str(i+1) + "/" + str(len(messages)) +"] Msg: " + messages[i].name + " type: " + messages[i].data_type + " count: " + str(messages[i].count) + " Rate: " + str(rate) + " Hz\n")
    fd.close()
    print "Analysis Complete"
    figs = drawgraphs(signals,show_graphics,False)
    drawgraphs(extra_signals,False,True)
    if(show_graphics == False):
        print "Saving Figures"
        for fig in figs:
            title = fig._suptitle.get_text() 
            fig.savefig(output_directory + "/" + title + ".png",bbox_inches='tight',dpi=600)
        print "Figures Saved"
    else:
        plt.show()
    
def flatten(signals):
    output = []
    for i in range(0,len(signals)):
        for j in range(0,len(signals[i])):
            output.append(signals[i][j])
    return output
def drawgraphs(signals,show_figures,suppress_warnings):
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
        if(suppress_warnings == False):
            print "WARN: No CPU Used Perc Signals Available."
    ramused_MB_signals = get_signalsbytype(signals,FileType_RESOURCEUSED,DataType_RAMMB)
    if(len(ramused_MB_signals) > 0):
        title = 'RAM Used by Node (MB)'
        fig = plt.figure()
        plt1 = fig.add_subplot(111)
        for i in range(0,len(ramused_MB_signals)):
            plt1.plot(ramused_MB_signals[i].tov,ramused_MB_signals[i].value,label=ramused_MB_signals[i].name)      
        plt1.legend(prop={'size':10})        
        plt.xlabel('Time (sec)')
        plt.ylabel('RAM (MB)')
        plt.suptitle(title)
        plt.xlim(left=0) 
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        if(suppress_warnings == False):
            print "WARN: No RAM Used Perc Signals Available."
    loadfactor_1min_signals = get_signalsbytype(signals,FileType_LOADFACTOR,DataType_LOADFACTOR_1MIN)
    loadfactor_5min_signals = get_signalsbytype(signals,FileType_LOADFACTOR,DataType_LOADFACTOR_5MIN)
    loadfactor_15min_signals = get_signalsbytype(signals,FileType_LOADFACTOR,DataType_LOADFACTOR_15MIN)
    if(len(loadfactor_1min_signals) > 0):
        title = 'Scaled Load Factor by Device'
        fig = plt.figure()        
        for j in range(0,len(loadfactor_1min_signals)):
            subplot_str = str(len(loadfactor_1min_signals)) + '1' + str(j+1)
            plt1 = []
            plt1 = fig.add_subplot(subplot_str)
            plt1.plot(loadfactor_1min_signals[j].tov,loadfactor_1min_signals[j].value,label='1 Min')  
            plt1.plot(loadfactor_5min_signals[j].tov,loadfactor_5min_signals[j].value,label='5 Min')
            plt1.plot(loadfactor_15min_signals[j].tov,loadfactor_15min_signals[j].value,label='15 Min')
            plt1.legend(prop={'size':10})  
            plt1.title.set_text(loadfactor_1min_signals[j].node_name) 
            plt.xlim(left=0)  
            plt.ylim(0,1)
        plt.xlabel('Time (sec)')
        plt.ylabel('Factor')
        plt.suptitle(title)
        plt.xlim(left=0) 
        fig.canvas.set_window_title(title)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figlist.append(fig)
    else:
        if(suppress_warnings == False):
            print "WARN: No Load Factor Signals Available."
    cpuav_perc_signals = get_signalsbytype(signals,FileType_RESOURCEAVAILABLE,DataType_CPUPERC)
    ramav_perc_signals = get_signalsbytype(signals,FileType_RESOURCEAVAILABLE,DataType_RAMPERC)
    diskav_perc_signals = get_signalsbytype(signals,FileType_RESOURCEAVAILABLE,DataType_DISKPERC)
    if(len(cpuav_perc_signals) > 0):
        title = 'Resource Available by Device'
        fig = plt.figure() 
        plt1 = fig.add_subplot(311)
        for i in range(0,len(cpuav_perc_signals)):
            plt1.plot(cpuav_perc_signals[i].tov,cpuav_perc_signals[i].value,label=cpuav_perc_signals[i].node_name)
        plt1.legend(prop={'size':10})  
        plt1.set_ylabel('Percent')
        plt1.title.set_text('CPU Available')
        plt.ylim(bottom=0,top=100) 
        plt.xlim(left=0) 
        plt2 = fig.add_subplot(312)
        for i in range(0,len(ramav_perc_signals)):
            plt2.plot(ramav_perc_signals[i].tov,ramav_perc_signals[i].value,label=ramav_perc_signals[i].node_name)
        plt2.title.set_text('RAM Available')
        plt2.set_ylabel('Percent') 
        plt2.legend(prop={'size':10})
        plt.ylim(bottom=0,top=100) 
        plt.xlim(left=0) 
        plt3 = fig.add_subplot(313)
        for i in range(0,len(diskav_perc_signals)):
            plt3.plot(diskav_perc_signals[i].tov,diskav_perc_signals[i].value,label=diskav_perc_signals[i].node_name)
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
        if(suppress_warnings == False):
            print "WARN: No Resource Available Signals Available."
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
        if((signals[i].filetype == filetype) and (signals[i].datatype == datatype)):
            out_signals.append(signals[i])
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
    elif(filetype == FileType_ARMEDSTATE):
        signal_armedstate = Signal(FileType_ARMEDSTATE,DataType_ARMEDSTATE,'armed_state','armed_state')
        tov = []
        armed_state = []
        DATETIME_COL = 0
        ARMEDSTATE_COL = 1
        with open (directory + f) as fd:
            content = fd.readlines()
            counter = 0
            for line in content:
                data = line.split(",")
                if(counter == 0):
                    a = 1 # Do Nothing
                else:
                    time,error = convert_datetime(data[DATETIME_COL])
                    if(error == 1):
                        continue
                    tov.append(time)
                    armed_state.append(int(data[ARMEDSTATE_COL]))
                counter=counter+1
        signal_armedstate.tov = tov[:]
        signal_armedstate.value = armed_state[:]
        signals.append(signal_armedstate)
        
    elif(filetype == FileType_DIAGNOSTIC):
        nodename = f[0:len(f)-15]
        signal_diaglevel = Signal(FileType_DIAGNOSTIC,DataType_DIAGNOSTICLEVEL,nodename+'_diagnostic',nodename)
        DATETIME_COL = 0
        LEVEL_COL = 7
        tov = []
        level = []
        with open (directory + f) as fd:
            content = fd.readlines()
            counter = 0
            for line in content:
                data = line.split(",")
                if(counter == 0):
                    a = 1 # Do Nothing
                else:
                    time,error = convert_datetime(data[DATETIME_COL])
                    if(error == 1):
                        continue
                    tov.append(time)
                    level.append(int(data[LEVEL_COL]))
                counter=counter+1
        signal_diaglevel.tov = tov[:]
        signal_diaglevel.value = level[:]
        signals.append(signal_diaglevel)
        
    elif(filetype == FileType_RESOURCEUSED):
        nodename = f[0:len(f)-13]
        signal_cpuused_perc = Signal(FileType_RESOURCEUSED,DataType_CPUPERC,nodename+'_cpuused',nodename)
        signal_ramused_MB = Signal(FileType_RESOURCEUSED,DataType_RAMMB,nodename+'_ramused',nodename)
        TIMESTAMP_SEC_COL = 1
        TIMESTAMP_NSEC_COL = 2
        RAMMB_COL = 5
        CPUPERC_COL = 6
        tov = []
        cpu = []
        ram = []
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
                    ram.append(float(data[RAMMB_COL]))
                    
                counter=counter+1
        signal_cpuused_perc.tov = tov[:]
        signal_cpuused_perc.value = cpu[:]
        signal_ramused_MB.tov = tov[:]
        signal_ramused_MB.value = ram[:]
        signals.append(signal_cpuused_perc)
        signals.append(signal_ramused_MB)
    elif(filetype == FileType_LOADFACTOR):
        devicename = f[0:len(f)-15]
        signal_1min = Signal(FileType_LOADFACTOR,DataType_LOADFACTOR_1MIN,devicename+'_loadfactor1min',devicename)
        signal_5min = Signal(FileType_LOADFACTOR,DataType_LOADFACTOR_5MIN,devicename+'_loadfactor5min',devicename)
        signal_15min = Signal(FileType_LOADFACTOR,DataType_LOADFACTOR_15MIN,devicename+'_loadfactor15min',devicename)
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
        signal_1min.tov = tov[:]
        signal_1min.value = min1[:]
        signal_5min.tov = tov[:]
        signal_5min.value = min5[:]
        signal_15min.tov = tov[:]
        signal_15min.value = min15[:]
        signals.append(signal_1min)
        signals.append(signal_5min)
        signals.append(signal_15min)
    elif(filetype == FileType_RESOURCEAVAILABLE):
        devicename = f[0:len(f)-23]
        signal_cpuav_perc = Signal(FileType_RESOURCEAVAILABLE,DataType_CPUPERC,devicename+'_cpuavailable',devicename)
        signal_ramav_perc = Signal(FileType_RESOURCEAVAILABLE,DataType_RAMPERC,devicename+'_ramavailable',devicename)
        signal_diskav_perc = Signal(FileType_RESOURCEAVAILABLE,DataType_DISKPERC,devicename+'_diskavailable',devicename)
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
        signal_cpuav_perc.tov = tov[:]
        signal_cpuav_perc.value = cpu[:]
        signal_ramav_perc.tov = tov[:]
        signal_ramav_perc.value = ram[:]
        signal_diskav_perc.tov = tov[:]
        signal_diskav_perc.value = disk[:]
        signals.append(signal_cpuav_perc)
        signals.append(signal_ramav_perc)
        signals.append(signal_diskav_perc)
    else:
        a = 1 #Do Nothing
    return signals

def main():
    parser = OptionParser("analyze_data.py [options]")
    parser.add_option("-d","--data_directory",dest="data_directory",help="Data Directory")
    parser.add_option("-o","--output_directory",dest="output_directory",help="Output Directory")
    parser.add_option("-e","--extra",dest="extra_signals",default="",help="Comma Separated list of signals to graph.")
    parser.add_option("-s","--show/save",dest="show_save",default="Show",help="Show/Save Graphics [default: %default]")
    (opts,args) = parser.parse_args()
    show_graphics = True
    if(opts.show_save == "Save"):
        show_graphics = False
    analyze(opts.data_directory,opts.output_directory,opts.extra_signals,show_graphics)
    

    
if __name__ == "__main__":
    main()
