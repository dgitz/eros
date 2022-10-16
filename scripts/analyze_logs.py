import sys,getopt,os
from optparse import OptionParser
from util.Helpers import *
from util.bag2csv import *
import rosbag
import shutil
import util.performance_analysis
import csv
import matplotlib.pyplot as plt
class LoadFactor:
    def __init__(self):
        self.device = ""
        self.stamp = []
        self.loadfactor_1min = []
        self.loadfactor_5min = []
        self.loadfactor_15min = []
    def add(self,stamp,lf_1min,lf_5min,lf_15min):
        self.stamp.append(stamp)
        self.loadfactor_1min.append(lf_1min)
        self.loadfactor_5min.append(lf_5min)
        self.loadfactor_15min.append(lf_15min)

class ResourceAvailable:
    def __init__(self):
        self.device = ""
        self.stamp = []
        self.PID = []
        self.CPU = []
        self.RAM = []
        self.Disk = []
    def add(self,stamp,CPU_av,RAM_av,Disk_av):
        self.stamp.append(stamp)
        self.CPU.append(CPU_av)
        self.RAM.append(RAM_av)
        self.Disk.append(Disk_av)
def convert_performance_logs(bag_dir,output_dir):
    topicTypeList = ['eros/loadfactor','eros/resource']
    print(CGREEN + "Starting Performance Scan..."  + CEND)
    convert_logs(bag_dir,output_dir,topicTypeList)

def analyze_performance_logs(csv_dir,output_dir):
     # Clear out Output Folder
    if os.path.exists(output_dir) and os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)
    # Load Data into memory
    ANALYZE_RAM=0
    ANALYZE_CPU=0
    ANALYZE_LOADFACTOR=1
    fileList = [os.path.join(r,file) for r,d,f in os.walk(csv_dir) for file in f]
        #for f in filenames:
        #    files.append(dir)
    figList = []
    for f in fileList:
        loadFactor = LoadFactor()
        resourceAvailable = ResourceAvailable()
        f_name = os.path.basename(f)
        if f_name == "loadfactor.csv" or f_name == "resource_available.csv":
            f = open(f,'r', newline='')
            reader = csv.reader(f, delimiter=',', quotechar='|')
            first = True
            for row in reader:
                if first == True:
                    first = False
                    continue
                if f_name == "loadfactor.csv":
                    if loadFactor.device == "":
                        loadFactor.device = row[1][1:]
                    loadFactor.add(float(row[0]),float(row[2]),float(row[3]),float(row[4]))
                elif f_name == "resource_available.csv":
                    if resourceAvailable.device == "":
                        resourceAvailable.device = row[1][1:]
                    resourceAvailable.add(float(row[0]),float(row[3]),float(row[4]),float(row[5]))
            if f_name == "loadfactor.csv":
                util.performance_analysis.analyze_loadfactor(output_dir,loadFactor)
                loadFactor = LoadFactor()
            elif f_name == "resource_available.csv":
                util.performance_analysis.analyze_resourceavailable(output_dir,resourceAvailable)
                resourceAvailable = ResourceAvailable()
                

    
            





def main():
    parser = OptionParser("analyze_logs.py [options]")
    parser.add_option("-m","--mode",dest="mode",default="performance",help="performance")
    parser.add_option("-s","--submode",dest="submode",default="analysis",help="conversion,analysis")
    parser.add_option("-d","--directory",dest="directory",default="~/",help="Location of Bag Files.")
    parser.add_option("-o","--output",dest="output",default="~/temp/",help="Output Location")
    (opts,args) = parser.parse_args()
    if(opts.mode=="performance"):
        if(opts.submode == "conversion"):
            convert_performance_logs(opts.directory,opts.output)
        elif(opts.submode == "analysis"):
            analyze_performance_logs(opts.directory,opts.output)
        else:
            print(CRED + "Not Supported!" + CEND)
    else:
        print(CRED + "Not Supported!" + CEND)



if __name__ == "__main__":
    main()
