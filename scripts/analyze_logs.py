import sys,getopt,os
from optparse import OptionParser
from util.Helpers import *
from util.bag2csv import *
import rosbag
import shutil
import util.performance_analysis

def convert_performance_logs(bag_dir,output_dir):
    topicTypeList = ['eros/loadfactor','eros/resource']
    print(CGREEN + "Starting Performance Scan..."  + CEND)
    convert_logs(bag_dir,output_dir,topicTypeList)

def analyze_performance_logs(csv_dir,output_dir):
    # Load Data into memory
    ANALYZE_RAM=0
    ANALYZE_CPU=0
    ANALYZE_LOADFACTOR=1
    fileList = [os.path.join(r,file) for r,d,f in os.walk(csv_dir) for file in f]
        #for f in filenames:
        #    files.append(dir)
    for f in fileList:
        f_name = os.path.basename(f)
        if(f_name == "loadfactor.csv"):
            util.performance_analysis.analyze_loadfactor(f)
        

    
            





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
