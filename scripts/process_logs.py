import sys,getopt,os
from optparse import OptionParser
from util.Helpers import *
from util.bag2csv import *
import rosbag
import shutil
def convert_performance_logs(bag_dir,output_dir):
    topicTypeList = ['eros/loadfactor','eros/resource']
    print(CGREEN + "Starting Conversion for Performance Logs..."  + CEND)
    convert_logs(bag_dir,output_dir,topicTypeList)

def main():
    parser = OptionParser("process_logs.py [options]")
    parser.add_option("-d","--directory",dest="directory",default="~/",help="Location of Bag Files.")
    parser.add_option("-o","--output",dest="output",default="~/temp/",help="Output Location")
    (opts,args) = parser.parse_args()
    convert_performance_logs(opts.directory,opts.output)


if __name__ == "__main__":
    main()
