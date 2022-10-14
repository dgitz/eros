import sys,getopt,os
from optparse import OptionParser
from util.Helpers import *
from util.bag2csv import *
import rosbag
import shutil


def performance_scan(bag_dir,output_dir):
    topicTypeList = ['eros/loadfactor','eros/resource']
    topicNameList = ['loadfactor','resource_used','resource_available']
    print(CGREEN + "Starting Performance Scan..."  + CEND)
    # Check bag_dir for bag files

    bagFileList=[]
    for file in os.listdir(bag_dir):
        if file.endswith(".bag"):
            bagFileList.append(file)
    print("Found: " + str(len(bagFileList)) + " Bag Files.")
    if len(bagFileList) == 0:
        print(CYELLOW + "No Bag Files found in Directory. Exiting." + CEND)
        return

    # Clear out Output Folder
    if os.path.exists(output_dir) and os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    
    for file in bagFileList:
        fileName = os.path.join(bag_dir,file)
        print("Scanning " + fileName)
        # Get Topic Names of Interest
        bag = rosbag.Bag(fileName)
        keys = list(bag.get_type_and_topic_info()[1].keys())
        values = list(bag.get_type_and_topic_info()[1].values())
        for i in range(0,len(keys)):
            topicName = keys[i]
            topicType = values[i].msg_type
            
            if any(topicType in x for x in topicTypeList):
                bag2csv(fileName,topicName,topicType,output_dir)
            




def main():
    parser = OptionParser("analyze_logs.py [options]")
    parser.add_option("-m","--mode",dest="mode",default="performance",help="performance")
    parser.add_option("-d","--directory",dest="directory",default="~/",help="Location of Bag Files.")
    parser.add_option("-o","--output",dest="output",default="~/temp/",help="Output Location")
    (opts,args) = parser.parse_args()
    if(opts.mode=="performance"):
        performance_scan(opts.directory,opts.output)


if __name__ == "__main__":
    main()
