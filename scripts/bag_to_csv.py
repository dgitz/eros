#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import rosbag
import subprocess
import os,shutil
from optparse import OptionParser
from datetime import datetime
import time
import pdb
def get_topics(mode,bag):
    topics = []
    search_list = []
    if((mode == "all") or (mode == "pose")):
        topics.append('/RightIMU')
        topics.append('/LeftIMU')
        topics.append('/RightIMU_Temperature')
        topics.append('/LeftIMU_Temperature')
        topics.append('/RightIMU_Simulated')
        topics.append('/LeftIMU_Simulated')
        topics.append('/LeftWheelEncoder_Simulated')
        topics.append('/RightWheelEncoder_Simulated')
        topics.append('/LeftMotorController')
        topics.append('/RightMotorController')
        topics.append('/TiltServo')
        topics.append('/TruthPose_Simulated')
        topics.append('/armed_state')
    if((mode == "all") or (mode == "performance")):
        search_list.append("resource")
        search_list.append("loadfactor")
        search_list.append("uptime")
        search_list.append("diagnostic")
    
    all_topic_list = bag.get_type_and_topic_info().topics
    for t in all_topic_list:
        for item in search_list:
            if(item in t):
                topics.append(t)
    return topics
def message_to_csv(breakpoint,stream, msg, flatten=False,):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(breakpoint,stream, val, flatten)
    except:
        msg_str = str(msg)
        if msg_str.find(",") is not -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
                msg_str = msg_str.replace('(',',')
                msg_str = msg_str.replace(')','')
        stream.write("," + msg_str)

def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name,s]))
    except:
        v = []
        if(parent_content_name[0] == "."):
            v = parent_content_name[1:]
        else:
            v = parent_content_name
        stream.write("," + v)

def format_csv_filename(form, topic_name):
    global seq
    if form==None:
        return "Convertedbag.csv"
    ret = form.replace('%t', topic_name.replace('/','-'))
    ret=ret[1:]
    return ret
 
def bag_to_csv(mode, output_dir,fname):
    stime = None
    etime = None
    try:
        bag = rosbag.Bag(fname)
        streamdict= dict()
    except Exception as e:
        print('failed to load bag file: %s', e)
        exit(1)

    try:
        for topic, msg, time in bag.read_messages(topics=get_topics(mode,bag),
                                                  start_time=stime,
                                                  end_time=etime):
            breakpoint=0            
            if streamdict.has_key(topic):
                stream = streamdict[topic]
            else:
                file = topic.replace("/","_")
                v = []
                if(file[0] == "_"):
                    v = file[1:]
                else:
                    v = file
                stream = open(output_dir+"/" + v + ".csv",'w')
                streamdict[topic] = stream
                stream.write('datetime')
                message_type_to_csv(stream, msg)
                stream.write('\n')

            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(breakpoint,stream, msg, flatten=False)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        print("fail: %s", e)
    finally:
        bag.close()

def convert(bag_directory,mode,output_dir):
    start = time.time()
    for f in os.listdir(bag_directory):
        
        if ".bag" in f:
            print "Converting: " + f
            folder_name = f[4:f.find(".bag")]
            if(os.path.isdir(output_dir + folder_name + "_CSV") == False):
                os.mkdir(output_dir + folder_name + "_CSV")
            shutil.copy(bag_directory + f,output_dir + folder_name + '_CSV/' + folder_name + ".bag")
            bag_to_csv(mode,output_dir + folder_name+ "_CSV",output_dir + folder_name + '_CSV/' + folder_name + ".bag")
            subprocess.call("rosbag info " + output_dir + folder_name + "_CSV" + "/" + folder_name + ".bag >> " + output_dir + folder_name + "_CSV/bag_info.txt",shell=True)
    print "Conversion of " + bag_directory + " Complete."
    
def main(options):
    convert(options.bag_directory,options.mode,options.output_dir)

if __name__ == '__main__':
    print "Extract Data!!"
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-d", "--bag directory",dest="bag_directory",
            help="bagfile directory")
    parser.add_option("-m", "--mode",dest="mode",
            help="Mode: Supported Options: all,pose,performance")
    parser.add_option("-o", "--output dir",dest="output_dir",
            help="output directory")
    (options, args) = parser.parse_args()


    main(options)
