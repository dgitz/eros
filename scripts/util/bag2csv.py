from .Helpers import *
from os import makedirs
import rosbag
def convert_logs(bag_dir,output_dir,topic_type_list):
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
        # Get Topic Names of Interest
        bag = rosbag.Bag(fileName)
        keys = list(bag.get_type_and_topic_info()[1].keys())
        values = list(bag.get_type_and_topic_info()[1].values())
        for i in range(0,len(keys)):
            topicName = keys[i]
            topicType = values[i].msg_type
            
            if any(topicType in x for x in topic_type_list):
                bag2csv(fileName,topicName,topicType,output_dir)
                
def bag2csv(fileName,topicName,topicType,output_dir):
    print(CGREEN + "Converting: " + fileName + " with Topic: " + topicName + CEND)
    outFileName = output_dir + topicName[1:]+".csv"
    outFileDirectory = os.path.dirname(outFileName)
    if os.path.exists(outFileDirectory) == False:
        os.makedirs(outFileDirectory)
    bag = rosbag.Bag(fileName)
    if os.path.isfile(outFileName) == False:
        f = open(outFileName,'a+')
        outstr = ""
    
        
        for topic, msg,time in bag.read_messages(topics=topicName):
            for field in type(msg).__slots__:
                outstr += field + ","
            outstr += "\n"
            f.write(outstr)
            break
        f.close()
    f = open(outFileName,'a+')
    for topic, msg, time in bag.read_messages(topics=topicName):
        outstr = ""
        for s in type(msg).__slots__:
            
            val = msg.__getattribute__(s)
            try: 
                if len(val.__slots__) == 2:
                    if val.__slots__[0] == 'secs' and val.__slots__[1] == 'nsecs':
                        outstr += str(val.to_sec()) + ","
            except:
                #pdb.set_trace()
                if type(val).__name__ == "tuple":
                    for field in range(val.__len__()):
                        outstr += str(val[field]) + ","
                else:
                    outstr += val.__str__() + "," 
        outstr += "\n"
        f.write(outstr)

    f.close()