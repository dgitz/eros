from .Helpers import *
from os import makedirs
def bag2csv(fileName,topicName,topicType,output_dir):
    print(CGREEN + "Converting: " + fileName + " with Topic: " + topicName + CEND)
    outFileName = output_dir + topicName[1:]+".csv"
    outFileDirectory = os.path.dirname(outFileName)
    if os.path.exists(outFileDirectory) == False:
        os.makedirs(outFileDirectory)
    f = open(outFileName,'a+')
    f.close()