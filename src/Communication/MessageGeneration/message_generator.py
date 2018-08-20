#!/usr/bin/python
import xml.etree.ElementTree as ET
import os.path
import sys
from datetime import datetime
from array import *
from inspect import currentframe, getframeinfo
import pdb
from shutil import copy2

class fieldobject(object):
    def __init__(self,datatype=None,name=None):
        self.datatype=datatype
        self.name=name
class protocolobject(object):
    def __init__(self,name=None):
        self.name=name

def print_usage():
    print "Usage Instructions: message_generator.py"
    print "No Options: This Menu."
    print "-?/-h This Menu."
    print "Generate Message: -g <MessageFile.xml>"

def generate_message(xmlfile):
    print "Generating Message files from:",xmlfile
    if(os.path.isfile(xmlfile) == False):
        print "Cannot find file: ",xmlfile, " Exiting."
        sys.exit(0)
    tree = ET.parse(xmlfile)
    root = tree.getroot()

    ros_udpmessagefile_header.write('#ifndef UDPMESSAGE_H\r\n#define UDPMESSAGE_H\r\n')
    ros_udpmessagefile_header.write('#include "ros/ros.h"\r\n#include "Definitions.h"\r\n#include "ros/time.h"\r\n#include <stdio.h>\r\n')
    ros_udpmessagefile_header.write('#include <iostream>\r\n#include <ctime>\r\n#include <fstream>\r\n#include <iostream>\r\n\r\n')
    gui_udpmessagefile_header.write('#ifndef UDPMESSAGE_H\r\n#define UDPMESSAGE_H\r\n')
    gui_udpmessagefile_header.write('#include <QString>\r\n#include <QList>\r\n')
    
    ros_serialmessagefile_header.write('#ifndef SERIALMESSAGE_H\r\n#define SERIALMESSAGE_H\r\n')
    arduino_serialmessagefile_header.write('#ifndef SERIALMESSAGE_H\r\n#define SERIALMESSAGE_H\r\n')
    propeller_serialmessagefile_header.write('#ifndef SERIALMESSAGE_H\r\n#define SERIALMESSAGE_H\r\n')

    arduino_spimessagefile_header.write('#ifndef SPIMESSAGE_H\r\n#define SPIMESSAGE_H\r\n')
    arduino_spimessagefile_cpp.write('#include "spimessage.h"\r\n')
    ros_spimessagefile_header.write('#ifndef SPIMESSAGE_H\r\n#define SPIMESSAGE_H\r\n')
    ros_spimessagefile_header.write('#include "ros/ros.h"\r\n#include "Definitions.h"\r\n#include "ros/time.h"\r\n#include <stdio.h>\r\n')
    ros_spimessagefile_header.write('#include <iostream>\r\n#include <ctime>\r\n#include <fstream>\r\n#include <iostream>\r\n\r\n')
    arduino_spimessagefile_header.write('#define BYTE2_OFFSET 32768\r\n')
    ros_spimessagefile_header.write('#define BYTE2_OFFSET 32768\r\n')

    arduino_i2cmessagefile_header.write('#ifndef I2CMESSAGE_H\r\n#define I2CMESSAGE_H\r\n')
    arduino_i2cmessagefile_cpp.write('#include "i2cmessage.h"\r\n')
    ros_i2cmessagefile_header.write('#ifndef I2CMESSAGE_H\r\n#define I2CMESSAGE_H\r\n')
    ros_i2cmessagefile_header.write('#include "ros/ros.h"\r\n#include "Definitions.h"\r\n#include "ros/time.h"\r\n#include <stdio.h>\r\n')
    ros_i2cmessagefile_header.write('#include <iostream>\r\n#include <ctime>\r\n#include <fstream>\r\n#include <iostream>\r\n\r\n')


    IDs = []
    for message in root:
        IDs.append(hex(int(message.get('id'),0)).upper())
        for i in range(len(IDs)-1):
            if(IDs[i] == hex(int(message.get('id'),0)).upper()):
                print "ERROR: Duplicate Message Defined: " + IDs[i]
                exit()
    
    for message in root:
        tempstr = "#define " + message.get('name').upper() + "_ID " +  hex(int(message.get('id'),0)).upper() + "\r\n"
        message_strings.append(tempstr)
        protocollist = []
        protocols = message.find('Protocols')
        for protocol in protocols:
            #print "Protocol: ", protocol.get('name')
            protocollist.append(protocolobject(protocol.get('name')))
            
            if(protocol.get('name') == 'UDP'):
                gui_udpmessagefile_header.write('#define UDP_' + message.get('name') + '_ID "' + message.get('id')[2:] +'"\r\n')
            if(protocol.get('name') == 'Serial'):
                ros_serialmessagefile_header.write('#define SERIAL_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
                arduino_serialmessagefile_header.write('#define SERIAL_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
                propeller_serialmessagefile_header.write('#define SERIAL_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
            if(protocol.get('name') == 'SPI'):
                arduino_spimessagefile_header.write('#define SPI_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
            if(protocol.get('name') == 'I2C'):
                arduino_i2cmessagefile_header.write('#define I2C_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
    ros_udpmessagefile_header.write('\r\nclass UDPMessageHandler\r\n{\r\npublic:\r\n\tenum MessageID\r\n\t{\r\n')
    ros_spimessagefile_header.write('\r\nclass SPIMessageHandler\r\n{\r\npublic:\r\n\tenum MessageID\r\n\t{\r\n')
    ros_i2cmessagefile_header.write('\r\nclass I2CMessageHandler\r\n{\r\npublic:\r\n\tenum MessageID\r\n\t{\r\n')
    for message in root:
        protocollist = []
        protocols = message.find('Protocols')
        for protocol in protocols:
            if(protocol.get('name') == 'UDP'):
                ros_udpmessagefile_header.write('\t\tUDP_' + message.get('name') + '_ID = ' + message.get('id') +',\r\n')
            if(protocol.get('name') == 'SPI'):
                ros_spimessagefile_header.write('\t\tSPI_' + message.get('name') + '_ID = ' +hex(int(message.get('id'),0)-int('0xAB00',0)) +',\r\n')
            if(protocol.get('name') == 'I2C'):
                ros_i2cmessagefile_header.write('\t\tI2C_' + message.get('name') + '_ID = ' +hex(int(message.get('id'),0)-int('0xAB00',0)) +',\r\n')
    ros_udpmessagefile_header.write('\t};\r\n\tUDPMessageHandler();\r\n\t~UDPMessageHandler();\r\n')
    ros_udpmessagefile_cpp.write('#include "../include/udpmessage.h"\r\nUDPMessageHandler::UDPMessageHandler(){}\r\nUDPMessageHandler::~UDPMessageHandler(){}\r\n')
    gui_udpmessagefile_header.write('\r\nclass UDPMessageHandler\r\n{\r\npublic:\r\n\tUDPMessageHandler();\r\n\t~UDPMessageHandler();\r\n')
    gui_udpmessagefile_cpp.write('#include "udpmessage.h"\r\nUDPMessageHandler::UDPMessageHandler(){}\r\nUDPMessageHandler::~UDPMessageHandler(){}\r\n')

    ros_serialmessagefile_header.write('\r\nclass SerialMessageHandler\r\n{\r\npublic:\r\n\tSerialMessageHandler();\r\n\t~SerialMessageHandler();\r\n')
    ros_serialmessagefile_cpp.write('#include "../include/serialmessage.h"\r\nSerialMessageHandler::SerialMessageHandler(){}\r\nSerialMessageHandler::~SerialMessageHandler(){}\r\n')
    arduino_serialmessagefile_header.write('\r\nclass SerialMessageHandler\r\n{\r\npublic:\r\n\tSerialMessageHandler();\r\n\t~SerialMessageHandler();\r\n')
    arduino_serialmessagefile_cpp.write('#include "serialmessage.h"\r\nSerialMessageHandler::SerialMessageHandler(){}\r\nSerialMessageHandler::~SerialMessageHandler(){}\r\n')
    propeller_serialmessagefile_cpp.write('#include "serialmessage.h"\r\n')

    ros_spimessagefile_header.write('\t};\r\n\tSPIMessageHandler();\r\n\t~SPIMessageHandler();\r\n')
    ros_spimessagefile_cpp.write('#include "../include/spimessage.h"\r\nSPIMessageHandler::SPIMessageHandler(){}\r\nSPIMessageHandler::~SPIMessageHandler(){}\r\n')

    ros_i2cmessagefile_header.write('\t};\r\n\tI2CMessageHandler();\r\n\t~I2CMessageHandler();\r\n')
    ros_i2cmessagefile_cpp.write('#include "../include/i2cmessage.h"\r\nI2CMessageHandler::I2CMessageHandler(){}\r\nI2CMessageHandler::~I2CMessageHandler(){}\r\n')
    for message in root:
        protocollist = []
        protocols = message.find('Protocols')
        for protocol in protocols:
            fieldlist = []
            fields = protocol.find('Fields')
            #print fields
            for field in fields:
                #print "Field date Type: ", field.get('type'), " name: ", field.get('name')
                fieldlist.append(fieldobject(field.get('type'),field.get('name')))
           
            if(protocol.get('name') == 'UDP'):
                encode_for_master = 0
                encode_for_gui = 0
                decode_for_master = 0
                decode_for_gui = 0
                for child in protocol.findall('Origin'):
                    if(child.text == "Master"):
                        encode_for_master = 1
                        decode_for_gui = 1
                    if(child.text == "GUI"):
                        decode_for_master = 1
                        encode_for_gui = 1
                if(encode_for_master == 1):
                    ros_udpmessagefile_header.write('\tstd::string encode_' + message.get('name') + 'UDP(')
                    ros_udpmessagefile_cpp.write('std::string UDPMessageHandler::encode_' + message.get('name') + 'UDP(')
            
                if(encode_for_gui == 1):
                    gui_udpmessagefile_header.write('\tQString encode_' + message.get('name') + 'UDP(')
                    gui_udpmessagefile_cpp.write('QString UDPMessageHandler::encode_' + message.get('name') + 'UDP(')
                index = 0
                for item in fieldlist:
                    if(encode_for_master == 1):
                        ros_udpmessagefile_header.write(item.datatype + ' ' + item.name)
                        ros_udpmessagefile_cpp.write(item.datatype + ' ' + item.name)
            
                    if(encode_for_gui == 1):
                        if(item.datatype =='std::string'):
                            gui_udpmessagefile_header.write(item.datatype + ' ' + item.name)
                            gui_udpmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'uint64_t'):
                            gui_udpmessagefile_header.write(item.datatype + ' ' + item.name)
                            gui_udpmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'int16_t'):
                            gui_udpmessagefile_header.write('int ' + item.name)
                            gui_udpmessagefile_cpp.write('int ' + item.name)
                        elif(item.datatype == 'uint8_t'):
                            gui_udpmessagefile_header.write('int ' + item.name)
                            gui_udpmessagefile_cpp.write('int ' + item.name)
                        elif(item.datatype == 'double'):
                            gui_udpmessagefile_header.write('double ' + item.name)
                            gui_udpmessagefile_cpp.write('double ' + item.name)
                        else:
                             print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    index += 1
                    if(index < len(fieldlist)):
                        if(encode_for_master == 1):
                            ros_udpmessagefile_header.write(',')
                            ros_udpmessagefile_cpp.write(',')
                        
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_header.write(',')
                            gui_udpmessagefile_cpp.write(',')

                if(encode_for_master == 1):    
                    ros_udpmessagefile_header.write(');\r\n')
                    ros_udpmessagefile_cpp.write(')\r\n{\r\n')
                    ros_udpmessagefile_cpp.write('\tstd::string tempstr = "";\r\n')
                    ros_udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>(UDP_' + message.get('name') + '_ID));\r\n')
                    ros_udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')

                if(encode_for_gui == 1):
                    gui_udpmessagefile_header.write(');\r\n')
                    gui_udpmessagefile_cpp.write(')\r\n{\r\n')
                    gui_udpmessagefile_cpp.write('\tQString tempstr = "";\r\n')
                    gui_udpmessagefile_cpp.write('\ttempstr.append(UDP_' + message.get('name') + '_ID);\r\n')
                    gui_udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')
                index = 0
                for item in fieldlist:
                    if(item.datatype =='std::string'):
                        if(encode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\ttempstr.append(' + item.name +');\r\n')
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\ttempstr.append(QString::fromStdString(' + item.name +'));\r\n')
                    elif(item.datatype == 'int16_t'):
                        if(encode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>((int)' + item.name + '));\r\n')
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\ttempstr.append(QString::number(' + item.name + '));\r\n')
                    elif(item.datatype == 'uint64_t'):
                        if(encode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>((uint64_t)' + item.name + '));\r\n')
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\ttempstr.append(QString::number(' + item.name + '));\r\n')
                    elif(item.datatype == 'uint8_t'):
                        if(encode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>((int)' + item.name + '));\r\n')
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\ttempstr.append(QString::number(' + item.name + '));\r\n')
                    else:
                        if(encode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>((int)' + item.name + '));\r\n')
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\ttempstr.append(QString::number(' + item.name + '));\r\n')

                    index += 1

                    if(index < len(fieldlist)):
                        if(encode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')
                        if(encode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')
                if(encode_for_master == 1):
                    ros_udpmessagefile_cpp.write('\treturn tempstr;\r\n')
                    ros_udpmessagefile_cpp.write('}\r\n')
                if(encode_for_gui == 1):
                    gui_udpmessagefile_cpp.write('\treturn tempstr;\r\n')
                    gui_udpmessagefile_cpp.write('}\r\n')

                if(decode_for_master == 1):
                    ros_udpmessagefile_header.write('\tint decode_' + message.get('name') + 'UDP(std::vector<std::string> items,')
                    ros_udpmessagefile_cpp.write('int UDPMessageHandler::decode_' + message.get('name') + 'UDP(std::vector<std::string> items,')
                if(decode_for_gui == 1):
                    gui_udpmessagefile_header.write('\tint decode_' + message.get('name') + 'UDP(QList<QByteArray> items,')
                    gui_udpmessagefile_cpp.write('int UDPMessageHandler::decode_' + message.get('name') + 'UDP(QList<QByteArray> items,')
                index = 0
                itemcounter = 1
                for item in fieldlist:
                    if(decode_for_master == 1):
                        if(item.datatype == 'uint8_t'):
                            ros_udpmessagefile_header.write(item.datatype + '* ' + item.name)
                            ros_udpmessagefile_cpp.write(item.datatype + '* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == 'int16_t'):
                            ros_udpmessagefile_header.write('int* ' + item.name)
                            ros_udpmessagefile_cpp.write('int* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == 'uint64_t'):
                            ros_udpmessagefile_header.write('uint64_t* ' + item.name)
                            ros_udpmessagefile_cpp.write('uint64_t* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == "std::string"):
                            ros_udpmessagefile_header.write('std::string* ' + item.name)
                            ros_udpmessagefile_cpp.write('std::string* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == "double"):
                            ros_udpmessagefile_header.write('double* ' + item.name)
                            ros_udpmessagefile_cpp.write('double* ' + item.name)
                            itemcounter = itemcounter + 1
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    if(decode_for_gui == 1):
                        if(item.datatype == 'uint8_t'):
                            gui_udpmessagefile_header.write('int* ' + item.name)
                            gui_udpmessagefile_cpp.write('int* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == 'uint16_t'):
                            gui_udpmessagefile_header.write('int* ' + item.name)
                            gui_udpmessagefile_cpp.write('int* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == 'int16_t'):
                            gui_udpmessagefile_header.write('int* ' + item.name)
                            gui_udpmessagefile_cpp.write('int* ' + item.name)
                            itemcounter = itemcounter + 1
                        elif(item.datatype == 'std::string'):
                            gui_udpmessagefile_header.write('std::string* ' + item.name)
                            gui_udpmessagefile_cpp.write('std::string* ' + item.name)
                            itemcounter = itemcounter + 1
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                        
                    index += 1
                    if(index < len(fieldlist)):
                        if(decode_for_master == 1):
                            ros_udpmessagefile_header.write(',')
                            ros_udpmessagefile_cpp.write(',')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_header.write(',')
                            gui_udpmessagefile_cpp.write(',')
                if(decode_for_master == 1):
                    ros_udpmessagefile_header.write(');\r\n')
                    ros_udpmessagefile_cpp.write(')\r\n{\r\n')
                if(decode_for_gui == 1):
                    gui_udpmessagefile_header.write(');\r\n')
                    gui_udpmessagefile_cpp.write(')\r\n{\r\n')
                if(decode_for_master == 1):
                    ros_udpmessagefile_cpp.write('\tchar tempstr[8];\r\n\tsprintf(tempstr,"0x%s",items.at(0).c_str());\r\n\tint id = (int)strtol(tempstr,NULL,0);\r\n')
                    ros_udpmessagefile_cpp.write('\tif(id != UDP_' + message.get('name') + '_ID){ return 0; }\r\n')
                    ros_udpmessagefile_cpp.write('\tif(items.size() != ' + str(itemcounter) + '){ return 0; }\r\n')
                if(decode_for_gui == 1):
                    #print "ID:" + message.get('name') + str(itemcounter)
                    gui_udpmessagefile_cpp.write('\tif(items.size() != ' + str(itemcounter) + '){ return 0; }\r\n')
                    #ros_udpmessagefile_cpp.write('\tif(std::stoi(items.at(0)) != UDP_' + message.get('name') + '_ID) { return 0; }\r\n')
                itemcounter = 0
                for item in fieldlist:
                    if(item.datatype == 'uint8_t'):
                        itemcounter = itemcounter + 1
                        if(decode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(' + item.datatype + ')atoi(items.at(' + str(itemcounter) + ').c_str());\r\n')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(int)items.at(' + str(itemcounter) + ').toInt();\r\n')
                    elif(item.datatype == 'int16_t'):
                        itemcounter = itemcounter + 1
                        if(decode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(' + item.datatype + ')atoi(items.at(' + str(itemcounter) + ').c_str());\r\n')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(int)items.at(' + str(itemcounter) + ').toInt();\r\n')
                    elif(item.datatype == 'uint16_t'):
                        itemcounter = itemcounter + 1
                        if(decode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(' + item.datatype + ')atoi(items.at(' + str(itemcounter) + ').c_str());\r\n')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(int)items.at(' + str(itemcounter) + ').toInt();\r\n')
                    elif(item.datatype == 'uint64_t'):
                        itemcounter = itemcounter + 1
                        if(decode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(' + item.datatype + ')strtoull(items.at(' + str(itemcounter) + ').c_str(),NULL,10);\r\n')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(int)items.at(' + str(itemcounter) + ').toInt();\r\n')
                    elif(item.datatype == 'std::string'):
                        itemcounter = itemcounter + 1
                        if(decode_for_master == 1):
                            ros_udpmessagefile_cpp.write('\t*' + str(item.name) + '=items.at(' + str(itemcounter) + ');\r\n')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\t*' + str(item.name) + '=items.at(' + str(itemcounter) + ').toStdString();\r\n')
                    elif(item.datatype == 'double'):
                        itemcounter = itemcounter + 1
                        if(decode_for_master == 1):
                             ros_udpmessagefile_cpp.write('\t*' + str(item.name) + '=(' + item.datatype + ')atof(items.at(' + str(itemcounter) + ').c_str());\r\n')
                        if(decode_for_gui == 1):
                            gui_udpmessagefile_cpp.write('\t*' + str(item.name) + '=items.at(' + str(itemcounter) + ').toDouble();\r\n')
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    
                if(decode_for_master == 1):
                    ros_udpmessagefile_cpp.write('\treturn 1;\r\n')
                    ros_udpmessagefile_cpp.write('}\r\n')
                if(decode_for_gui == 1):
                    gui_udpmessagefile_cpp.write('\treturn 1;\r\n')
                    gui_udpmessagefile_cpp.write('}\r\n')
            elif(protocol.get('name') == 'Serial'):
                encode_for_master = 0
                encode_for_slave = 0
                decode_for_master = 0
                decode_for_slave = 0
                for child in protocol.findall('Origin'):
                    if(child.text == "Master"):
                        encode_for_master = 1
                        decode_for_slave = 1
                    if(child.text == "Slave"):
                        decode_for_master = 1
                        encode_for_slave = 1
                #print message.get('name'), " Encoding/Decoding for Master: ", encode_for_master, ",", decode_for_master, " Encoding/Decoding for Slave: ", encode_for_slave, ",", decode_for_slave 
                if(encode_for_master == 1):                
                    ros_serialmessagefile_header.write('\tint encode_' + message.get('name') + 'Serial(unsigned char* outbuffer,int* length,')
                    ros_serialmessagefile_cpp.write('int SerialMessageHandler::encode_' + message.get('name') + 'Serial(unsigned char* outbuffer,int* length,')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_header.write('int encode_' + message.get('name') + 'Serial(int* outbuffer,int* length,')
                    propeller_serialmessagefile_cpp.write('int encode_' + message.get('name') + 'Serial(int* outbuffer,int* length,')
                    arduino_serialmessagefile_header.write('\tint encode_' + message.get('name') + 'Serial(unsigned char* outbuffer,int* length,')
                    arduino_serialmessagefile_cpp.write('int SerialMessageHandler::encode_' + message.get('name') + 'Serial(unsigned char* outbuffer,int* length,')
                index = 0
                for item in fieldlist:
                    if(encode_for_master == 1):
                        if(item.datatype == 'char'):
                            ros_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            ros_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)               
                        elif(item.datatype == 'unsigned char'):
                            ros_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            ros_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            ros_serialmessagefile_header.write('int ' + item.name)
                            ros_serialmessagefile_cpp.write('int ' + item.name)
                        elif(item.datatype == 'int16_t'):
                            ros_serialmessagefile_header.write('int ' + item.name)
                            ros_serialmessagefile_cpp.write('int ' + item.name)  
                        elif(item.datatype == 'uint32_t'):
                            ros_serialmessagefile_header.write('unsigned long ' + item.name)
                            ros_serialmessagefile_cpp.write('unsigned long ' + item.name)
                        elif(item.datatype == 'int32_t'):
                            ros_serialmessagefile_header.write('long ' + item.name)
                            ros_serialmessagefile_cpp.write('long ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    if(encode_for_slave == 1):
                        if(item.datatype == 'char'):
                            propeller_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            propeller_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                            arduino_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            arduino_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'unsigned char'):
                            propeller_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            propeller_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                            arduino_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            arduino_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            propeller_serialmessagefile_header.write('int ' + item.name)
                            propeller_serialmessagefile_cpp.write('int ' + item.name)
                            arduino_serialmessagefile_header.write('int ' + item.name)
                            arduino_serialmessagefile_cpp.write('int ' + item.name)
                        elif(item.datatype == 'int16_t'):
                            propeller_serialmessagefile_header.write('int ' + item.name)
                            propeller_serialmessagefile_cpp.write('int ' + item.name)
                            arduino_serialmessagefile_header.write('int ' + item.name)
                            arduino_serialmessagefile_cpp.write('int ' + item.name)
                        elif(item.datatype == 'uint32_t'):
                            propeller_serialmessagefile_header.write('unsigned long ' + item.name)
                            propeller_serialmessagefile_cpp.write('unsigned long ' + item.name)
                            arduino_serialmessagefile_header.write('unsigned long ' + item.name)
                            arduino_serialmessagefile_cpp.write('unsigned long ' + item.name)
                        elif(item.datatype == 'int32_t'):
                            propeller_serialmessagefile_header.write('long ' + item.name)
                            propeller_serialmessagefile_cpp.write('long ' + item.name)
                            arduino_serialmessagefile_header.write('long ' + item.name)
                            arduino_serialmessagefile_cpp.write('long ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    index += 1
                    if(index < len(fieldlist)):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_header.write(',')
                            ros_serialmessagefile_cpp.write(',')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_header.write(',')
                            propeller_serialmessagefile_cpp.write(',')
                            arduino_serialmessagefile_header.write(',')
                            arduino_serialmessagefile_cpp.write(',')
                if(encode_for_master == 1):
                    ros_serialmessagefile_header.write(');\r\n')
                    ros_serialmessagefile_cpp.write(')\r\n{\r\n')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_header.write(');\r\n')
                    propeller_serialmessagefile_cpp.write(')\r\n{\r\n')
                    arduino_serialmessagefile_header.write(');\r\n')
                    arduino_serialmessagefile_cpp.write(')\r\n{\r\n')
                message_id = hex(int(message.get('id'),0)-int('0xAB00',0))
                bytelength = 0
                for item in fieldlist:
                    if(item.datatype == 'char'):
                        bytelength = bytelength +1
                    elif(item.datatype == 'unsigned char'):
                        bytelength = bytelength +1
                    elif(item.datatype == 'uint16_t'):
                        bytelength = bytelength +2
                    elif(item.datatype == "int16_t"):
                        bytelength = bytelength +2
                    elif(item.datatype == "uint32_t"):
                        bytelength = bytelength + 4
                    elif(item.datatype == "int32_t"):
                        bytelength = bytelength + 4
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                #if(bytelength > 12): print "ERROR ERROR ERROR: Currently Serial Messages longer than 12 bytes (",str(bytelength), " defined) are not supported.", " at line: ",currentframe().f_lineno
                if(encode_for_master == 1):
                    ros_serialmessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 0xAB;\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + message_id +';\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + str(bytelength) + ';\r\n')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_cpp.write('\tint byte_counter=0;\r\n')
                    propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = 0xAB;\r\n')
                    propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = ' + message_id +';\r\n')
                    propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = ' + str(bytelength) + ';\r\n')
                    arduino_serialmessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                    arduino_serialmessagefile_cpp.write('\t*p_outbuffer++ = 0xAB;\r\n')
                    arduino_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + message_id +';\r\n')
                    arduino_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + str(bytelength) + ';\r\n')
                for item in fieldlist:
                    if(item.datatype == 'char'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = ' + item.name +';\r\n')
                            arduino_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                    elif(item.datatype == 'unsigned char'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = ' + item.name +';\r\n')
                            arduino_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                    elif(item.datatype == 'uint16_t'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 8;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = (' + item.name + ' & 0xFF);\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '1 = ' + item.name + ' >> 8;\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'1;\r\n')
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '2 = ' + item.name + ' -(v_'  + item.name + '1 << 8);\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'2;\r\n')
                            
                            arduino_serialmessagefile_cpp.write('\tfor(int i = 2; i>0;i--)\r\n\t{\r\n')
                            arduino_serialmessagefile_cpp.write('\t\t*p_outbuffer++ = ((' + item.name + ' >> 8*(i-1)) & 0xFF);\r\n\t}\r\n')
                    elif(item.datatype == 'int16_t'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '1 = ' + item.name + ' >> 8;\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'1;\r\n')
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '2 = ' + item.name + ' -(v_'  + item.name + '1 << 8);\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'2;\r\n')
                            arduino_serialmessagefile_cpp.write('\tfor(int i = 2; i>0;i--)\r\n\t{\r\n')
                            arduino_serialmessagefile_cpp.write('\t\t*p_outbuffer++ = ((' + item.name + ' >> 8*(i-1)) & 0xFF);\r\n\t}\r\n')
                    elif(item.datatype == 'uint32_t'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 24;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 16;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 8;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = (' + item.name + ' & 0xFF);\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '1 = ' + item.name + ' >> 8;\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'1;\r\n')
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '2 = ' + item.name + ' -(v_'  + item.name + '1 << 8);\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'2;\r\n')
                            arduino_serialmessagefile_cpp.write('\tfor(int i = 4; i>0;i--)\r\n\t{\r\n')
                            arduino_serialmessagefile_cpp.write('\t\t*p_outbuffer++ = ((' + item.name + ' >> 8*(i-1)) & 0xFF);\r\n\t}\r\n')
                    elif(item.datatype == 'int32_t'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 24;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 16;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ' >> 8;\r\n')
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = (' + item.name + ' & 0xFF);\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '1 = ' + item.name + ' >> 8;\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'1;\r\n')
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '2 = ' + item.name + ' -(v_'  + item.name + '1 << 8);\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'2;\r\n')
                            arduino_serialmessagefile_cpp.write('\tfor(int i = 4; i>0;i--)\r\n\t{\r\n')
                            arduino_serialmessagefile_cpp.write('\t\t*p_outbuffer++ = ((' + item.name + ' >> 8*(i-1)) & 0xFF);\r\n\t}\r\n')
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                #for b in range(bytelength,12):
                #    if(encode_for_master == 1):
                #        ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 0;\r\n')
                #    if(encode_for_slave == 1):
                #        propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = 0;\r\n') 
                if(encode_for_master == 1):
                    ros_serialmessagefile_cpp.write('\tint checksum = 0;\r\n')
                    ros_serialmessagefile_cpp.write('\tfor(int i = 3; i < (3+' + str(bytelength) + ');i++)\r\n\t{\r\n')
                    ros_serialmessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    ros_serialmessagefile_cpp.write('\t}\r\n\t*p_outbuffer++ = checksum;\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 10;\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 13;\r\n')
                    ros_serialmessagefile_cpp.write('\t*length = p_outbuffer-&outbuffer[0];\r\n')
                    ros_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    ros_serialmessagefile_cpp.write('}\r\n')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_cpp.write('\tint checksum = 0;\r\n')
                    propeller_serialmessagefile_cpp.write('\tfor(int i = 3; i < (3+' + str(bytelength) + ');i++)\r\n\t{\r\n')
                    propeller_serialmessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    propeller_serialmessagefile_cpp.write('\t}\r\n\toutbuffer[byte_counter] = checksum;\r\n\tlength[0] = 3+' + str(bytelength) + '+1;\r\n')
                    propeller_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    propeller_serialmessagefile_cpp.write('}\r\n')
                    arduino_serialmessagefile_cpp.write('\tint checksum = 0;\r\n')
                    arduino_serialmessagefile_cpp.write('\tfor(int i = 3; i < (3+' + str(bytelength) + ');i++)\r\n\t{\r\n')
                    arduino_serialmessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    arduino_serialmessagefile_cpp.write('\t}\r\n\t*p_outbuffer++ = checksum;\r\n\t*length = p_outbuffer-&outbuffer[0];\r\n')
                    arduino_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    arduino_serialmessagefile_cpp.write('}\r\n')
                
                if(decode_for_master == 1):
                    ros_serialmessagefile_header.write('\tint decode_' + message.get('name') + 'Serial(unsigned char* inpacket,')
                    ros_serialmessagefile_cpp.write('int SerialMessageHandler::decode_' + message.get('name') + 'Serial(unsigned char* inpacket,')
                if(decode_for_slave == 1):
                    propeller_serialmessagefile_header.write('int decode_' + message.get('name') + 'Serial(int* inpacket,int length,int checksum,')
                    propeller_serialmessagefile_cpp.write('int decode_' + message.get('name') + 'Serial(int* inpacket,int length,int checksum,')
                    arduino_serialmessagefile_header.write('\tint decode_' + message.get('name') + 'Serial(unsigned char* inpacket,')
                    arduino_serialmessagefile_cpp.write('int SerialMessageHandler::decode_' + message.get('name') + 'Serial(unsigned char* inpacket,')
                index = 0
                for item in fieldlist:
                    if(decode_for_master == 1):
                        if(item.datatype == 'char'):
                            ros_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            ros_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'unsigned char'):
                            ros_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            ros_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            ros_serialmessagefile_header.write('int* ' + item.name)
                            ros_serialmessagefile_cpp.write('int* ' + item.name)
                        elif(item.datatype == 'int16_t'):
                            ros_serialmessagefile_header.write('int* ' + item.name)
                            ros_serialmessagefile_cpp.write('int* ' + item.name)
                        elif(item.datatype == 'uint32_t'):
                            ros_serialmessagefile_header.write('unsigned long* ' + item.name)
                            ros_serialmessagefile_cpp.write('unsigned long* ' + item.name)
                        elif(item.datatype == 'int32_t'):
                            ros_serialmessagefile_header.write('long* ' + item.name)
                            ros_serialmessagefile_cpp.write('long* ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    if(decode_for_slave == 1):
                        if(item.datatype == 'char'):
                            propeller_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            propeller_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                            arduino_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            arduino_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'unsigned char'):
                            propeller_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            propeller_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                            arduino_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            arduino_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            propeller_serialmessagefile_header.write('int* ' + item.name)
                            propeller_serialmessagefile_cpp.write('int* ' + item.name)
                            arduino_serialmessagefile_header.write('int* ' + item.name)
                            arduino_serialmessagefile_cpp.write('int* ' + item.name)
                        elif(item.datatype == 'int16_t'):
                            propeller_serialmessagefile_header.write('int* ' + item.name)
                            propeller_serialmessagefile_cpp.write('int* ' + item.name)
                            arduino_serialmessagefile_header.write('int* ' + item.name)
                            arduino_serialmessagefile_cpp.write('int* ' + item.name)
                        elif(item.datatype == 'uint32_t'):
                            propeller_serialmessagefile_header.write('unsigned long* ' + item.name)
                            propeller_serialmessagefile_cpp.write('unsigned long* ' + item.name)
                            arduino_serialmessagefile_header.write('unsigned long* ' + item.name)
                            arduino_serialmessagefile_cpp.write('unsigned long* ' + item.name)
                        elif(item.datatype == 'int32_t'):
                            propeller_serialmessagefile_header.write('long* ' + item.name)
                            propeller_serialmessagefile_cpp.write('long* ' + item.name)
                            arduino_serialmessagefile_header.write('long* ' + item.name)
                            arduino_serialmessagefile_cpp.write('long* ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    index += 1
                    if(index < len(fieldlist)):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_header.write(',')
                            ros_serialmessagefile_cpp.write(',')
                        if(decode_for_slave == 1):
                            propeller_serialmessagefile_header.write(',')
                            propeller_serialmessagefile_cpp.write(',')
                            arduino_serialmessagefile_header.write(',')
                            arduino_serialmessagefile_cpp.write(',')
                if(decode_for_master == 1):
                    ros_serialmessagefile_header.write(');\r\n')
                    ros_serialmessagefile_cpp.write(')\r\n{\r\n')
                if(decode_for_slave == 1):
                    propeller_serialmessagefile_header.write(');\r\n')
                    propeller_serialmessagefile_cpp.write(')\r\n{\r\n')
                    propeller_serialmessagefile_cpp.write('\tint computed_checksum = 0;\r\n')
                    propeller_serialmessagefile_cpp.write('\tfor(int i = 0; i < length; i++)\r\n\t{\r\n')
                    propeller_serialmessagefile_cpp.write('\t\tcomputed_checksum ^= inpacket[i];\r\n\t}\r\n')
                    propeller_serialmessagefile_cpp.write('\tif(computed_checksum != checksum) { return -1; }\r\n')
                    arduino_serialmessagefile_header.write(');\r\n')
                    arduino_serialmessagefile_cpp.write(')\r\n{\r\n')
                bytecounter = 0
                for item in fieldlist:
                    if(item.datatype == 'char'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '];\r\n')
                        if(decode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '];\r\n')
                            arduino_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '];\r\n')
                        bytecounter = bytecounter + 1
                    elif(item.datatype == 'unsigned char'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '];\r\n')
                        if(decode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '];\r\n')
                            arduino_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '];\r\n')
                        bytecounter = bytecounter + 1
                    elif(item.datatype == 'uint16_t'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1;\r\n')
                            bytecounter = bytecounter + 1
                        elif(decode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter) + ']<<8;\r\n')
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            propeller_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '] + v_' + str(item.name) + '1;\r\n')
                            arduino_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1;\r\n')
                            bytecounter = bytecounter + 1
                    elif(item.datatype == 'int16_t'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1;\r\n')
                            bytecounter = bytecounter + 1
                        elif(decode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter) + ']<<8;\r\n')
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            propeller_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '] + v_' + str(item.name) + '1;\r\n')
                            arduino_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1;\r\n')
                            bytecounter = bytecounter + 1
                    elif(item.datatype == 'uint32_t'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '3=inpacket[' + str(bytecounter+3) + ']<<24;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '2=inpacket[' + str(bytecounter+3) + ']<<16;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1 ' + \
                                        '+ v_' + str(item.name) + '2 + v_' + str(item.name) + '3;\r\n')
                            bytecounter = bytecounter + 1
                        elif(decode_for_slave == 1):
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '3=inpacket[' + str(bytecounter+3) + ']<<24;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '2=inpacket[' + str(bytecounter+3) + ']<<16;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1 ' + \
                                        '+ v_' + str(item.name) + '2 + v_' + str(item.name) + '3;\r\n')
                            bytecounter = bytecounter + 1
                    elif(item.datatype == 'int32_t'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '3=inpacket[' + str(bytecounter+3) + ']<<24;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '2=inpacket[' + str(bytecounter+3) + ']<<16;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1 ' + \
                                        '+ v_' + str(item.name) + '2 + v_' + str(item.name) + '3;\r\n')
                            bytecounter = bytecounter + 1
                        elif(decode_for_slave == 1):
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '3=inpacket[' + str(bytecounter+3) + ']<<24;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '2=inpacket[' + str(bytecounter+3) + ']<<16;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter+3) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter+3) + '] + v_' + str(item.name) + '1 ' + \
                                        '+ v_' + str(item.name) + '2 + v_' + str(item.name) + '3;\r\n')
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                if(decode_for_master == 1):
                    ros_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    ros_serialmessagefile_cpp.write('}\r\n')
                if(decode_for_slave == 1):
                    propeller_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    propeller_serialmessagefile_cpp.write('}\r\n')
                    arduino_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    arduino_serialmessagefile_cpp.write('}\r\n')
            elif(protocol.get('name') == 'SPI'):
                type_query = 0
                type_command = 0
                for child in protocol.findall('Type'):
                    if(child.text == "Query"):
                         type_query = 1
                    if(child.text == "Command"):
                        type_command = 1
                if(type_query == 1):
                    arduino_spimessagefile_header.write('\r\nint encode_' + message.get('name') + 'SPI(unsigned char* outbuffer,int* length,')
                    arduino_spimessagefile_cpp.write('int encode_' + message.get('name') + 'SPI(unsigned char* outbuffer,int* length,')
                    ros_spimessagefile_header.write('\r\n\tint decode_' + message.get('name') + 'SPI(unsigned char* inbuffer,int * length,')
                    ros_spimessagefile_cpp.write('int SPIMessageHandler::decode_' + message.get('name') + 'SPI(unsigned char* inbuffer,int * length,')
                elif(type_command == 1):
                    arduino_spimessagefile_header.write('\r\nint decode_' + message.get('name') + 'SPI(unsigned char* inbuffer,int* length,unsigned char checksum,')
                    arduino_spimessagefile_cpp.write('\r\nint decode_' + message.get('name') + 'SPI(unsigned char* inbuffer,int* length,unsigned char checksum,')
                    ros_spimessagefile_header.write('\r\n\tint encode_' + message.get('name') + 'SPI(unsigned char* outbuffer,int * length,')
                    ros_spimessagefile_cpp.write('int SPIMessageHandler::encode_' + message.get('name') + 'SPI(unsigned char* outbuffer,int * length,')
                index = 0
                for item in fieldlist:
                    if(type_query == 1):
                        if(item.datatype == 'unsigned char'):
                            arduino_spimessagefile_header.write( item.datatype + ' ' + item.name)
                            arduino_spimessagefile_cpp.write(item.datatype + ' ' + item.name)
                            ros_spimessagefile_header.write( item.datatype + '* ' + item.name)
                            ros_spimessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            arduino_spimessagefile_header.write( 'unsigned int ' + item.name)
                            arduino_spimessagefile_cpp.write('unsigned int ' + item.name)
                            ros_spimessagefile_header.write( item.datatype + '* ' + item.name)
                            ros_spimessagefile_cpp.write(item.datatype + '* ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    elif(type_command == 1):
                        if(item.datatype == 'unsigned char'):
                            arduino_spimessagefile_header.write('unsigned char * ' + item.name)
                            arduino_spimessagefile_cpp.write('unsigned char * ' + item.name)
                            ros_spimessagefile_header.write( item.datatype + ' ' + item.name)
                            ros_spimessagefile_cpp.write( item.datatype + ' ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    index += 1
                    if(index < len(fieldlist)):
                        if((type_query == 1) or (type_command == 1)):
                            arduino_spimessagefile_header.write(',')
                            arduino_spimessagefile_cpp.write(',')
                            ros_spimessagefile_header.write(',')
                            ros_spimessagefile_cpp.write(',')
                if((type_query == 1) or (type_command == 1)):
                    arduino_spimessagefile_header.write(');\r\n')
                    arduino_spimessagefile_cpp.write(')\r\n{\r\n')
                    ros_spimessagefile_header.write(');\r\n')
                    ros_spimessagefile_cpp.write(')\r\n{\r\n')
                message_id = hex(int(message.get('id'),0)-int('0xAB00',0))
                bytelength = 0
                for item in fieldlist:
                    if(item.datatype == 'unsigned char'):
                        bytelength = bytelength +1
                    elif(item.datatype == 'uint16_t'):
                        bytelength = bytelength +2
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                if(bytelength > 12): print "ERROR ERROR ERROR: Currently SPI Messages longer than 12 bytes are not supported.", " at line: ",currentframe().f_lineno
                if(type_query == 1):
                    arduino_spimessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                elif(type_command == 1):
                    ros_spimessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                bytecounter = 0
                for item in fieldlist:
                    if(item.datatype == 'unsigned char'): 
                        if(type_query == 1):
                            arduino_spimessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                            ros_spimessagefile_cpp.write('\t*' + item.name + ' = inbuffer[' + str(bytecounter) + '];\r\n')
                            bytecounter = bytecounter + 1
                        elif(type_command == 1):
                            arduino_spimessagefile_cpp.write('\t*' + item.name + ' = inbuffer[' + str(bytecounter) + '];\r\n')
                            ros_spimessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ';\r\n')
                            bytecounter = bytecounter + 1
                    elif(item.datatype == 'uint16_t'): 
                        if(type_query == 1):
                            arduino_spimessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +'>>8;\r\n')
                            ros_spimessagefile_cpp.write('\tint v_' + item.name + ' = inbuffer[' + str(bytecounter) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_spimessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                            ros_spimessagefile_cpp.write('\t*' + item.name + ' = v_' + item.name + ' + inbuffer[' + str(bytecounter) + '];\r\n')
                            bytecounter = bytecounter + 1
                        elif(type_command == 1):
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                for b in range(bytelength,12):
                    if(type_query == 1):
                        arduino_spimessagefile_cpp.write('\t*p_outbuffer++ = 0;\r\n')
                    elif(type_command == 1):
                        ros_spimessagefile_cpp.write('\t*p_outbuffer++ = 0;\r\n') 
                if(type_query == 1):
                    arduino_spimessagefile_cpp.write('\tunsigned char checksum = 0;\r\n')
                    arduino_spimessagefile_cpp.write('\tfor(int i = 0; i < 12;i++)\r\n\t{\r\n')
                    arduino_spimessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    arduino_spimessagefile_cpp.write('\t}\r\n\t*p_outbuffer++ = checksum;\r\n\tlength[0] = 12;\r\n')
                    arduino_spimessagefile_cpp.write('\treturn 1;\r\n')
                    arduino_spimessagefile_cpp.write('}\r\n')
                    ros_spimessagefile_cpp.write('\treturn 1;\r\n')
                    ros_spimessagefile_cpp.write('}\r\n')
                elif(type_command == 1):
                    arduino_spimessagefile_cpp.write('\tunsigned char calc_checksum = SPI_' + message.get('name') + '_ID;\r\n')
                    arduino_spimessagefile_cpp.write('\tfor(int i = 0; i < 12; i++)\r\n\t{\r\n')
                    arduino_spimessagefile_cpp.write('\t\tcalc_checksum ^= inbuffer[i];\r\n\t}\r\n')
                    arduino_spimessagefile_cpp.write('\tif(calc_checksum == checksum)\r\n\t{\r\n')
                    arduino_spimessagefile_cpp.write('\t\treturn 1;\r\n\t}\r\n')
                    arduino_spimessagefile_cpp.write('\telse\r\n\t{\r\n\t\treturn 0;\r\n\t}\r\n}')
                    ros_spimessagefile_cpp.write('\tunsigned char checksum = 0;\r\n')
                    ros_spimessagefile_cpp.write('\tfor(int i = 0; i < 12; i++)\r\n\t{\r\n')
                    ros_spimessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n\t}\r\n')
                    ros_spimessagefile_cpp.write('\t*p_outbuffer++ = checksum;\r\n')
                    ros_spimessagefile_cpp.write('\tlength[0] = 12;\r\n')
                    ros_spimessagefile_cpp.write('\treturn 1;\r\n}')
            elif(protocol.get('name') == 'I2C'):
                type_query = 0
                type_command = 0
                for child in protocol.findall('Type'):
                    if(child.text == "Query"):
                         type_query = 1
                    #if(child.text == "Command"):
                    #    type_command = 1
                if(type_query == 1):
                    arduino_i2cmessagefile_header.write('\r\nint encode_' + message.get('name') + 'I2C(unsigned char* outbuffer,int* length,')
                    arduino_i2cmessagefile_cpp.write('int encode_' + message.get('name') + 'I2C(unsigned char* outbuffer,int* length,')
                    ros_i2cmessagefile_header.write('\r\n\tint decode_' + message.get('name') + 'I2C(unsigned char* inbuffer,int * length,')
                    ros_i2cmessagefile_cpp.write('int I2CMessageHandler::decode_' + message.get('name') + 'I2C(unsigned char* inbuffer,int * length,')
                #elif(type_command == 1):
                #    arduino_spimessagefile_header.write('\r\nint decode_' + message.get('name') + 'SPI(unsigned char* inbuffer,int* length,unsigned char checksum,')
                #    arduino_spimessagefile_cpp.write('\r\nint decode_' + message.get('name') + 'SPI(unsigned char* inbuffer,int* length,unsigned char checksum,')
                #    ros_spimessagefile_header.write('\r\n\tint encode_' + message.get('name') + 'SPI(unsigned char* outbuffer,int * length,')
                #    ros_spimessagefile_cpp.write('int SPIMessageHandler::encode_' + message.get('name') + 'SPI(unsigned char* outbuffer,int * length,')
                index = 0
                for item in fieldlist:
                    if(type_query == 1):
                        if(item.datatype == 'unsigned char'):
                            arduino_i2cmessagefile_header.write( item.datatype + ' ' + item.name)
                            arduino_i2cmessagefile_cpp.write(item.datatype + ' ' + item.name)
                            ros_i2cmessagefile_header.write( item.datatype + '* ' + item.name)
                            ros_i2cmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            arduino_i2cmessagefile_header.write( 'unsigned int ' + item.name)
                            arduino_i2cmessagefile_cpp.write('unsigned int ' + item.name)
                            ros_i2cmessagefile_header.write( item.datatype + '* ' + item.name)
                            ros_i2cmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    #elif(type_command == 1):
                    #    if(item.datatype == 'unsigned char'):
                    #        arduino_spimessagefile_header.write('unsigned char * ' + item.name)
                    #        arduino_spimessagefile_cpp.write('unsigned char * ' + item.name)
                    #        ros_spimessagefile_header.write( item.datatype + ' ' + item.name)
                    #        ros_spimessagefile_cpp.write( item.datatype + ' ' + item.name)
                    #    else:
                    #        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    index += 1
                    if(index < len(fieldlist)):
                        if((type_query == 1)):# or (type_command == 1)):
                            arduino_i2cmessagefile_header.write(',')
                            arduino_i2cmessagefile_cpp.write(',')
                            ros_i2cmessagefile_header.write(',')
                            ros_i2cmessagefile_cpp.write(',')
                if((type_query == 1)):# or (type_command == 1)):
                    arduino_i2cmessagefile_header.write(');\r\n')
                    arduino_i2cmessagefile_cpp.write(')\r\n{\r\n')
                    ros_i2cmessagefile_header.write(');\r\n')
                    ros_i2cmessagefile_cpp.write(')\r\n{\r\n')
                message_id = hex(int(message.get('id'),0)-int('0xAB00',0))
                bytelength = 0
                for item in fieldlist:
                    if(item.datatype == 'unsigned char'):
                        bytelength = bytelength +1
                    elif(item.datatype == 'uint16_t'):
                        bytelength = bytelength +2
                    else:
                        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                if(bytelength > 12): print "ERROR ERROR ERROR: Currently SPI Messages longer than 12 bytes are not supported.", " at line: ",currentframe().f_lineno
                if(type_query == 1):
                    arduino_i2cmessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                #elif(type_command == 1):
                #    ros_spimessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                bytecounter = 0
                for item in fieldlist:
                    if(item.datatype == 'unsigned char'): 
                        if(type_query == 1):
                            arduino_i2cmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                            ros_i2cmessagefile_cpp.write('\t*' + item.name + ' = inbuffer[' + str(bytecounter) + '];\r\n')
                            bytecounter = bytecounter + 1
                        #elif(type_command == 1):
                        #    arduino_spimessagefile_cpp.write('\t*' + item.name + ' = inbuffer[' + str(bytecounter) + '];\r\n')
                        #    ros_spimessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name + ';\r\n')
                        #    bytecounter = bytecounter + 1
                    elif(item.datatype == 'uint16_t'): 
                        if(type_query == 1):
                            arduino_i2cmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +'>>8;\r\n')
                            ros_i2cmessagefile_cpp.write('\tint v_' + item.name + ' = inbuffer[' + str(bytecounter) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            arduino_i2cmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                            ros_i2cmessagefile_cpp.write('\t*' + item.name + ' = v_' + item.name + ' + inbuffer[' + str(bytecounter) + '];\r\n')
                            bytecounter = bytecounter + 1
                    #    elif(type_command == 1):
                    #        print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                    #else:
                    #    print "ERROR: Datatype not supported:",item.datatype, " at line: ",currentframe().f_lineno
                for b in range(bytelength,12):
                    if(type_query == 1):
                        arduino_i2cmessagefile_cpp.write('\t*p_outbuffer++ = 0;\r\n')
                    #elif(type_command == 1):
                    #    ros_spimessagefile_cpp.write('\t*p_outbuffer++ = 0;\r\n') 
                if(type_query == 1):
                    arduino_i2cmessagefile_cpp.write('\tunsigned char checksum = 0;\r\n')
                    arduino_i2cmessagefile_cpp.write('\tfor(int i = 0; i < 12;i++)\r\n\t{\r\n')
                    arduino_i2cmessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    arduino_i2cmessagefile_cpp.write('\t}\r\n\t*p_outbuffer++ = checksum;\r\n\tlength[0] = 12;\r\n')
                    arduino_i2cmessagefile_cpp.write('\treturn 1;\r\n')
                    arduino_i2cmessagefile_cpp.write('}\r\n')
                    ros_i2cmessagefile_cpp.write('\treturn 1;\r\n')
                    ros_i2cmessagefile_cpp.write('}\r\n')
                #elif(type_command == 1):
                #    arduino_spimessagefile_cpp.write('\tunsigned char calc_checksum = SPI_' + message.get('name') + '_ID;\r\n')
                #    arduino_spimessagefile_cpp.write('\tfor(int i = 0; i < 12; i++)\r\n\t{\r\n')
                #    arduino_spimessagefile_cpp.write('\t\tcalc_checksum ^= inbuffer[i];\r\n\t}\r\n')
                #    arduino_spimessagefile_cpp.write('\tif(calc_checksum == checksum)\r\n\t{\r\n')
                #    arduino_spimessagefile_cpp.write('\t\treturn 1;\r\n\t}\r\n')
                #    arduino_spimessagefile_cpp.write('\telse\r\n\t{\r\n\t\treturn 0;\r\n\t}\r\n}')
                #    ros_spimessagefile_cpp.write('\tunsigned char checksum = 0;\r\n')
                #    ros_spimessagefile_cpp.write('\tfor(int i = 0; i < 12; i++)\r\n\t{\r\n')
                #    ros_spimessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n\t}\r\n')
                #    ros_spimessagefile_cpp.write('\t*p_outbuffer++ = checksum;\r\n')
                #    ros_spimessagefile_cpp.write('\tlength[0] = 12;\r\n')
                #    ros_spimessagefile_cpp.write('\treturn 1;\r\n}')
                
                
                    
    f = open("/home/robot/catkin_ws/src/eROS/include/eROS_Definitions.h", "r")
    contents = f.readlines()
    start_index = 0
    finish_index = 0
    stop_index = 0
    f.close()
    for i in range(0, len(contents)):
        found = contents[i].find("TAG: Start Message")
        if (found > -1):
            start_index = i+1
        found = contents[i].find("TAG: End Message")
        if (found > -1):
            stop_index = i+1

    for i in range(start_index+1,stop_index):
        del contents[start_index]
    f = open("/home/robot/catkin_ws/src/eROS/include/eROS_Definitions.h", "w")
    for i in range(0,len(message_strings)):
        contents.insert(start_index,message_strings[i])  
    contents = "".join(contents)
    f.write(contents)
    f.close()
    
    ros_udpmessagefile_header.write('private:\r\n')
    ros_udpmessagefile_header.write('};\r\n#endif')
    gui_udpmessagefile_header.write('private:\r\n')
    gui_udpmessagefile_header.write('};\r\n#endif')
    ros_serialmessagefile_header.write('private:\r\n')
    ros_serialmessagefile_header.write('};\r\n#endif')
    arduino_serialmessagefile_header.write('private:\r\n')
    arduino_serialmessagefile_header.write('};\r\n#endif')
    propeller_serialmessagefile_header.write('#endif')
    arduino_spimessagefile_header.write('#endif')
    ros_spimessagefile_header.write('private:\r\n')
    ros_spimessagefile_header.write('};\r\n#endif')
    arduino_i2cmessagefile_header.write('#endif')
    ros_i2cmessagefile_header.write('private:\r\n')
    ros_i2cmessagefile_header.write('};\r\n#endif')



if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-?"):
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-h"):
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-g"):
    ros_udpmessagefile_header = open('generated/ros/udpmessage.h','w+')
    ros_udpmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_udpmessagefile_header.write('/***Created on:')
    ros_udpmessagefile_header.write( str(datetime.now()))
    ros_udpmessagefile_header.write('***/\r\n')
    ros_udpmessagefile_cpp = open('generated/ros/udpmessage.cpp','w+')
    ros_udpmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_udpmessagefile_cpp.write('/***Created on:')
    ros_udpmessagefile_cpp.write( str(datetime.now()))
    ros_udpmessagefile_cpp.write('***/\r\n')

    gui_udpmessagefile_header = open('generated/gui/udpmessage.h','w+')
    gui_udpmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    gui_udpmessagefile_header.write('/***Created on:')
    gui_udpmessagefile_header.write( str(datetime.now()))
    gui_udpmessagefile_header.write('***/\r\n')
    gui_udpmessagefile_cpp = open('generated/gui/udpmessage.cpp','w+')
    gui_udpmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    gui_udpmessagefile_cpp.write('/***Created on:')
    gui_udpmessagefile_cpp.write( str(datetime.now()))
    gui_udpmessagefile_cpp.write('***/\r\n')

    ros_serialmessagefile_header = open('generated/ros/serialmessage.h','w+')
    ros_serialmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_serialmessagefile_header.write('/***Created on:')
    ros_serialmessagefile_header.write( str(datetime.now()))
    ros_serialmessagefile_header.write('***/\r\n')
    ros_serialmessagefile_header.write("/***Target: ROS ***/\r\n")
    ros_serialmessagefile_cpp = open('generated/ros/serialmessage.cpp','w')
    ros_serialmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_serialmessagefile_cpp.write('/***Created on:')
    ros_serialmessagefile_cpp.write( str(datetime.now()))
    ros_serialmessagefile_cpp.write('***/\r\n')
    ros_serialmessagefile_cpp.write("/***Target: ROS ***/\r\n")
    
    arduino_serialmessagefile_header = open('generated/arduino/serialmessage.h','w+')
    arduino_serialmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    arduino_serialmessagefile_header.write('/***Created on:')
    arduino_serialmessagefile_header.write( str(datetime.now()))
    arduino_serialmessagefile_header.write('***/\r\n')
    arduino_serialmessagefile_header.write("/***Target: Arduino ***/\r\n")
    arduino_serialmessagefile_cpp = open('generated/arduino/serialmessage.cpp','w')
    arduino_serialmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    arduino_serialmessagefile_cpp.write('/***Created on:')
    arduino_serialmessagefile_cpp.write( str(datetime.now()))
    arduino_serialmessagefile_cpp.write('***/\r\n')
    arduino_serialmessagefile_cpp.write("/***Target: Arduino ***/\r\n")

    propeller_serialmessagefile_header = open('generated/propeller/serialmessage.h','w')
    propeller_serialmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    propeller_serialmessagefile_header.write('/***Created on:')
    propeller_serialmessagefile_header.write( str(datetime.now()))
    propeller_serialmessagefile_header.write('***/\r\n')
    propeller_serialmessagefile_header.write("/***Target: Parallax Propeller ***/\r\n")
    propeller_serialmessagefile_cpp = open('generated/propeller/serialmessage.c','w')
    propeller_serialmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    propeller_serialmessagefile_cpp.write('/***Created on:')
    propeller_serialmessagefile_cpp.write( str(datetime.now()))
    propeller_serialmessagefile_cpp.write('***/\r\n')
    propeller_serialmessagefile_cpp.write("/***Target: Parallax Propeller ***/\r\n")

    arduino_spimessagefile_header = open('generated/arduino/spimessage.h','w')
    arduino_spimessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    arduino_spimessagefile_header.write('/***Created on:')
    arduino_spimessagefile_header.write( str(datetime.now()))
    arduino_spimessagefile_header.write('***/\r\n')
    arduino_spimessagefile_header.write("/***Target: Arduino ***/\r\n")
    arduino_spimessagefile_cpp = open('generated/arduino/spimessage.cpp','w')
    arduino_spimessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    arduino_spimessagefile_cpp.write('/***Created on:')
    arduino_spimessagefile_cpp.write( str(datetime.now()))
    arduino_spimessagefile_cpp.write('***/\r\n')
    arduino_spimessagefile_cpp.write("/***Target: Arduino ***/\r\n")

    ros_spimessagefile_header = open('generated/ros/spimessage.h','w+')
    ros_spimessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_spimessagefile_header.write('/***Created on:')
    ros_spimessagefile_header.write( str(datetime.now()))
    ros_spimessagefile_header.write('***/\r\n')
    ros_spimessagefile_header.write("/***Target: Raspberry Pi ***/\r\n")
    ros_spimessagefile_cpp = open('generated/ros/spimessage.cpp','w')
    ros_spimessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_spimessagefile_cpp.write('/***Created on:')
    ros_spimessagefile_cpp.write( str(datetime.now()))
    ros_spimessagefile_cpp.write('***/\r\n')
    ros_spimessagefile_cpp.write("/***Target: Raspberry Pi ***/\r\n")

    arduino_i2cmessagefile_header = open('generated/arduino/i2cmessage.h','w')
    arduino_i2cmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    arduino_i2cmessagefile_header.write('/***Created on:')
    arduino_i2cmessagefile_header.write( str(datetime.now()))
    arduino_i2cmessagefile_header.write('***/\r\n')
    arduino_i2cmessagefile_header.write("/***Target: Arduino ***/\r\n")
    arduino_i2cmessagefile_cpp = open('generated/arduino/i2cmessage.cpp','w')
    arduino_i2cmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    arduino_i2cmessagefile_cpp.write('/***Created on:')
    arduino_i2cmessagefile_cpp.write( str(datetime.now()))
    arduino_i2cmessagefile_cpp.write('***/\r\n')
    arduino_i2cmessagefile_cpp.write("/***Target: Arduino ***/\r\n")

    ros_i2cmessagefile_header = open('generated/ros/i2cmessage.h','w+')
    ros_i2cmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_i2cmessagefile_header.write('/***Created on:')
    ros_i2cmessagefile_header.write( str(datetime.now()))
    ros_i2cmessagefile_header.write('***/\r\n')
    ros_i2cmessagefile_header.write("/***Target: Raspberry Pi ***/\r\n")
    ros_i2cmessagefile_cpp = open('generated/ros/i2cmessage.cpp','w')
    ros_i2cmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_i2cmessagefile_cpp.write('/***Created on:')
    ros_i2cmessagefile_cpp.write( str(datetime.now()))
    ros_i2cmessagefile_cpp.write('***/\r\n')
    ros_i2cmessagefile_cpp.write("/***Target: Raspberry Pi ***/\r\n")
    
    message_strings = []
    #eros_definitionsfile_header = open('/home/robot/catkin_ws/src/eROS/include/eROS_Definitions.h','a')
   
    generate_message(sys.argv[2])
    ros_udpmessagefile_header.close()
    ros_udpmessagefile_cpp.close()
    gui_udpmessagefile_header.close()
    gui_udpmessagefile_cpp.close()

    ros_serialmessagefile_header.close()
    ros_serialmessagefile_cpp.close()
    propeller_serialmessagefile_header.close()
    propeller_serialmessagefile_cpp.close()
    arduino_serialmessagefile_header.close()
    arduino_serialmessagefile_cpp.close()

    arduino_spimessagefile_header.close()
    arduino_spimessagefile_cpp.close()
    ros_spimessagefile_header.close()
    ros_spimessagefile_cpp.close()

    arduino_i2cmessagefile_header.close()
    arduino_i2cmessagefile_cpp.close()
    ros_i2cmessagefile_header.close()
    ros_i2cmessagefile_cpp.close()
    

    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/serialmessage.h','/home/robot/catkin_ws/src/icarus_rover_v2/include/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/udpmessage.h','/home/robot/catkin_ws/src/icarus_rover_v2/include/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/spimessage.h','/home/robot/catkin_ws/src/icarus_rover_v2/include/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/i2cmessage.h','/home/robot/catkin_ws/src/icarus_rover_v2/include/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/serialmessage.cpp','/home/robot/catkin_ws/src/icarus_rover_v2/util/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/udpmessage.cpp','/home/robot/catkin_ws/src/icarus_rover_v2/util/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/spimessage.cpp','/home/robot/catkin_ws/src/icarus_rover_v2/util/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/ros/i2cmessage.cpp','/home/robot/catkin_ws/src/icarus_rover_v2/util/')
    
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/gui/udpmessage.h','/home/robot/gui/DriverStation/DriverStation/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/gui/udpmessage.cpp','/home/robot/gui/DriverStation/DriverStation/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/gui/udpmessage.h','/home/robot/gui/Diagnostics_GUI/Diagnostics_GUI/')
    copy2('/home/robot/catkin_ws/src/eROS/src/Communication/MessageGeneration/generated/gui/udpmessage.cpp','/home/robot/gui/Diagnostics_GUI/Diagnostics_GUI/')


