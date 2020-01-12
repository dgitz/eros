import xml.etree.ElementTree as ET
import pdb
import glob,os,shutil
devicefile = '/home/robot/config/DeviceFile.xml'
class Device():
    def __init__(self,Name='',Parent='',IPAddress='',Capability='',ID=0,PartNumber='',Architecture='',DeviceType=''):
        self.Name = Name
        self.Parent = Parent
        self.ID = ID
        self.PartNumber = PartNumber
        self.IPAddress = IPAddress
        self.Capability = Capability
        self.Architecture = Architecture
        self.DeviceType=DeviceType
class FirmwareVersion():
    def __init__(self,Name='',Major_Release=0,Minor_Release=0,Build_Number=0,Description=''):
        self.Name = Name
        self.Major_Release = Major_Release
        self.Minor_Release = Minor_Release
        self.Build_Number = Build_Number
        self.Description = Description    
def checkDeviceFileFormat():
    try:
        tree = ET.parse(devicefile)
        return True
    except ET.ParseError as err:
        print "XML " + devicefile + " parsing error: " + format(err)
        return False
def ReadDeviceList(Capability):
    #global devicefile
    DeviceList = []
    tree = ET.parse(devicefile)
    root = tree.getroot()
    for List in root:
        #print DeviceList.tag#, child.text, child.attrib
        for device in List:
            #print Device.tag, Device.text, Device.attrib
            newDevice = Device()
            add_device = 0
            for entry in device:
                if (entry.tag == 'DeviceName'):
                    newDevice.Name = entry.text
                elif (entry.tag == 'PrimaryIP'):
                    newDevice.IPAddress = entry.text
                elif (entry.tag == 'Capability'):
                    newDevice.Capability = entry.text
                elif (entry.tag == 'ID'):
                    newDevice.ID = int(entry.text)
                elif (entry.tag == 'PartNumber'):
                    newDevice.PartNumber = entry.text
                elif (entry.tag == 'ParentDevice'):
                    newDevice.Parent = entry.text
                elif (entry.tag == 'Architecture'):
                    newDevice.Architecture = entry.text 
                elif (entry.tag == 'DeviceType'):
                    newDevice.DeviceType = entry.text
            if(Capability == newDevice.Capability):
                DeviceList.append(newDevice)
    return DeviceList
def GetDeviceInfo(devicename):
    #global devicefile
    DeviceList = []
    tree = ET.parse(devicefile)
    root = tree.getroot()
    for List in root:
        #print DeviceList.tag#, child.text, child.attrib
        for device in List:
            #print Device.tag, Device.text, Device.attrib
            newDevice = Device()
            add_device = 0
            for entry in device:
                if (entry.tag == 'DeviceName'):
                    newDevice.Name = entry.text
                elif (entry.tag == 'PrimaryIP'):
                    newDevice.IPAddress = entry.text
                elif (entry.tag == 'Capability'):
                    newDevice.Capability = entry.text
                elif (entry.tag == 'ID'):
                    newDevice.ID = int(entry.text)
                elif (entry.tag == 'PartNumber'):
                    newDevice.PartNumber = entry.text
                elif (entry.tag == 'ParentDevice'):
                    newDevice.Parent = entry.text
                elif (entry.tag == 'Architecture'):
                    newDevice.Architecture = entry.text
                elif (entry.tag == 'DeviceType'):
                    newDevice.DeviceType = entry.text
            DeviceList.append(newDevice)
    for dev in DeviceList:
        if(dev.Name == devicename):
            return dev
    
def ReadCapabilityList():
    #global devicefile
    CapabilityList = []
    tree = ET.parse(devicefile)
    root = tree.getroot()
    for List in root:
        for device in List:
            capability = ""
            for entry in device:
                if(entry.tag == 'Capability'):
                    capability = entry.text
            found = False
            for i in range(0,len(CapabilityList)):
                if(CapabilityList[i] == capability):
                    found = True
            if(found == False):
                CapabilityList.append(capability)
    return CapabilityList
def ReadFirmwareVersions(content_directory):
    firmware_list = []
    for directory,dir_list,file_list in os.walk(content_directory):
        for fname in file_list:
            if('sandbox' not in directory):
                if 'node.h' in fname:
                    with open(directory + "/" + fname, 'r') as f:
                        lines = f.readlines()
                        contains_info = False
                        node_name = ''
                        major_version = 0
                        minor_version = 0
                        build_number = 0
                        description = ''
                        
                        for line in lines:
                            if 'BASE_NODE_NAME' in line:
                                contains_info = True
                                node_name = line[32:-3]
                            if 'MAJOR_RELEASE_VERSION' in line:
                                major_version = int(line[38:-2].strip())
                            if 'MINOR_RELEASE_VERSION' in line:
                                minor_version = int(line[38:-2].strip())
                            if 'BUILD_NUMBER' in line:
                                build_number = int(line[29:-2].strip())
                            if 'FIRMWARE_DESCRIPTION' in line:
                                description = line[38:-3]
                        if(contains_info == True):
                            firmware_list.append(FirmwareVersion(node_name,major_version,minor_version,build_number,description))
    firmware_list = sorted(firmware_list,key=lambda firmware: firmware.Name)   
    return firmware_list

    
