import xml.etree.ElementTree as ET
import pdb
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

    
