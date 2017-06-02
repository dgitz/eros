import xml.etree.ElementTree as ET
devicefile = '/home/robot/config/DeviceFile.xml'
class Device():
    def __init__(self,Name='',IPAddress='',Capability=''):
        self.Name = Name
        self.IPAddress = IPAddress
        self.Capability = Capability

def ReadDeviceList(Capability):
    #global devicefile
    DeviceList = []
    try:
        tree = ET.parse(devicefile)
    except ET.ParseError as err:
        print "XML " + devicefile + " parsing error: " + format(err)
        return DeviceList
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
            if(Capability == newDevice.Capability):
                DeviceList.append(newDevice)
    return DeviceList

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

    
