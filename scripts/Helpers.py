import xml.etree.ElementTree as ET
import pdb
devicefile = '/home/robot/config/DeviceFile.xml'
class Device():
    def __init__(self,Name='',Parent='',IPAddress='',Capability='',ID=0,PartNumber=''):
        self.Name = Name
        self.Parent = Parent
        self.ID = ID
        self.PartNumber = PartNumber
        self.IPAddress = IPAddress
        self.Capability = Capability
        self.ChildDevices = []
        self.ConnectedDevice = []
        self.Pins = []
        self.ConnectedSensor = []
        
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
def ReadDeviceTree():
    tree = ET.parse(devicefile)
    root = tree.getroot()
    root_devices = []
    device_count = 0
    for List in root:
        for device in List:
            newDevice = Device()
            device_count = device_count + 1
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
            if(newDevice.Parent == "None"):
                root_devices.append(newDevice)
    root = tree.getroot()
    added_device_count = len(root_devices)
    for List in root:
        for device in List:
            newDevice = Device()
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
                elif (entry.tag == 'Pin'):
                    for p in entry:
                        if(p.tag == 'ConnectedDevice'):
                            newDevice.ConnectedDevice.append(p.text)
                        elif(p.tag == 'ConnectedSensor'):
                            newDevice.ConnectedSensor.append(p.text)
                    
            for i in range(len(root_devices)):
                if(newDevice.Parent == root_devices[i].Name):
                    added_device_count = added_device_count + 1
                    root_devices[i].ChildDevices.append(newDevice)
    print_devicetree(root_devices)

def print_devicetree(devicetree):
    for r in devicetree:
        print r.Name
        for c in r.ChildDevices:
            print "\t" + c.Name
            for d in c.ConnectedDevice:
                print "\t\t" + d
            for s in c.ConnectedSensor:
                print "\t\t" + s
        

    
