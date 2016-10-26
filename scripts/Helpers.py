import xml.etree.ElementTree as ET
class Device():
    def __init__(self,Name='',IPAddress='',Capability=''):
        self.Name = Name
        self.IPAddress = IPAddress
        self.Capability = Capability

def ReadDeviceList(Capability):
    DeviceList = []
    tree = ET.parse('/home/robot/config/DeviceFile.xml')
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
            if((newDevice.Capability == Capability) or (Capability == '')):
                DeviceList.append(newDevice)
    return DeviceList
