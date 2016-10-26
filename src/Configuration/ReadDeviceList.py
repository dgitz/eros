import xml.etree.ElementTree as ET
def ReadDeviceList():
    tree = ET.parse('/home/robot/config/DeviceFile.xml')
    root = tree.getroot()
    for DeviceList in root:
        #print DeviceList.tag#, child.text, child.attrib
        for Device in DeviceList:
            #print Device.tag, Device.text, Device.attrib
            for entry in Device:
                if (entry.tag == 'DeviceName'):
                    print "DeviceName: " + entry.text
                #print entry.tag, entry.text, entry.attrib
    return 1
