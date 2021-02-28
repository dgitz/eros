import xml.etree.ElementTree as ET
import pdb
import glob,os,shutil
class Device():
    def __init__(self,Name='',Parent='',ID=0,PartNumber='',Capability='',CatkinWS = '',Architecture='',Jobs=0):
        self.Name = Name
        self.Parent = Parent
        self.ID = ID
        self.PartNumber = PartNumber
        self.Capability = Capability
        self.CatkinWS = CatkinWS
        self.Architecture = Architecture
        self.Jobs = Jobs
class Folder(object):
    def __init__(self,Name='',Type='',Path=''):
        self.Name = Name
        self.Type = Type
        self.Directory = Path
def checkDeviceFileFormat():
    try:
        tree = ET.parse(devicefile)
        return True
    except ET.ParseError as err:
        print("XML " + devicefile + " parsing error: " + format(err))
        return False
def ReadSyncConfig(file_path):
    #global devicefile
    FolderList = []
    tree = ET.parse(file_path)
    root = tree.getroot()
    for List in root:
        for folder in List:
            newFolder = Folder()
            for entry in folder:
                if (entry.tag == 'Name'):
                    newFolder.Name = entry.text
                elif (entry.tag == 'Type'):
                    newFolder.Type = entry.text
                elif (entry.tag == 'Directory'):
                    newFolder.Directory = entry.text
            FolderList.append(newFolder)
    return FolderList
def ReadDeviceList(file_path,Capability):
    #global devicefile
    DeviceList = []
    tree = ET.parse(file_path)
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
                elif (entry.tag == 'Capability'):
                    newDevice.Capability = entry.text
                elif (entry.tag == 'ID'):
                    newDevice.ID = int(entry.text)
                elif (entry.tag == 'PartNumber'):
                    newDevice.PartNumber = entry.text
                elif (entry.tag == 'ParentDevice'):
                    newDevice.Parent = entry.text
                elif (entry.tag == 'CatkinWS'):
                    newDevice.CatkinWS = entry.text
                elif (entry.tag == 'Architecture'):
                    newDevice.Architecture = entry.text 
                elif (entry.tag == 'Jobs'):
                    newDevice.Jobs = entry.text 
                elif (entry.tag == 'DeviceType'):
                    newDevice.DeviceType = entry.text
            if(Capability == newDevice.Capability):
                DeviceList.append(newDevice)
    return DeviceList

    
