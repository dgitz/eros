import xml.etree.ElementTree as ET
import pdb
import glob,os,shutil
import json
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
    def __init__(self,Name='',Type='',Path='',Architectures=[]):
        self.Name = Name
        self.Type = Type
        self.Directory = Path
        self.Architectures = Architectures
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
            Architectures = []
            for entry in folder:
                if (entry.tag == 'Name'):
                    newFolder.Name = entry.text
                elif (entry.tag == 'Type'):
                    newFolder.Type = entry.text
                elif (entry.tag == 'Directory'):
                    newFolder.Directory = entry.text
                elif (entry.tag == 'Architecture'):
                    Architectures.append(entry.text)
            newFolder.Architectures=Architectures
            FolderList.append(newFolder)
    return FolderList
def ReadDeviceList(file_path,Capability):
    #global devicefile
    DeviceList = []
    with open(file_path) as f:
         data = json.load(f)
         data = data['DeviceList']
         for device_name in data:
            obj = data[device_name]
            try:
                if(obj['Capability'] == 'ROS'):
                    try:
                        newDevice = Device()
                        newDevice.Name = device_name
                        newDevice.Capability = 'ROS'
                        newDevice.CatkinWS = obj['CatkinWS']
                        newDevice.Architecture = obj['Architecture']
                        newDevice.Jobs = int(obj['Jobs'])
                        newDevice.DeviceType = obj['Type']
                        DeviceList.append(newDevice)
                    except KeyError as err:
                        print("Device: " + device_name + " Missing Key! with error: ",err)
            except KeyError:
                 a = 1 # Do Nothing
    return DeviceList

    
