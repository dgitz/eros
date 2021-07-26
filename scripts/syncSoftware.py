# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb
import Helpers
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../include/eros/")
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/ArchitectureUtility/")
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/SyncUtility/")
from eROS_Definitions import *
from ArchitectureType import ArchitectureType
from Architecture import Architecture
from ContentSyncDefinition import ContentSyncType
from ContentSync import ContentSync

# Main
def main():
    parser = OptionParser("syncSoftware.py [options]\nASSUMES:\n\t- Destination File Structure matches Host File Structure.")
    parser.add_option("-q","--dry-run",default=0,dest="dryrun",type="int",help="Prepares Sync and instead of synchronizing writes to console. Options: 0(default) or 1")
    parser.add_option("-v","--verbose",dest="verbose",default="0",help="How Verbose this script should run.  Options: 0(Default),1,2")
    parser.add_option("-x","--clean",dest="clean",default=1,type="int",help="Clean Target folder before syncing. (Default=1)")
    parser.add_option("-s","--syncconfigfile",dest="syncconfigfile",help="File Path to Sync Config File.")
    parser.add_option("-f","--folderlist", dest="folder_list",action="append", type="str",help="Folder list to sync.  Input multiple values with option preceeding each one.")
    parser.add_option("-d","--device",dest="device",help="Device to sync to.")
    parser.add_option("-u","--user",dest="user",default="robot",help="User account on Target. Default=robot")
    (opts,args) = parser.parse_args()
    verbose = Verbosity.convert(opts.verbose)
    folder_list = []
    folder_type_list = []
    folders = []
    devices = []
    sync = ContentSync()
    if(opts.syncconfigfile is None):
        content_mode = ContentSyncType.MANUAL
    else:
        content_mode = ContentSyncType.AUTOMATIC
    if(content_mode == ContentSyncType.MANUAL):
        folders,devices = sync.read_config_cli(opts.folder_list,opts.user,opts.device)
    else:
        folders,devices = sync.read_config_file(opts.syncconfigfile)
        pdb.set_trace()

    #if(content_mode == ContentSyncType.MANUAL):
    if sync.sync(devices,folders,opts.clean,opts.dryrun,verbose) == False:
        print COLOR_RED + " Sync Failed!" + COLOR_END
if __name__ == "__main__":
    main()
