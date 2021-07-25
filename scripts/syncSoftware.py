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
    parser.add_option("-c","--contentmode",dest="content_mode",default="manual",help="Content Mode.  Options: manual(Default),automatic")
    parser.add_option("-f","--folderlist", dest="folder_list",action="append", type="str",help="Folder list to sync.  Input multiple values with option preceeding each one.")
    parser.add_option("-t","--foldertype",dest="folder_type_list",action="append",type="str",help="Folder Types.  Default is GENERIC.  Input multiple values with option preceeding each one.  If no option is set, Generic will be assumed.")
    parser.add_option("-d","--device",dest="device",help="Device to sync to.")
    parser.add_option("-u","--user",dest="user",default="robot",help="User account on Target. Default=robot")
    #parser.add_option("-f","--folderlist",dest="folder_list",default="",action="append",help="folder list to sync.",type="string")
    #parser.add_option("-s","--syncmode",dest="syncmode",default="all",help="all,buildserver,buildserver_target, [default: %default]")
    #parser.add_option("-b","--build",dest="build",default=0,help="1,0 [default: %default]",type="int")
    #parser.add_option("-d","--devices",dest="devices",default="",help="DeviceName1,DeviceName2,... [default: %default]")
    #parser.add_option("-c","--config_dir",dest="config_dir",default="~/config/",help="Location where Config dir should be found. default=~/config")
    (opts,args) = parser.parse_args()
    verbose = Verbosity.convert(opts.verbose)
    content_mode = ContentSync.convert_ContentMode(opts.content_mode)
    if(content_mode != ContentSyncType.MANUAL):
        print COLOR_YELLOW + "ERROR: Content Model MANUAL is currently Required." + COLOR_END
        return
    if(opts.folder_list is None):
        print COLOR_YELLOW + "ERROR: Folder List is currently required." + COLOR_END
        return
    if(len(opts.folder_list) > 0):
        if(opts.folder_type_list is None):
            opts.folder_type_list = []
            for folder in opts.folder_list:
                opts.folder_type_list.append("generic")
        elif(len(opts.folder_type_list) != len(opts.folder_list)):
            print COLOR_RED + " ERROR: Folder Type List must match the size of the Folder List." + COLOR_END
            return
    

    folder_list = opts.folder_list
    folder_type_list = []
    for folder_type in opts.folder_type_list:
        t = ContentSync.convert_FolderType(folder_type)
        folder_type_list.append(t)
    # Validate any input arguments
    if(content_mode == ContentSyncType.UNKNOWN):
        return
    
    print COLOR_GREEN + " Sync Beginning to: " + opts.device + "..." + COLOR_END
    if ContentSync.sync(opts.user,opts.device,folder_type_list,folder_list,opts.clean,opts.dryrun,verbose) == False:
        print COLOR_RED + " Sync Failed!" + COLOR_END
if __name__ == "__main__":
    main()
