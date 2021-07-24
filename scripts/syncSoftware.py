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
from ContentSyncType import ContentSyncType
from ContentSync import ContentSync

# Main
def main():
    parser = OptionParser("syncSoftware.py [options]")
    parser.add_option("-d","--dry-run",dest="dryrun",default="False",help="Prepares Sync and instead of synchronizing writes to console.")
    parser.add_option("-v","--verbose",dest="verbose",default="0",help="How Verbose this script should run.  Options: 0,1,2")
    parser.add_option("-c","--contentmode",dest="content_mode",default="manual",help="Content Mode.  Options: manual,automatic")
    #parser.add_option("-s","--syncmode",dest="syncmode",default="all",help="all,buildserver,buildserver_target, [default: %default]")
    #parser.add_option("-b","--build",dest="build",default=0,help="1,0 [default: %default]",type="int")
    #parser.add_option("-d","--devices",dest="devices",default="",help="DeviceName1,DeviceName2,... [default: %default]")
    #parser.add_option("-c","--config_dir",dest="config_dir",default="~/config/",help="Location where Config dir should be found. default=~/config")
    (opts,args) = parser.parse_args()
    verbose = Verbosity.convert(opts.verbose)
    content_mode = ContentSync.convert(opts.content_mode)

    # Validate any input arguments
    if(content_mode == ContentSyncType.UNKNOWN):
        return
    
    print COLOR_GREEN + " Sync Beginning..." + COLOR_END
if __name__ == "__main__":
    main()
