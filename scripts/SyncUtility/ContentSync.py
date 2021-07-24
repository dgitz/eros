import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../../include/eros/")
from eROS_Definitions import *
from ContentSyncType import ContentSyncType
import pdb
class ContentSync():

    @staticmethod
    def convert(content_mode_str):
        if(content_mode_str == "manual"):
            return ContentSyncType.MANUAL
        elif(content_mode_str == "auto"):
            return ContentSyncType.AUTOMATIC
        else:
            print(COLOR_RED + "ERROR Content Mode : " + content_mode_str + " Not supported." + COLOR_END)
            return ContentSyncType.UNKNOWN