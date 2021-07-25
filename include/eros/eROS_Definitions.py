#!/usr/bin/env python
##
# @file eROS_Definitions.py
from enum import Enum
import pdb
# Defines
COLOR_RED = '\33[31m'
COLOR_YELLOW = '\33[33m'
COLOR_GREEN = '\33[32m'
COLOR_BLUE = '\33[34m'
COLOR_END = '\033[0m'

class VerbosityLevel(Enum):
    QUIET=0,
    VERBOSE=1,
    VERYVERBOSE=2
class Verbosity():

    @staticmethod
    def convert(verbose_str):
        if(verbose_str == "0"):
            return VerbosityLevel.QUIET
        elif(verbose_str == "1"):
            return VerbosityLevel.VERBOSE
        elif(verbose_str == "2"):
            return VerbosityLevel.VERYVERBOSE
        else:
            print "ERROR Verbose Level: " + verbose_str + " Not supported.  Setting to Very Verbose."
            return VerbosityLevel.VERYVERBOSE