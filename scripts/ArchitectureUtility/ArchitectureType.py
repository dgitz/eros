#!/usr/bin/env python
##
# @file ArchitectureType.py
from enum import Enum
import subprocess
import pdb


## @package pyexample
#  Documentation for this module.
#
#  More details.
class ArchitectureType(Enum):
 
 """! The ArchitectureType Enum
    The ArchitectureType is an enum that holds different supported Architectures.
    """
    UNKNOWN = 0
    X86_64 =1 
    ARMV7L = 2
    AARCH64 = 3
    END_OF_LIST= 4
