import unittest
import sys
import pdb
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../")
from ArchitectureUtility import Architecture,ArchitectureType

class TestStringMethods(unittest.TestCase):
    def test_initialization(self):
        v = Architecture()
        self.assertTrue(v.is_initialized())
    def test_local(self):
        v = Architecture()
        arch = v.get_architecture('localhost')
        self.assertFalse(arch==ArchitectureType.UNKNOWN)
    def test_remote(self):
        v = Architecture()
        arch = v.get_architecture('DevModule1','robot')
        self.assertFalse(arch==ArchitectureType.UNKNOWN)
        arch = v.get_architecture('GPUModule1','robot')
        self.assertFalse(arch==ArchitectureType.UNKNOWN)
    def test_noexistdevice(self):
        print "The following should print an Error Message about not being able to connect to a device."
        v = Architecture()
        arch = v.get_architecture('ADeviceThatWillNeverExist','robot')
        self.assertTrue(arch==ArchitectureType.UNKNOWN)

if __name__ == '__main__':
    unittest.main()