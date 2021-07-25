import unittest
import sys
import pdb
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))  + "/../")
from ContentSyncDefinition import ContentSyncType
from ContentSync import ContentSync

class TestStringMethods(unittest.TestCase):
    def test_initialization(self):
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()