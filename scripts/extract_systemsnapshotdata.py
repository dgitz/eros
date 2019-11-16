#!/usr/bin/python
import sys,getopt,os
import Helpers
import psutil
import pdb
import subprocess
import os
import socket
import bag_to_csv
from os import walk
from zipfile import ZipFile
from subprocess import call

def print_usage():
    print "Usage Instructions: extract_systemsnapshotdata."
    print "No Options: This Menu."
    print "-?/-h This Menu."
    print "-p <folder> Extract SystemSnap Data and perform all functions in folder containing multiple SystemSnap zip's."

def process_SystemSnapDirectory(main_directory):
    for (dirpath, dirnames, filenames) in walk(main_directory):
        for f in filenames:
            if(f[len(f)-4:len(f)] == '.zip'):
                with ZipFile(dirpath + f, 'r') as zipObj:
                    zipObj.extractall(main_directory)
                os.rename(main_directory+f, main_directory + f[0:len(f)-4] + "/" + f[0:len(f)-4] + ".bk")
        break
    for (dirpath, dirnames, filenames) in walk(main_directory):
        for directory in dirnames:
            bag_to_csv.convert(dirpath + "/" + directory + "/","all",dirpath + "/" + directory + "/")
        break

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?hp:",["help"])
    if(len(opts) == 0):
        print_usage()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-h':
            print_usage()
        elif opt == '-p':
            process_SystemSnapDirectory(arg)

if __name__ == "__main__":
    main()
