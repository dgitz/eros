#!/usr/bin/python
import sys,getopt,os
import Helpers
import psutil
import pdb
import subprocess
import os
import socket
import bag_to_csv
from optparse import OptionParser
from os import walk
from zipfile import ZipFile
from subprocess import call

def process_SystemSnapDirectory(data_directory,output_folder):
    for (dirpath, dirnames, filenames) in walk(data_directory):
        for f in filenames:
            if(f[len(f)-4:len(f)] == '.zip'):
                with ZipFile(dirpath + f, 'r') as zipObj:
                    zipObj.extractall(data_directory)
                os.rename(data_directory+f, data_directory + f[0:len(f)-4] + "/" + f[0:len(f)-4] + ".bk")
        break
    for (dirpath, dirnames, filenames) in walk(data_directory):
        for directory in dirnames:
            bag_to_csv.convert(dirpath + "/" + directory + "/","all",output_folder + "/")
        break
def process_BagDirectory(data_directory,output_folder):
    bag_to_csv.convert(data_directory + "/" ,"all",output_folder + "/")
    
def main():
    parser = OptionParser("extract_systemsnapshotdata.py [options]")
    parser.add_option("-m","--mode",dest="mode",default="Auto",help="Bag,Snap [default: %default]")
    parser.add_option("-d","--datadir",dest="data_dir",help="Directory containing data")
    parser.add_option("-o","--output",dest="output",help="Output Directory")
    (opts,args) = parser.parse_args()
    if(opts.mode == "Bag"):
        process_BagDirectory(opts.data_dir,opts.output)
    elif(opts.mode == "Snap"):
        process_SystemSnapDirectory(opts.data_dir,opts.output)
    elif(opts.mode == "Auto"):
        for (dirpath, dirnames, filenames) in walk(opts.data_dir):
            for f in filenames:
                if(f[len(f)-4:len(f)] == '.zip'):
                    process_SystemSnapDirectory(opts.data_dir,opts.output)
                    break
                elif(f[len(f)-4:len(f)] == '.bag'):
                    process_BagDirectory(opts.data_dir,opts.output)
                    break
                else:
                    print "Directory does not contain either .bag or .zip files. Exiting."
                    break
            break

if __name__ == "__main__":
    main()
