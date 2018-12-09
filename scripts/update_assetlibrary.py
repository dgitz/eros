#Author: David Gitz
#Purpose: Reads FAST Robotics Library and copies Device Images to UI assets library
import sys
import time
import os
from PIL import Image
from shutil import copyfile
import pdb
def print_usage():
    print "Usage Instructions"
    print "Update Assets: -u <FAST Library> <Asset Library>"

def update(sourcelib,destlib):
    counter = 0
    for (dirpath, dirnames, filenames) in os.walk(sourcelib):
        for d in dirnames:
            if(d == "MEDIA"):
                pn = dirpath[dirpath.rfind("/")+1:]
                source_image_jpg = dirpath + "/MEDIA/" + pn + ".jpg"
                source_image_png = dirpath + "/MEDIA/" + pn + ".png"
                if((os.path.isfile(source_image_jpg) == True) and (os.path.isfile(source_image_png) == False)):
                    print "ONLY JPG EXISTS: " + source_image_jpg
                    im = Image.open(source_image_jpg)
                    im.save(source_image_png) 
                
                if(os.path.isfile(source_image_png) == True):
                    #print "PNG EXISTS: " + source_image_jpg 
                    dest_dir = destlib + pn + "/"
                    if not os.path.exists(dest_dir):
                        os.mkdir(dest_dir)
                    if not os.path.exists(dest_dir + "MEDIA/"):
                        os.mkdir(dest_dir + "MEDIA/")
                    dest_dir_path = dest_dir + "MEDIA/" + pn + ".png"
                    copyfile(source_image_png,dest_dir_path)
                    
                else:
                    print "IMAGE DOES NOT EXIST: " + source_image_png

if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-u"):
    update(sys.argv[2],sys.argv[3])

        
