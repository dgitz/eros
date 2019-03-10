#!/usr/bin/python
import sys
import platform
BuildServer = False
if(platform.processor() == ''):
    BuildServer = True
if(BuildServer == True):
    import RPi.GPIO as GPIO
from time import sleep
import os 
import check_unittests
import pdb
led_red = 26
led_yellow = 20
led_green = 21
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
def print_usage():
    print "Usage Instructions:"
    print "No Parameters Needed."
def run_auto():
    ok = True
    print bcolors.OKBLUE + "BUILDING SOURCE CODE..." + bcolors.ENDC
    if(BuildServer == True):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(led_red, GPIO.OUT)
        GPIO.setup(led_yellow, GPIO.OUT)
        GPIO.setup(led_green, GPIO.OUT)
        GPIO.output(led_red,GPIO.LOW)
        GPIO.output(led_yellow,GPIO.LOW)
        GPIO.output(led_green,GPIO.LOW)
        GPIO.output(led_yellow,GPIO.HIGH)
    v = os.system("bash /home/robot/scripts/build.sh")
    if(v == 0):
        print bcolors.OKGREEN + "BUILD OF SOURCE CODE SUCCEEDED." + bcolors.ENDC
    else:
        ok = False
        print bcolors.FAIL + "BUILD OF SOURCE CODE FAILED!" + bcolors.ENDC
        
    v = os.system("bash /home/robot/scripts/run_unittests.sh")
    if(v == 0):
        print bcolors.OKGREEN + "BUILD OF UNIT TESTS SUCCEEDED." + bcolors.ENDC
    else:
        ok = False
        print bcolors.FAIL + "BUILD OF UNIT TESTS FAILED." + bcolors.ENDC
    
    print bcolors.OKBLUE + "RUNNING UNIT TESTS..." + bcolors.ENDC
    [passed,failed_list,passed_list] = check_unittests.check_all()
    if(passed == True):
        print bcolors.OKGREEN + "ALL UNIT TESTS(" + str(len(passed_list)) + ") RAN SUCCESSFULLY." + bcolors.ENDC
    else:
        ok = False
        print bcolors.FAIL + "UNIT TEST FAILED COUNT: " + str(len(failed_list)) + "/" + str(len(passed_list)) + bcolors.ENDC

    if(BuildServer == True):
        GPIO.output(led_yellow,GPIO.LOW)
    if(BuildServer == True):
        if(ok == True):
            GPIO.output(led_green,GPIO.HIGH)
            GPIO.output(led_red,GPIO.LOW)
        else:
            GPIO.output(led_green,GPIO.LOW)
            GPIO.output(led_red,GPIO.HIGH)
    if(ok == True):
        print bcolors.OKGREEN + "AUTO-BUILD SUCCESSFUL." + bcolors.ENDC
    else:
        print bcolors.FAIL + "AUTO-BUILD FAILED." + bcolors.ENDC
run_auto()

