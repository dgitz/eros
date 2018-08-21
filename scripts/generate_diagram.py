import Helpers
import sys,getopt,os

DeviceList = []
def print_usage():
    print "Usage Instructions: generate_diagram."
    print "No Options: This Menu."
    print "-?/-h This Menu."
    print "-a Generate Diagram"


def generate_diagram():
    DeviceList = Helpers.ReadDeviceTree()
    

def main():
    opts, args = getopt.getopt(sys.argv[1:],"?ha",["help"])
    if(len(opts) == 0):
        print_usage()
    for opt, arg in opts:
        if opt == '-?':
            print_usage()
        elif opt == '-h':
            print_usage()
        elif opt == '-a':
            generate_diagram()

if __name__ == "__main__":
    main()
