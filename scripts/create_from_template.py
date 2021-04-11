# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb
import shutil
from cookiecutter.main import cookiecutter

# Defines
CRED = '\33[31m'
CYELLOW = '\33[33m'
CGREEN = '\33[32m'
CBLUE = '\33[34m'
CEND = '\033[0m'

def generate(output_folder,template_path):
    print(CGREEN + "Generating From Template:"  + template_path  + " and storing at: " + output_folder + CEND)
    if(os.path.isdir(output_folder) == False):
        os.mkdir(output_folder)
    else:
        shutil.rmtree(output_folder)
        os.mkdir(output_folder)
    #pdb.set_trace()
    os.system("cookiecutter " + template_path + " -o " + output_folder)
    print("----------\n")
    print(CGREEN + "Generating Content is Now Complete." + CEND)
    os.system("cat " + template_path + "/ExtraInstructions.md")

# Main
def main():
    parser = OptionParser("create_from_template.py [options]")
    parser.add_option("-o","--output",dest="output_folder",default="~/tmp/",help="Output Directory,[default: %default]")
    parser.add_option("-t","--tempate_path",dest="template_path",help="Path to Template")
    (opts,args) = parser.parse_args()
    generate(opts.output_folder,opts.template_path)
    

if __name__ == "__main__":
    main()
