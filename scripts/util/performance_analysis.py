from .Helpers import *
import matplotlib.pyplot as plt
class Figure:
    def __init__(self,title,figure):
        self.title = title
        self.figure = figure
def analyze_loadfactor(output_dir,loadFactor):
    stamp = [x- loadFactor.stamp[0] for x in loadFactor.stamp]
    figList = []
    fig = plt.figure()
    plt.plot(stamp,loadFactor.loadfactor_1min,'b',label='1 Min')
    plt.plot(stamp,loadFactor.loadfactor_5min,'g',label='5 Min')
    plt.plot(stamp,loadFactor.loadfactor_15min,'r',label='15 Min')
    plt.xlabel('Time (s)')
    plt.ylabel('Load Factor')
    plt.legend()
    plt.title("LoadFactor: " + loadFactor.device)
    fig = plt
    figList.append(Figure('LoadFactor_' + loadFactor.device,fig))
    for fig in figList:
        fig.figure.savefig(output_dir + fig.title,bbox_inches='tight', dpi=400)
def analyze_resourceavailable(output_dir,resourceAvailable):
    stamp = [x- resourceAvailable.stamp[0] for x in resourceAvailable.stamp]
    figList = []
    fig = plt.figure()
    plt.plot(stamp,resourceAvailable.CPU,'b',label='CPU')
    plt.plot(stamp,resourceAvailable.RAM,'g',label='RAM')
    plt.plot(stamp,resourceAvailable.Disk,'r',label='Disk')
    plt.ylim([0,100])
    plt.xlabel('Time (s)')
    plt.ylabel('Resource Available (%)')
    plt.title("Resource Available (%): " + resourceAvailable.device)
    plt.legend()
    fig = plt
    figList.append(Figure('ResourceAvailable_' + resourceAvailable.device,fig))
    for fig in figList:
        fig.figure.savefig(output_dir + fig.title,bbox_inches='tight', dpi=400)
def analyze_resourceused(output_dir,resourceUsed):
    stamp = [x- resourceUsed.stamp[0] for x in resourceUsed.stamp]
    figList = []
    fig = plt.figure()
    plt.plot(stamp,resourceUsed.CPU,'b',label='CPU')
    plt.plot(stamp,resourceUsed.RAM,'g',label='RAM')
    plt.plot(stamp,resourceUsed.Disk,'r',label='Disk')
    plt.ylim([0,100])
    plt.xlabel('Time (s)')
    plt.ylabel('Resource Used (%)')
    plt.title("Resource Used (%): " + resourceUsed.device + ": " + resourceUsed.process)
    plt.legend()
    fig = plt
    figList.append(Figure('ResourceUsed_' + resourceUsed.device + "_" + resourceUsed.process,fig))
    for fig in figList:
        fig.figure.savefig(output_dir + fig.title,bbox_inches='tight', dpi=400)