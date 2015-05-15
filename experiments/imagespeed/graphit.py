#!/usr/bin/env python3
import sys,os,re
import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import rc
from scipy import stats

files = ["320at160.txt", "320at320.txt", "640at320.txt",
         "640at640.txt", "1280at640.txt", "1280at1280.txt"]
labelz= ["320x240\n(160x120)", "320x240\n(320x240)", "640x480\n(320x240)",
         "640x480\n(640x480)", "1280x720\n(640x480)", "1280x720\n(1280x720)"]

font = {'family' : 'serif'}
rc('font', **font)

def readdata(input):
    data = pd.read_csv(input, parse_dates=[[1,2]], delimiter=r"\s+")
    data["MEM"] = data["MEM"] * 446272/102400;
    data["DATE_TIME"] -= data["DATE_TIME"].iat[0]
    data["DATE_TIME"] = data["DATE_TIME"].astype('timedelta64[s]').astype('float') #/ 3600
    data = data.set_index("DATE_TIME", drop=False)
    return data#[:1.2]

def main():
    old = [readdata("old/" + d)["CPU"] for d in files]
    new = [readdata("new/proc" + d)["CPU"] for d in files]
    oldcpu = [d.mean() for d in old]
    newcpu = [d.mean() for d in new]
    
    ind = np.arange(len(labelz))
    width = 0.35
    ax = plt.gca()
    
    ax.bar(ind, oldcpu, width, color='r', label="2014 code")
    ax.bar(ind+width, newcpu, width, color='b', label="2015 code")
    
    handles, labels = plt.gca().get_legend_handles_labels()
    plt.gca().legend(handles, labels, prop={'size': 10}, fancybox=True, framealpha=0.7).draggable()
    plt.title("Image processing performance comparison")
    ax.set_xticks(ind+width)
    ax.set_xticklabels(labelz)
    plt.xlabel("Processing resolution")
    plt.grid()
    plt.ylabel("Mean CPU usage (\\%)")
    plt.show()
    
if __name__ == "__main__":
    if len(sys.argv) < 1:
        print("Usage: %s syrupy_log.txt" % sys.argv[0])
    else:
        main()
   
