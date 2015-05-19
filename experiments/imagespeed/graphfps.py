#!/usr/bin/env python3
import sys,os,re
import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import rc
from scipy import stats

oldfps_noload = [30,21,18,6.1,7.5,2.2]
oldfps_load   = [23,12,9.5,2.5,2.5,0.9]
newfps = [30,25,20,8,9.4,2.8]
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
    ind = np.arange(len(labelz))
    width = 0.35
    ax = plt.gca()
    
    ax.bar(ind, oldfps_noload, width, color='r', label="2014 code (no object)",yerr=0.5)
    ax.bar(ind, oldfps_load, width, color='y', label="2014 code (tracking object)",yerr=0.5)
    ax.bar(ind+width, newfps, width, color='b', label="2015 code",yerr=0.5)
    
    handles, labels = plt.gca().get_legend_handles_labels()
    plt.gca().legend(handles, labels, prop={'size': 10}, fancybox=True, framealpha=0.7).draggable()
    plt.title("Image processing performance comparison")
    ax.set_xticks(ind+width)
    ax.set_xticklabels(labelz)
    plt.xlabel("Processing resolution")
    plt.grid()
    plt.ylabel("Processing rate (FPS)")
    plt.show()
    
if __name__ == "__main__":
    if len(sys.argv) < 1:
        print("Usage: %s syrupy_log.txt" % sys.argv[0])
    else:
        main()
   
