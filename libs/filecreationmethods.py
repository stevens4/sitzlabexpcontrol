'''by stevens4, mod: stevens4
2015-03-21: removed measurementType as an input to filenameGen().

2014-03-04: added the logFile class with methods for reading, writing,
creating logFiles for other objects like the LoggedStepperMotor.

2013-06-13: switched over to using full os method functionality so these
methods should be OS independent

some simple functions to help with creation of files on our network drive

our data is organized thusly:
    Z:\data\[chamber]\[date]\[measurementType]\[time]
        [chamber] = pooh, tigger, or piglet
        [date] = date data was saved
        [time] = time data was saved
'''

import csv
import datetime
import os
import numpy as np

from config.filecreation import POOHDATAPATH

#generates a relative path and filename according to our data structure given a measurement type
def filenameGen(measurementType=None):
    date = datetime.datetime.now().strftime("%Y-%m-%d")
    time = datetime.datetime.now().strftime("%H%M")
    if measurementType is not None:
        path = os.path.join(date,measurementType)
    else:
        path = os.path.join(date)
    filename = time
    return path, filename

#checks if absolute path exists, if path doesn't exist, will be created
def checkPath(path):
    if not os.path.exists(path):
        os.makedirs(path)

#given a measurementType (string), dataArray (numpyArray), and parentPath (path)
#save a CSV file according to our data structure (see above)
def saveCSV(dataArray,parentPath=POOHDATAPATH,subDir='',description=None):
    date, filename = filenameGen()
    path = os.path.join(parentPath,date)
    checkPath(path)
    if description is not None:
        filename += '_'+description
    fullFileName = os.path.join(path,filename+".csv")
    np.savetxt(fullFileName, dataArray, delimiter=",")
    
class LogFile:
    def __init__(
        self,
        logFileName
    ):
        self.logFileName = logFileName
        try:
            self.logFile = open(self.logFileName, 'r+')
        except IOError:
            self.logFile = open(self.logFileName, 'w+')
    
    def readLastLine(self):
        lines = self.logFile.readlines()
        last_line = '\n'
        i = 1
        while last_line == '\n' or i <= len(lines):
            last_line = lines[len(lines)-i]
            i += 1
        if last_line == '\n':
            last_line = 'never\t0\tforwards'
        lastLineTuple = tuple(last_line.strip().split('\t'))
        return lastLineTuple
        
    def update(self,tupleToWrite):
        timestamp = str(datetime.datetime.now())
        logElements = []
        logElements.append(timestamp)
        for element in tupleToWrite:
           logElements.append(str(element))
           #logElements.append('\t')
        logEntry = '\t'.join(logElements)
        self.logFile.write(logEntry+'\n')
    
    def close(self):
        self.logFile.close()
    
