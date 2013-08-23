# -*- coding: utf-8 -*-
"""
Created on Fri Aug 16 11:48:45 2013

@author: Peter
"""
import time

filename = 'log001.bin'
# filename = 'conv.zip'

import serial
ser = serial.Serial()
ser.port = 1
ser.baudrate = 921600 #480e6
ser.timeout = 1
ser.open()

serline = ''
while (len(serline) == 0):
    ser.write('\n')
    serline = ser.readlines()
    for sl in serline:
        print sl

serline = ser.readlines()   
ser.write('sdlogimu stop\n') 
ser.write('nocrlf\n')
serline = ser.readlines()   
ser.write('ls /fs/microsd\n')
sessdirs = ser.readlines()
print 'Directories available: \n'
for sd in sessdirs: 
    print sd
sessdir = input('Enter Name of Directory to Use (in quotes):  ')
print 'Directory chosen: ' + sessdir


# Now get that file to dump with "cat"
ser.write('\n')
command = ser.readline() # echo line
ser.write('cat /fs/microsd/'+sessdir+'/'+filename+'\n')
command = ser.readline()  # cat print of what it is doing
starttime = time.time(); 
filelines = ser.readlines() # actually get the whole dump 

transferendtime = time.time(); 
transfertime = transferendtime-starttime; 
print "Transfer Time: " + str(transfertime)

# now write to file
directory = os.getcwd()+'\\'+sessdir 
if not os.path.isdir(directory):
    os.makedirs(directory)
fw = open(directory+'\\'+filename,'wb')
nfln = range(len(filelines))
for ii in nfln:
    if ii == nfln[-1]:
        fw.write(filelines[ii][:-8])
    else:
        fw.write(filelines[ii])
        
fw.close()

ser.close()

filewriteendtime = time.time(); 
filewritetime = filewriteendtime-transferendtime; 
print "File Write Time: " + str(filewritetime)

