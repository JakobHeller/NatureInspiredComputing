# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import matplotlib.pyplot as plt
import numpy as np

path = "/home/jakob/studium/master3/NIC/NatureInspiredComputing/RBF.txt"
fileInput = []
collumn = []

with open(path, "r") as fileread:
    #read line by line
    for line in fileread:
        #extract all positive integers rom string
        fileInput.append( [int(s) for s in line.split() if s.isdigit()])
       
for iter in range (0,6):
        collumn.append ([row[iter] for row in fileInput])


print np.hstack(collumn)


f, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, sharex='col', sharey='row')
ax1.plot(collumn[0], np.zeros_like(collumn[0]),  'x', color='r')
ax2.plot(collumn[1], np.zeros_like(collumn[1]),  'x', color='r')
ax3.plot(collumn[2], np.zeros_like(collumn[2]),  'x', color='r')
ax4.plot(collumn[3], np.zeros_like(collumn[3]),  'x', color='r')
ax5.plot(collumn[4], np.zeros_like(collumn[4]),  'x', color='r')
ax6.plot(collumn[5], np.zeros_like(collumn[5]),  'x', color='r')

plt.figure(2)
plt.hist(np.hstack(collumn))