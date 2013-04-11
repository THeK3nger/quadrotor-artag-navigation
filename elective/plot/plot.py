#!/usr/env/bin python
import matplotlib.pyplot as plt

#fileInput =  open('markerLog.txt', 'r')
#fileInput =  open('plot1.txt', 'r')
fileInput =  open('plot2.txt', 'r')
x=[]
y=[]
z=[]
fig = plt.figure()
ax1 = fig.add_subplot(3,1,1)
ax2 = fig.add_subplot(3,1,2)
ax3 = fig.add_subplot(3,1,3)

for i in fileInput:
	point = [float(i.split(';')[0]),float(i.split(';')[3]),float(i.split(';')[6])]
	x.append(point)
	point = [float(i.split(';')[1]),float(i.split(';')[4]),float(i.split(';')[7])]
	y.append(point)
	point = [float(i.split(';')[2]),float(i.split(';')[5]),float(i.split(';')[8])]
	z.append(point)

ax1.plot(x)
#plt.plot(plot)
ax2.plot(y)
ax3.plot(z)
ax1.set_xlabel('time (ms)')
ax1.set_ylabel('x (meter)')

ax2.set_xlabel('time (ms)')
ax2.set_ylabel('y (meter)')

ax3.set_xlabel('time (ms)')
ax3.set_ylabel('z (meter)')

handles, labels = ax1.get_legend_handles_labels()
#left 0.5
#right 0.78
#hspace 0.30

ax2.legend( ('Extimated Postion','Actual Position','Desired Position'), bbox_to_anchor=(1.05, 1), loc=2)

plt.show()
