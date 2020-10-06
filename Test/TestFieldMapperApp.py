import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import ListedColormap
from FRCFieldMapper import FrcFieldMapper

#Specify file with field element locations
filename = 'C:\\Users\\timfu\\Documents\\FRC General\\Programs\\2020FieldElements.txt'

#Create a new field
fieldData = FrcFieldMapper(filename)

#Specify X and Y ranges for plotting
x = [i for i in range(0,56,1)]
y = [i for i in range(0,29,1)]

#Set colors for plot
colorvals = np.ones((10,4))
colorvals[0,0] = 0.0
colorvals[0,1] = 0.0
colorvals[0,2] = 0.0
colorvals[1,0] = 0.9
colorvals[1,1] = 0.9
colorvals[1,2] = 0.9
colorvals[2,0] = 1.0
colorvals[2,1] = 0.0
colorvals[2,2] = 0.0
colorvals[3,0] = 0.0
colorvals[3,1] = 1.0
colorvals[3,2] = 0.0
colorvals[4,0] = 0.0
colorvals[4,1] = 0.0
colorvals[4,2] = 1.0
colorvals[5,0] = 1.0
colorvals[5,1] = 1.0
colorvals[5,2] = 0.0
colorvals[6,0] = 1.0
colorvals[6,1] = 0.0
colorvals[6,2] = 1.0
colorvals[7,0] = 0.0
colorvals[7,1] = 1.0
colorvals[7,2] = 1.0
colorvals[8,0] = 0.5
colorvals[8,1] = 0.0
colorvals[8,2] = 0.0
colorvals[9,0] = 0.0
colorvals[9,1] = 0.5
colorvals[9,2] = 0.0
newcmp = ListedColormap(colorvals)

#Create a new figure with subplots
fig, (ax1, ax2) = plt.subplots(2)

#Plot the starting field map with labels and defined colors
im1 = ax1.imshow(fieldData.fieldMap, cmap=newcmp, vmin=-1, vmax=9)
# for i in y:
#     for j in x:
#         label = fieldData.fieldMap[i, j]
#         text = ax1.text(j, i, label,
#                        ha="center", va="center", color="w")
#psm1 = ax1.pcolormesh(fieldData.fieldMap, cmap=newcmp, rasterized=True, vmin=-1, vmax=9)
#fig.colorbar(cm.ScalarMappable(norm=None, cmap=newcmp), ax=ax1)

#Update the robot's position on the field
fieldData.UpdatePosition(48, 45)

#Plot the new position with labels and defined colors
im2 = ax2.imshow(fieldData.fieldMap, cmap=newcmp, vmin=-1, vmax=9)
# for i in y:
#     for j in x:
#         label = fieldData.fieldMap[i, j]
#         text = ax2.text(j, i, label,
#                         ha="center", va="center", color="w")

#Show the figure
plt.show()
