import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

fig,ax = plt.subplots()
ax.set_xlim([-10,10])
ax.set_ylim([-10,10])
width =2 
height = 0.5
Beamangle1 =180
x1,y1 = 0,0
ellipse1 = Ellipse(xy=(x1+0.075, y1), width=width, height=height, angle=Beamangle1, facecolor="yellow")
ax.add_patch(ellipse1)
plt.draw()
plt.show()
