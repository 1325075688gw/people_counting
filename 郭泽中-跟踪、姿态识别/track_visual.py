from matplotlib import pyplot as plt
import numpy as np
import math

def visual(ax,locations,gestures,frame):
    angle = np.arange(math.pi / 4, math.pi *3/ 4, 0.1)

    x = np.cos(angle)*8
    y = np.sin(angle)*8

    x=np.insert(x,0,0)
    y=np.insert(y,0,0)
    x=np.insert(x,len(x),0)
    y=np.insert(y,len(y),0)

    ax.plot(x, y)

    ax.set_xlim(-4,4)
    ax.set_ylim(0,8)

    ax.set_title('当前帧有'+str(len(gestures))+'个人')
    ax.plot(0,0,'o',markersize=40)

    for person in locations:
        location=locations[person]
        gesture=gestures[person]

        ax.plot(location[0],location[1],'o',markersize=40)
        ax.text(location[0],location[1],str(gesture))