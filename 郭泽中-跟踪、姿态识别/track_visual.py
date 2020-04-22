from matplotlib import pyplot as plt
import numpy as np
import math

def visual(locations,gestures,cluster_num):
    angle = np.arange(math.pi / 6, math.pi *5/ 6, math.pi/60)

    x = np.cos(angle)*8
    y = np.sin(angle)*8

    x=np.insert(x,0,0)
    y=np.insert(y,0,0)
    x=np.insert(x,len(x),0)
    y=np.insert(y,len(y),0)

    plt.plot(x, y)

    plt.xlim(-4*math.sqrt(3),4*math.sqrt(3))
    plt.ylim(0,8)

    plt.title('当前帧有'+str(len(gestures))+'个人,当前帧聚出了'+str(cluster_num)+'类')
    plt.plot(0,0,'o',markersize=40)

    for person in locations:
        location=locations[person]
        gesture=gestures[person]

        plt.plot(location[0],location[1],'o',markersize=40)
        plt.text(location[0],location[1],str(gesture))

    plt.pause(0.0001)
    plt.cla()