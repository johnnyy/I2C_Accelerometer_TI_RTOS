import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from matplotlib import gridspec
import serial
import math

ser = serial.Serial('/dev/ttyUSB0', 115200)




def isnumber(value):
    try:
         float(value)
    except ValueError:
         return False
    return True






fig = plt.figure(figsize=(15,9))
gs = gridspec.GridSpec(2, 2, height_ratios=[1, 1],width_ratios=[1,1]) 
ax = fig.add_subplot(gs[0])
ay = fig.add_subplot(gs[2])
aa = fig.add_subplot(gs[3])
a3 = fig.add_subplot(gs[1], projection='3d')


ax.set_ylim(-2,2)
ay.set_ylim(-30,30)


aa.set_ylim(-360,360)


xtime = []

xs = []
ys = []
zs = []

xf = []
yf = [] 
zf = []

xa = []
ya = []
za = []

xp = []
yp = [] 
zp = []


i = 0

gray_patch = mpatches.Patch(color='gray', label="X")
blue_patch = mpatches.Patch(color='blue', label="Y")
green_patch = mpatches.Patch(color='green', label="Z")

def animate(i, xtime, xs, ys, zs, xf, yf, zf,xa, ya, za, xp, yp, zp):



    atual = []
    while(True):
        data = ser.readline()
        if isnumber(data[:-2].decode("utf-8")):
           atual.append(float(data[:-2].decode("utf-8")) / 1000.0)
          # print(atual)
           for i in range(11):
               data = ser.readline()
               atual.append(float(data[:-2].decode("utf-8")) / 1000.0)
           break
    



    
    print(atual)
    print("sum Acc: ",math.sqrt(atual[0]**2+ atual[1]**2+atual[2]**2))
    
    xs.append(atual[0])
    ys.append(atual[1])
    zs.append(atual[2])

    xf.append(atual[3])
    yf.append(atual[4])
    zf.append(atual[5])

    xp.append(atual[6])
    yp.append(atual[7])
    zp.append(atual[8])

    xa.append(atual[9])
    ya.append(atual[10])
    za.append(atual[11])




    xtime.append(dt.datetime.now().strftime('%H:%M:%S'))
    
    xtime = xtime[-8:]
    xs = xs[-8:]
    ys = ys[-8:]
    zs = zs[-8:]    

    xf = xf[-8:]
    yf = yf[-8:]
    zf = zf[-8:]
    
    xp = xp[-8:]
    yp = yp[-8:]
    zp = zp[-8:]

    xa = xa[-8:]
    ya = ya[-8:]
    za = za[-8:]

    ax.clear()
    ay.clear()
    a3.clear()
    aa.clear()

    ax.plot(xtime, xs,color='gray',marker='o')
    ax.plot(xtime, ys,color='blue',marker='o')
    ax.plot(xtime, zs,color='green',marker='o')

    ay.plot(xtime, xf,color='gray',marker='o')
    ay.plot(xtime, yf,color='blue',marker='o')
    ay.plot(xtime, zf,color='green',marker='o')
    
    aa.plot(xtime, xa,color='gray',marker='o')
    aa.plot(xtime, ya,color='blue',marker='o')
    aa.plot(xtime, za,color='green',marker='o')
    
    a3.scatter(xp, yp, zp, marker='o')
    
    ax.set_ylim(-2,2)
    ay.set_ylim(-30,30)
    aa.set_ylim(-360,360)
      




    ax.label_outer()

    ax.set(ylabel='Aceleração')
    ax.legend(handles=[gray_patch,blue_patch,green_patch]);
    



    ay.label_outer()
    ay.set(ylabel='Velocidade de Giro')
    ay.legend(handles=[gray_patch,blue_patch,green_patch]);
    
    aa.label_outer()


    aa.set(ylabel='Angulo')
    aa.legend(handles=[gray_patch,blue_patch,green_patch]);

    a3.set_xlabel('X Label')
    a3.set_ylabel('Y Label')
    a3.set_zlabel('Z Label')
    #a3.set_xlim(-1,1)
   # a3.set_ylim(-1,1)
   # a3.set_zlim(-1,1)

ani = animation.FuncAnimation(fig, animate, fargs=(xtime, xs, ys, zs, xf, yf, zf, xa, ya, za, xp, yp, zp), interval=500)
plt.show()
