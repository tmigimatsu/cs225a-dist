import redis
import matplotlib.pyplot as plt
import numpy as np
import math
import string
import sys
from Tkinter import *
from time import sleep
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

r = redis.StrictRedis(host='localhost', port=6379, db=0)

# fig, ax = plt.subplots()
# plt.subplots_adjust(left=0.25, bottom=0.3)
# t = np.arange(0.0, 1.0, 0.001)
xstart = 0.0
ystart = 0.5
zstart = 0.5
xrotstart = 3.14/2;
yrotstart = 3.14/2;
zrotstart = 3.14/2;
# s = a0*np.sin(2*np.pi*f0*t)
# l, = plt.plot(t, s, lw=2, color='red')
# plt.axis([0, 0, -10, 10])

axcolor = 'lightgoldenrodyellow'
axxpos = plt.axes([0.25, 0.2, 0.65, 0.03])
axypos = plt.axes([0.25, 0.15, 0.65, 0.03])
axzpos = plt.axes([0.25, 0.1, 0.65, 0.03])

axxrot = plt.axes([0.25, 0.35, 0.65, 0.03])
axyrot = plt.axes([0.25, 0.3, 0.65, 0.03])
axzrot = plt.axes([0.25, 0.25, 0.65, 0.03])


x_pos = Slider(axxpos, 'X_pos', -1.0, 1.0, valinit=xstart)
y_pos = Slider(axypos, 'Y_pos', -1.0, 1.0, valinit=ystart)
z_pos = Slider(axzpos, 'Z_pos', -1.0, 1.0, valinit=zstart)

x_rot = Slider(axxrot, 'X_rot', -3.14, 3.14, valinit=xrotstart)
y_rot = Slider(axyrot, 'Y_rot', -3.14, 3.14, valinit=yrotstart)
z_rot = Slider(axzrot, 'Z_rot', -3.14, 3.14, valinit=zrotstart)


def update(val):
    xpos = x_pos.val
    ypos = y_pos.val
    zpos = z_pos.val
    xrot = x_rot.val
    yrot = y_rot.val
    zrot = z_rot.val
    xmat = np.matrix([[1, 0, 0], [0, math.cos(xrot), -math.sin(xrot)], [0, math.sin(xrot), math.cos(xrot)]])
    ymat = np.matrix([[math.cos(yrot), 0, math.sin(yrot)], [0, 1, 0], [-math.sin(yrot), 0, math.cos(yrot)]])
    zmat = np.matrix([[math.cos(zrot), -math.sin(zrot), 0], [math.sin(zrot), math.cos(zrot), 0], [0, 0, 1]])
    rotation = xmat * ymat * zmat
    r.set("position", str(xpos) + ", " + str(ypos) + ", " + str(zpos))
    r.set("rotation", str(rotation.item((0,0))) + " " +str(rotation.item((0,1))) + " " + str(rotation.item((0,2))) + "; " +str(rotation.item((1,0))) + " " +str(rotation.item((1,1))) + " " + str(rotation.item((1,2))) + "; " +str(rotation.item((2,0))) + " " +str(rotation.item((2,1))) + " " + str(rotation.item((2,2))))

    # l.set_ydata(amp*np.sin(2*np.pi*freq*t))
    # fig.canvas.draw_idle()
x_pos.on_changed(update)
y_pos.on_changed(update)
z_pos.on_changed(update)
x_rot.on_changed(update)
y_rot.on_changed(update)
z_rot.on_changed(update)

resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


def reset(event):
    x_pos.reset()
    y_pos.reset()
    z_pos.reset()
    x_rot.reset()
    y_rot.reset()
    z_rot.reset()
button.on_clicked(reset)

# rax = plt.axes([0.025, 0.5, 0.15, 0.15])
# radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)


# def colorfunc(label):
#     l.set_color(label)
#     fig.canvas.draw_idle()
# radio.on_clicked(colorfunc)

plt.show()

# def show_values():
#     print (w1.get(), w2.get())

# master = Tk()
# w1 = Scale(master, variable=DoubleVar, tickinterval=0.1, from_=-1, to=1, orient=HORIZONTAL)
# w1.set(0)
# w1.pack()
# w2 = Scale(master, variable=DoubleVar, from_=0, to=200, orient=HORIZONTAL)
# w2.set(23)
# w2.pack()
# # Button(master, text='Show', command=show_values).pack()



# # r = redis.StrictRedis(host='localhost', port=6379, db=0)


# mainloop()

# while(1):
# 	r.set('position', w1.get());
# 	print r.get("position")
# 	sleep(1)

# a = QApplication(sys.argv)
# window = redisGUI()
# sys.exit(a.exec_())