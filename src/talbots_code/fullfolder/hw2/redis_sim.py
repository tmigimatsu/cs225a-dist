import redis
import math
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider, Button

r = redis.StrictRedis(host='localhost', port=6379, db=0)

xstart = 0.0
ystart = 0.5
zstart = 0.5

xrotstart = np.pi/2;
# Due to singularity of rotation when y = np.pi/2, yrot should always be 0
# We don't really need y rotation control since x & z take care of planar rotation
yrotstart = 0;
zrotstart = np.pi/2;

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

x_rot = Slider(axxrot, 'X_rot', -np.pi, np.pi, valinit=xrotstart)
y_rot = Slider(axyrot, 'Y_rot', -np.pi, np.pi, valinit=yrotstart)
z_rot = Slider(axzrot, 'Z_rot', -np.pi, np.pi, valinit=zrotstart)


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
    
    # Rotate by y, then x, then z
    rotation = zmat * xmat * ymat
    r.set("position", str(xpos) + ", " + str(ypos) + ", " + str(zpos))
    r.set("rotation", str(rotation.item((0,0))) + " " +str(rotation.item((0,1))) + " " + str(rotation.item((0,2))) + "; " +str(rotation.item((1,0))) + " " +str(rotation.item((1,1))) + " " + str(rotation.item((1,2))) + "; " +str(rotation.item((2,0))) + " " +str(rotation.item((2,1))) + " " + str(rotation.item((2,2))))

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

plt.show()
