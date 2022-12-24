from glob import glob
from djitellopy import tello
import keyPressedModule as kp
import numpy as np

import time
import cv2
import math

### PARAMETERS ###
fSpeed = 117/10 # Forward Speed (cm/s)
aSpeed = 360/10 # Angular Speed (deg/s)
interval = 0.25

dInterval = fSpeed*interval
aInterval = aSpeed*interval
##################
x, y = 250, 250
a = 0
yaw = 0

kp.init()

# 1280 x 720 for 720p resolution
me = tello.Tello()
me.connect()
me.streamon()

global img, img2

battery = me.get_battery
print(me.get_battery())

points = [(0, 0), (0, 0)]

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 70
    aspeed = 50
    global yaw, x, y, a
    d = 0

    if kp.getKey("LEFT"): 
        lr = -speed
        d = dInterval
        a = -180
    elif kp.getKey("RIGHT"): 
        lr = speed
        d = -dInterval
        a = 180

    if kp.getKey("UP"): 
        fb = speed
        d = dInterval
        a = 270
    elif kp.getKey("DOWN"): 
        fb = -speed
        d = -dInterval
        a = -90

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): 
        yv = -aspeed
        yaw -= aInterval
    elif kp.getKey("d"): 
        yv = aspeed
        yaw += aInterval

    if kp.getKey("r"): me.takeoff()
    elif kp.getKey("f"): me.land()

    time.sleep(interval)
    a += yaw
    x += int(d*math.cos(math.radians(a)))
    y += int(d*math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y, battery]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, (point), 5, (0, 255, 0), cv2.FILLED)
    cv2.circle(img, points[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]- 500)/100}), ({(points[-1][1]- 500)/100})m',
                (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 0, 255), 1)

while True:
    img2 = me.get_frame_read().frame
    img2 = cv2.resize(img2, (500, 500))
    cv2.imshow("VIDEO", img2)

    value = getKeyboardInput()
    me.send_rc_control(value[0], value[1], value[2], value[3])

    img = np.zeros((500, 500, 3), np.uint8)
    
    if (points[-1][0] != value[4] or points[-1][1] != value[5]):
        points.append((value[4], value[5]))
    drawPoints(img, points)


    cv2.imshow("OUTPUT", img)
    cv2.waitKey(1)