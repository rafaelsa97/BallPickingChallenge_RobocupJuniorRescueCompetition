from packages import initialization
from packages import pioneer3dx as p3dx
p3dx.init()

%matplotlib inline
import matplotlib.pyplot as plt

import cv2
import numpy

MIN_WALL_THRESHOLD = 0.1 
MAX_WALL_THRESHOLD = 0.2
DEF_X_SPEED = 0.2 
DEF_YAW_SPEED = 0.15

def is_obstacle_detected():
    if p3dx.distance[3] and p3dx.distance[4] >= 0.5:
        return False
    else:
        return True

def is_green_square_detected():
    image = p3dx.image
    plt.imshow(image);
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_line = numpy.array([37, 50, 50])
    upper_line = numpy.array([75, 255, 255])
    mask = cv2.inRange(hsv, lower_line, upper_line)
    plt.imshow(mask,cmap='gray');
    mask[0:80, 0:150] = 0
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return False
    else:
        return True
    
def green_square_detected():
    hsv = cv2.cvtColor(p3dx.image, cv2.COLOR_RGB2HSV)
    lower_line = numpy.array([37, 50, 50])
    upper_line = numpy.array([170, 255, 255])
    mask = cv2.inRange(hsv, lower_line, upper_line)
    plt.imshow(mask,cmap='gray');
    mask[0:80, 0:150] = 0
    plt.imshow(mask,cmap='gray');
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    axes = plt.gca()
    axes.add_artist(plt.Circle((cx,cy),10,color='r'));
    cx_purple, cy_purlpe = line_centroid(p3dx.image)
    if cx > cx_purple:
        move(DEF_X_SPEED,-DEF_YAW_SPEED * 6)
    else:
        move(DEF_X_SPEED,DEF_YAW_SPEED * 6)

def follow_line():
    kp = 0.1
    width = p3dx.image.shape[1]
    print('Following the line')
    while not (is_obstacle_detected()):
        if is_green_square_detected():
            green_square_detected()
        cx, cy = line_centroid(p3dx.image)
        err = cx - (width/2)
        linear = 2
        angular = -kp*err
        move(linear, angular)
        while not is_line_detected():
            p3dx.move(2.0,2.0)
    print('Obstacle detected')
    
def is_line_detected():
    image = p3dx.image
    plt.imshow(image);
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_line = numpy.array([135, 50, 50])
    upper_line = numpy.array([170, 255, 255])
    mask = cv2.inRange(hsv, lower_line, upper_line)
    plt.imshow(mask,cmap='gray');
    mask[0:80, 0:150] = 0
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return False
    else:
        return True
    
def line_centroid(image):
    plt.imshow(image);
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_line = numpy.array([135, 50, 50])
    upper_line = numpy.array([170, 255, 255])
    mask = cv2.inRange(hsv, lower_line, upper_line)
    plt.imshow(mask,cmap='gray');
    mask[0:80, 0:150] = 0
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    plt.imshow(image)
    axes = plt.gca()
    axes.add_artist(plt.Circle((cx,cy),10,color='r'));
    return cx, cy

def getWall():
    rightSide, frontSide = getSonars()
    while (p3dx.distance[3] and p3dx.distance[4]) > MAX_WALL_THRESHOLD:       
        move(DEF_X_SPEED,0)
        rightside, frontSide = getSonars()
        
    while p3dx.distance[0] > MAX_WALL_THRESHOLD:
        if (p3dx.distance[3] and p3dx.distance[4]) < min(p3dx.distance[:2]):
            yawSpeed = -DEF_YAW_SPEED * 3
        else:
            yawSpeed = -DEF_YAW_SPEED
        move(0,yawSpeed)
        rightside, frontSide = getSonars()
    move(0,0)
    
def follow_wall():
    print('Following the wall')
    while not is_line_detected():
        rightSide, frontSide = getSonars()
        xSpeed = DEF_X_SPEED
        yawSpeed = 0
        if (p3dx.distance[3] and p3dx.distance[4]) < MIN_WALL_THRESHOLD:
            xSpeed = 0.1
            yawSpeed = DEF_YAW_SPEED * 4
        else:
            if min(p3dx.distance[:2]) < MIN_WALL_THRESHOLD:
                xSpeed  = DEF_X_SPEED * 0.5
                yawSpeed = DEF_YAW_SPEED * 4
            else:
                if min(p3dx.distance[:2]) > MAX_WALL_THRESHOLD:
                    xSpeed  = DEF_X_SPEED * 0.5
                    yawSpeed = DEF_YAW_SPEED * 8
        # Move the robot
        move(xSpeed,yawSpeed)
    print('Line detected')
    
def getLine():
    while not is_line_detected():
        move(2,-5)
    p3dx.stop()
    
def move(V_robot,w_robot):
    r = 0.1953 / 2
    L = 0.33
    w_r = (2 * V_robot + L * w_robot) / (2*r)
    w_l = (2 * V_robot - L * w_robot) / (2*r)
    p3dx.move(w_l, w_r)
    
def getSonars(): 
    rightSide = p3dx.distance[:2]
    frontSide = p3dx.distance[3:5] 
    return rightSide, frontSide



p3dx.tilt(-0.47)
try:
    while True:
        follow_line()
        getWall()
        follow_wall()
        getLine()
except KeyboardInterrupt:
    move(0,0)
