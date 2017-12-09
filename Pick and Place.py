import packages.initialization
import packages.pioneer3dx as p3dx
p3dx.init()
import cv2
import numpy
%matplotlib inline
import matplotlib.pyplot as plt
lower_blue = numpy.array([110,  50,  50])
upper_blue = numpy.array([130, 255, 255])
lower_red = numpy.array([ 0, 140, 140]) 
upper_red = numpy.array([ 10, 255, 255])

def color_ball():
    hsv = cv2.cvtColor(p3dx.image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    M = cv2.moments(mask)
    area = M['m00']
    if area > 0:
        cx = int(M['m10']/area)
        cy = int(M['m01']/area)
    else:
        cx = None
        cy = None
    return area, cx, cy

def color_ball_2():
    hsv = cv2.cvtColor(p3dx.image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask[0:80, 0:150] = 0
    M = cv2.moments(mask)
    area = M['m00']
    if area > 0:
        cx = int(M['m10']/area)
        cy = int(M['m01']/area)
    else:
        cx = None
        cy = None
    return area, cx, cy

def is_ball_centered():
    area, cx, cy = color_ball()
    if area > 0 and cx >= 70 and cx < 80:
        return True
    else:
        return False
    
def is_ball_close():
    area, cx, cy = color_ball()
    if area > 0 and cy >= 90:
        return True
    else:
        return False

def is_ball_close_2():
    area, cx, cy = color_ball_2()
    if area > 0 and cy >= 90:
        return True
    else:
        return False
    
def color_square():
    hsv = cv2.cvtColor(p3dx.image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    M = cv2.moments(mask)
    area = M['m00']
    if area > 0:
        cx = int(M['m10']/area)
        cy = int(M['m01']/area)
    else:
        cx = None
        cy = None
    return area, cx, cy

def is_square_centered():
    area, cx, cy = color_square()
    if area > 0 and cx >= 70 and cx < 80:
        return True
    else:
        return False
    
def is_square_close():
    area, cx, cy = color_square()
    if area > 0 and cy >= 90:
        return True
    else:
        return False

def search_ball():
    p3dx.tilt(-0.35)
    p3dx.gripper(0.05,0.1) 
    p3dx.sleep(2)
    while not is_ball_centered():
        p3dx.move(-0.5,0.5)
    p3dx.stop()
    print("Ball Founded!!")
    p3dx.tilt(-0.47)
    p3dx.sleep(0.5)
    kp = 0.1
    width = p3dx.image.shape[1]
    while not is_ball_close_2():
        area, cx, cy = color_ball()
        if cx > 80:
            p3dx.move(2.0,1.5)
        elif cx < 70:
            p3dx.move(1.5,2.0)
        else:
            p3dx.move(2.0,2.0)
    p3dx.stop()

def catch_ball():
    p3dx.tilt(-0.55)
    p3dx.sleep(1)
    while not is_ball_centered():
        area, cx, cy = color_ball()
        if cx >=80:
            p3dx.move(0.5,-0.5)
        else:
            p3dx.move(-0.5,0.5)
    p3dx.stop()
    while not is_ball_close_2():
        p3dx.move(0.5,0.5)
    p3dx.stop()
    p3dx.sleep(2)
    p3dx.gripper(0.05,0.0)
    p3dx.sleep(2)
    p3dx.gripper(0.00,0.0)
    p3dx.sleep(2)
    print("Ball Catched!!")
    
def search_square():
    p3dx.tilt(0.0)
    while not is_square_centered():
        p3dx.move(-0.5,0.5)
    print("Square Founded!!")
    p3dx.stop()
    p3dx.sleep(2)
    p3dx.tilt(-0.47)
    while not is_square_close():
        p3dx.move(2.0,2.0)
    p3dx.stop()
    
def drop_square():
    p3dx.sleep(3)
    p3dx.gripper(0.05,0.0)
    p3dx.sleep(3)
    p3dx.gripper(0.0,0.05)
    p3dx.sleep(3)
    p3dx.move(-1.0,-1.0)
    p3dx.sleep(3)
    p3dx.move(-1.0,1.0)
    p3dx.sleep(4)
    p3dx.stop()
    print("Ball Dropped in the Square!!")


try:
    c = 0
    while c < 5: 
        search_ball()
        catch_ball()
        search_square()
        drop_square()
        c = c + 1
        
except KeyboardInterrupt:
    p3dx.move(0,0)

