#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
import time
import csv
import math

# Some OpenGL and pygame code inspired by, or borrowed from:
#   http://mattzz.no-ip.org/wiki/Projects/PlayingWithInertialMeasurementUnits
#   https://github.com/mattzzw/Arduino-mpu6050

ser = serial.Serial('/dev/tty.usbmodem1411', 115200, timeout=1)

# Initialize params
quat = True
conn = False


def resize((width, height)):
    """Resizes the GL window."""

    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    """Initializes the GL window."""

    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def drawbody(x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3):
    """Draws the passive body lines and the active body lines."""

    # Draw active and passive arm
    glLoadIdentity()
    glColor3f(1.0,0.0,1.0) 
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glVertex3f(x2,y2,-30.0)
    glVertex3f(x3,y3,-30.0)

    glVertex3f(x1,y1,-30.0)
    glVertex3f(x2,y2,-30.0)

    glVertex3f(x0,y0,-30.0)
    glVertex3f(x1,y1,-30.0)

    glVertex3f(x0,y0,-30.0)
    glVertex3f(x0-3,y0,-30.0)

    glVertex3f(x0-3,y0,-30.0)
    glVertex3f(x0-4,y0-4,-30.0)

    glVertex3f(x0-4,y0-4,-30.0)
    glVertex3f(x0-3,y0-8,-30.0)
    glEnd()

    # Draw head
    glLoadIdentity()
    glLineWidth(5.0)
    glBegin(GL_LINES)
    glVertex3f(x0,y0+2.0,-30.0)
    glVertex3f(x0+2.0,y0+4.0,-30.0)

    glVertex3f(x0,y0+2.0,-30.0)
    glVertex3f(x0-2.0,y0+4.0,-30.0)

    glVertex3f(x0,y0+6.0,-30.0)
    glVertex3f(x0-2.0,y0+4.0,-30.0)

    glVertex3f(x0,y0+6.0,-30.0)
    glVertex3f(x0+2.0,y0+4.0,-30.0)
    glEnd()

    # Draw body and legs
    glLoadIdentity()
    glLineWidth(10.0)
    glBegin(GL_LINES)
    glVertex3f(x0,y0,-30.0)
    glVertex3f(x0,-7,-30.0)

    glVertex3f(x0-5,-20,-30.0)
    glVertex3f(x0,-7,-30.0)

    glVertex3f(x0+5,-20,-30.0)
    glVertex3f(x0,-7,-30.0)
    glEnd()
         
def read_data():
    """Reads data from the serial port."""

    global quat
    ax1 = ay1 = az1 = 0.0
    ax2 = ay2 = az2 = 0.0
    ax3 = ay3 = az3 = 0.0
    qw1 = qx1 = qy1 = qz1 = 0.0
    qw2 = qx2 = qy2 = qz2 = 0.0
    qw3 = qx3 = qy3 = qz3 = 0.0

    # request data by sending a dot
    ser.write(".")
    line = ser.readline() 
    angles = line.split(",")
    try:
        if (quat == False and len(angles) >= 3):    
            ax1 = float(angles[2])
            ay1 = float(angles[1])
            az1 = float(angles[0])

            ax2 = float(angles[5])
            ay2 = float(angles[4])
            az2 = float(angles[3])

            ax3 = float(angles[8])
            ay3 = float(angles[7])
            az3 = float(angles[6])
        if (quat == True and len(angles) >= 4):
            qw1 = float(angles[0])
            qx1 = float(angles[1])
            qy1 = float(angles[2])
            qz1 = float(angles[3])
            ax1, ay1, az1 = toEulerAngle(qw1, qx1, qy1, qz1)

            qw2 = float(angles[10])
            qx2 = float(angles[11])
            qy2 = float(angles[12])
            qz2 = float(angles[13])
            ax2, ay2, az2 = toEulerAngle(qw2, qx2, qy2, qz2)

            qw3 = float(angles[20])
            qx3 = float(angles[21])
            qy3 = float(angles[22])
            qz3 = float(angles[23])
            ax3, ay3, az3 = toEulerAngle(qw3, qx3, qy3, qz3)
    except:
        pass

    return ax1, ay1, az1, ax2, ay2, az2, ax3, ay3, az3


def toEulerAngle(qw, qx, qy, qz):
    """Converts quaternion orientation to Euler angles."""

     # roll (x-axis rotation)
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

     # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

     # yaw (z-axis rotation)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz); 
    yaw = math.atan2(siny, cosy)

    ax = roll*180/math.pi
    ay = pitch*180/math.pi
    az = yaw*180/math.pi

    return ax, ay, az


def main():
    """ Main function.

    - Sets up GL window.
    - Connects to serial port.
    - Reads data in loop until broken with escape key.
    - Draws updated body each loop.
    """

    video_flags = OPENGL|DOUBLEBUF

    conn = False
    
    pygame.init()
    screen = pygame.display.set_mode((896,672), video_flags)
    pygame.display.set_caption("Press Esc to quit")
    resize((896,672))
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    
    time.sleep(1)
    
    # Connect
    while not conn:
        serin = ser.read()
        conn = True

    while conn:
        try:
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                break      

            ax1, ay1, az1, ax2, ay2, az2, ax3, ay3, az3 = read_data()

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

            # Params
            L1 = 3.5
            L2 = 4.5
            s = 0

            # Sensor 1 Position
            x0 = -2.0
            y0 = 0.0
            z0 = 0.0

            # Virtual Shoulder
            x1 = x0 + 3.0
            y1 = 0.0
            z1 = 0.0

            # Sensor 2 Position
            x2 = x1 + L1*math.cos(math.radians(-ax2))
            y2 = y1 + L1*math.sin(math.radians(-ax2))
            z2 = s

            # Sensor 3 Position
            x3 = x2 + L2*math.cos(math.radians(-ax3))
            y3 = y2 + L2*math.sin(math.radians(-ax3))
            z3 = 0.0

            drawbody(x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3)
          
            pygame.display.flip()
            frames = frames+1

        except KeyboardInterrupt:
            break

    ser.close()

if __name__ == '__main__': main()''