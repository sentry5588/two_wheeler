# -*- coding: utf-8 -*-
"""
Created on Sat Mar 30 19:07:37 2019
@author: sentry5588
A very crude way to visualize the computed robot position from Arduino
This script is created for troubleshooting purpose
"""

import pygame, sys, time
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial

# initialize serial port
ser = serial.Serial('COM21', baudrate = 115200, timeout = 1)

# parse the data into specific format
def parse_values(arduino_serial_output):
    words = arduino_serial_output.decode("utf-8").split(", ")
    print(words)
    # data format: [x, y, z, current_time]
    data_ang_p = [float(w) for w in words]
    return data_ang_p
    
# define verticies using tuples
verticies = (
     (2, -0.1, -0.5),
     (2, 0.1, -0.5),
     (-2, 0.1, -0.5),
     (-2, -0.1, -0.5),
     (2, -0.1, 0.5),
     (2, 0.1, 0.5),
     (-2, -0.1, 0.5),
     (-2, 0.1, 0.5)
     )

# edges tuples
edges = (
     (0, 1),
     (0, 3),
     (0, 4),
     (2, 1),
     (2, 3),
     (2, 7),
     (6, 3),
     (6, 4),
     (6, 7),
     (5, 1),
     (5, 4),
     (5, 7)
     )

def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()
    
def main():
    pygame_update_time_step = 10 # 10 ms = 0.01 s
    pygame.init()
    display = (800, 600)
    
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.9, 50.0)
    glTranslatef(0.0, 0.0, -5)
    glRotatef(0, 0, 0, 0)
        
    i = 0
    x = 0 # initialize rotate position in x direction
    y = 0 # initialize rotate position in y direction
    z = 0 # initialize rotate position in z direction
    while i < 10000:
        for event in pygame.event.get():
            # close the graphic if escape key is pressed
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.quit()
                quit()
        
        arduino_data = ser.readline() # read data from Arduino via serial
        data_ang_p = parse_values(arduino_data) # parse data into list
        x = data_ang_p[0] # x rotation deg/s
        y = data_ang_p[1] # y rotation deg/s
        z = -data_ang_p[2] # z rotation deg/s, mirror the display
        print("x: {0:4.4f}\t".format(x), \
              "y: {0:4.4f}\t".format(y), \
              "z: {0:4.4f}\t".format(z), \
              "time: ", time.time())
        
        # paired with glPopMatrix(), preparing to restore the object to the 
        # original position after displaying the object
        glPushMatrix()
        glRotatef(y, 1, 0, 0)
        glRotatef(x, 0, 1, 0)
        glRotatef(z, 0, 0, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        pygame.display.flip()
        # paired with glPushMatrix(), to restore object to the original
        # position afte displaying the object
        glPopMatrix()
        
        pygame.time.wait(pygame_update_time_step)
        i = i + 1
        
    pygame.quit()
    quit()
                
main()
