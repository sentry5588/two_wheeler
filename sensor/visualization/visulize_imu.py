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
    K_x_pressed = True
    K_y_pressed = True
    K_z_pressed = True
    x = 0
    y = 0
    z = 0
    while i < 10000:
        for event in pygame.event.get():
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.quit()
                quit()
            elif event.type == KEYDOWN and event.key == K_x:
                if K_x_pressed == False:
                    K_x_pressed = True
                else:
                    K_x_pressed = False
            elif event.type == KEYDOWN and event.key == K_y:
                if K_y_pressed ==False:
                    K_y_pressed = True
                else:
                    K_y_pressed = False
            elif event.type == KEYDOWN and event.key == K_z:
                if K_z_pressed == False:
                    K_z_pressed = True
                else:
                    K_z_pressed = False

        arduino_data = ser.readline() # read data from Arduino via serial
        data_ang_p = parse_values(arduino_data) # parse data into list
        x_pre = x
        y_pre = y
        z_pre = z
        x = data_ang_p[0] # x rotation deg/s
        y = data_ang_p[1] # y rotation deg/s
        z = -data_ang_p[2] # z rotation deg/s, mirror the display
        print("x: {0:4.4f}\t".format(x), \
              "y: {0:4.4f}\t".format(y), \
              "z: {0:4.4f}\t".format(z), \
              "time: ", time.time())
    
        if K_x_pressed == True: # pygame axis is different from MPU
            glRotatef(y-y_pre, 1, 0, 0) 
        if K_y_pressed == True: # pygame axis is different from MPU
            glRotatef(x-x_pre, 0, 1, 0) 
        if K_z_pressed == True:
            glRotatef(z-z_pre, 0, 0, 1)
                
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        pygame.display.flip()
        pygame.time.wait(pygame_update_time_step)
        i = i + 1
        
    pygame.quit()
    quit()
                
main()


