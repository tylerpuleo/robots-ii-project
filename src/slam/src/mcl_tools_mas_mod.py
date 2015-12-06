
# mcl_tools
#
# Some utility and visualization functions to help with
# planar Monte Carlo Localization.

import sys
import math
import time
import random
import roslib
import numpy as np


MAP_WIDTH  = 53.9
MAP_HEIGHT = 10.6
MAP_PPM    = 10.0 
LASER_MAX  = 5.0

'''
added to plot map
'''
# each cell is 10cm
CELL_HEIGHT = 10
CELL_WIDTH = 10
CELL_COUNT_WIDTH = int(MAP_WIDTH*100/CELL_WIDTH)
CELL_COUNT_HEIGHT = int(MAP_HEIGHT*100/CELL_HEIGHT)

# map
my_map = np.ones([CELL_COUNT_WIDTH,CELL_COUNT_HEIGHT],dtype=np.float64)/2;
have_updated = np.zeros([CELL_COUNT_WIDTH,CELL_COUNT_HEIGHT],dtype=np.bool);
my_map[:,50] = 1
my_map[1,:] = 0
my_map_x_pos = np.linspace(start = 0, stop = MAP_WIDTH,num=CELL_COUNT_WIDTH) / MAP_WIDTH
my_map_y_pos = np.linspace(start = 0, stop = MAP_HEIGHT,num=CELL_COUNT_HEIGHT) / MAP_HEIGHT




real_pose = (0,0,0)
best_pose = (0,0,0)

try:
    import Image
    import numpy
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    print "Error importing libraries, try"
    print
    print "    apt-get install python-opengl python-numpy"
    print
    sys.exit(1)

# Generate and sample particles.

import mcl_debug

def show_map(new_map,new_update):
    global my_map
    global have_updated
    my_map = new_map
    have_updated = new_update
    time.sleep(0.01)

def gl_display():
    global parset
    global real_pose
    global best_pose

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # display grid map martin sinclair modification
    glDisable( GL_TEXTURE_2D )
    glPointSize(1.0)
    glBegin(GL_POINTS)
    for x_index in xrange(0,CELL_COUNT_WIDTH):
        for y_index in xrange(0,CELL_COUNT_HEIGHT):
            if have_updated[x_index,y_index]:
                glColor3f(my_map[x_index,y_index], my_map[x_index,y_index], my_map[x_index,y_index])
            else:
                glColor3f(0, 0, 1)
            x = my_map_x_pos[x_index]
            y = my_map_y_pos[y_index]
            glVertex2f(x, y)
    glEnd()

    glutSwapBuffers()

def gl_idle():
    glutPostRedisplay()
    time.sleep(0.05)

def gl_click(button, state, px, py):
    if button != 0 or state != 1:
        return

    wx = px_to_wx(px)
    wy = py_to_wy(py)

    print "Click at world =", wx, wy, "; image =", px, py, "; hit =", map_hit(wx, wy)

def mcl_init(pkg_name):


    pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
    map_img = Image.open(pkg_dir + "/hallway.png")
    (ww, hh) = map_img.size


    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
    glutInitWindowSize(ww, hh)
    glutInitWindowPosition(100, 100)
    glutCreateWindow(sys.argv[0])

    gluOrtho2D(0.0, 1.0, 0.0, 1.0)

    glEnable( GL_POINT_SMOOTH );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glPointSize( 1.0 );

    glClearColor ( 0, 0, 0, 0 )
    glShadeModel( GL_SMOOTH )
    glDisable( GL_LIGHTING )
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR )
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR )
    glTexImage2D( GL_TEXTURE_2D, 0, 3, ww, hh, 0, GL_RGB, GL_UNSIGNED_BYTE, 
                  map_img.tostring("raw", "RGB", 0, -1))
    glEnable( GL_TEXTURE_2D )

    glutDisplayFunc(gl_display)
    glutIdleFunc(gl_idle)
    glutMouseFunc(gl_click)

def mcl_run_viz():
    glutMainLoop()
