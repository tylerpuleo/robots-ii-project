#!/usr/bin/env python
#from ..world.nodes import car
from __future__ import division
import roslib; roslib.load_manifest('car_controller')
import rospy as rp
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import itertools as it
import mcl_tools_mas_mod 

np.set_printoptions(threshold='nan')

# Map dimentions meters
mapW = 54.0
mapH = 54.0

# Each cell 10cm
cellH = 10.0
cellW = 10.0

# Cells in map
cells_in_mapW = int(mapW*100/cellW)
cells_in_mapH = int(mapH*100/cellH)

# Given a cell x index it returns the distance in meters
index_to_meter_x = np.linspace(start = 0, stop = mapW, num = cells_in_mapW)
index_to_meter_y = np.linspace(start = 0, stop = mapH, num = cells_in_mapH)

# pose[2] in radians
pose = (0,0,0)

the_map = np.ones([cells_in_mapW,cells_in_mapH])/2 # initially filled with 0.5

the_map = np.log(the_map/(1-the_map)) # do log odds from Thrun book

seen = np.zeros([cells_in_mapW,cells_in_mapH],dtype=np.bool) # Once a cell has been visited update to true

# Relative angles of range finder beams
phi = np.array([math.radians(t) for t in xrange(-90,91,45)])




"""
def handle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),tf.transformations.quaternion_from_euler(0, 0, msg.theta),rp.Time.now(),"car_1","world")
"""
def laser_cb(msg):

    if msg.ranges[90] < 2:
        cmd_vel_pub.publish(angular=Vector3(z=3))
    else:
        if msg.ranges[45] < 2 and msg.ranges[135] > 2:
            cmd_vel_pub.publish(linear=Vector3(x=0.5))
            cmd_vel_pub.publish(angular=Vector3(z=1))
        elif msg.ranges[135] < 2 and msg.ranges[45] > 2:
            cmd_vel_pub.publish(linear=Vector3(x=0.5))
            cmd_vel_pub.publish(angular=Vector3(z=-1))
        else:
            cmd_vel_pub.publish(linear=Vector3(x=0.5))

    robot_cell_x = np.searchsorted(index_to_meter_x, pose[0])
    robot_cell_y = np.searchsorted(index_to_meter_x, pose[1])




    print msg.ranges[0]

    #print "x:", robot_cell_x
    #print "y:", robot_cell_y

    theta = phi + pose[2]

    for(ray_len,ray_theta) in it.izip(msg.ranges,theta):
        ray_trace = np.arange(0,ray_len + 5, 0.05)

        ray_trace_sin = ray_trace * math.sin(ray_theta)
        ray_trace_cos = ray_trace * math.cos(ray_theta)

        ray_x = ray_trace_cos + pose[0]
        ray_y = ray_trace_sin + pose[1]

        ray_points_in_play = (ray_y<mapH)&(ray_x<mapW)&(ray_x>0)&(ray_y>0)

        ray_x = ray_x[ray_points_in_play]
        ray_y = ray_y[ray_points_in_play]

        ray_trace = ray_trace[ray_points_in_play]

        valid_ray_points = np.vstack((ray_x,ray_y)).T

        cell_x_floor_idx = np.searchsorted(index_to_meter_x,ray_x)
        cell_y_floor_idx = np.searchsorted(index_to_meter_y,ray_y)

        valid_cels_idx = np.vstack((cell_x_floor_idx,cell_y_floor_idx)).T

        cell_x_floor = index_to_meter_x[cell_x_floor_idx]
        cell_y_floor = index_to_meter_y[cell_y_floor_idx]

        valid_cells = np.vstack((cell_x_floor,cell_y_floor)).T

        ray_to_cg_del = valid_cells - valid_ray_points

        P_occ = 0.7
        P_free = 0.3
        d_free = 0.5
        d_over = 0.5

        if ray_len >=5:
            ray_len = 20

        free_zone = ray_trace<ray_len - d_free
        unknown_zone = ray_trace > ray_len + d_over
        approach_zone = (ray_trace<ray_len) & np.logical_not(free_zone)
        recede_zone = (ray_trace>ray_len) & np.logical_not(unknown_zone)

        p_cell_g_pose_scan = np.empty_like(ray_trace)
        p_cell_g_pose_scan[free_zone] = P_free
        p_cell_g_pose_scan[unknown_zone] = 0.5
        p_cell_g_pose_scan[approach_zone] = P_free=(P_occ-P_free) * (ray_trace[approach_zone] + d_free- ray_len)/d_free
        p_cell_g_pose_scan[recede_zone] = P_occ + (0.5-P_occ) * (ray_trace[recede_zone] - ray_len)/d_over

        log_odds = np.log(p_cell_g_pose_scan/(1- p_cell_g_pose_scan ))
        seen[valid_cels_idx[:,0],valid_cels_idx[:,1]] = True
        the_map[valid_cels_idx[:,0],valid_cels_idx[:,1]] = the_map[valid_cels_idx[:,0],valid_cels_idx[:,1]] + p_cell_g_pose_scan

    print seen[50]
    mcl_tools_mas_mod.show_map(1/(1+np.exp(the_map)),seen)
    

    """
    for (position,value) in np.ndenumerate(the_map):
        if math.sqrt((position[0]-robot_cell_x)**2 + (position[1]-robot_cell_y)**2) < 5:
            1
            # Process
            # the_map[x][y] = the_map[x][y] + inverse_sensor_model
        # else
            # keep the same
    """
    # Occupancy Grid Mapping
    # For cells in map:
      # If cell is in laser view
        # Lt,i = Lt-1,i + inverse sensor model(cell,position,sensor reading) - L0       L is log odds
      # else
        # Keep same (Lt,i = Lt-1,i)



def main():
    rp.init_node('car_1', anonymous=True)
    rp.Subscriber('/car_1/scan', LaserScan, laser_cb, queue_size=1)
    #rp.Subscriber('/robot_0/base_pose_ground_truth',turtlesim.msg.Pose,handle_pose)

    global cmd_vel_pub
    cmd_vel_pub = rp.Publisher('/car_1/velocity', Twist, queue_size=1)
    mcl_tools_mas_mod.mcl_init("slam")
    mcl_tools_mas_mod.gl_display()
    rp.spin()
if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass