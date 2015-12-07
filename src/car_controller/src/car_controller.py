#!/usr/bin/env python
#from ..world.nodes import car
from __future__ import division
import cv2
import numpy as np
import roslib; roslib.load_manifest('car_controller')
import rospy as rp
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import slam

oldOdom = None
oldScan = None
pose = (0, 0, 0)
gmap = np.ones((1100, 1100), np.float32) * 0.5

def drive(msg):
  global cmd_vel_pub
  minRange = min(msg.ranges[85], msg.ranges[95])
  control = Twist()
  if minRange < 5:
    control.angular.z = 2
  else:
    control.linear.x = 2
  cmd_vel_pub.publish(control)
  return control

def laser_cb(msg):
  global oldOdom, oldScan, pose, gmap
  control = drive(msg)
  if oldOdom != None:
    newPose = slam.scanMatch(gmap, pose, msg.ranges, oldScan, oldOdom)
    slam.gridMapping(gmap, newPose, msg.ranges)
    pose = (newPose[0], newPose[1], newPose[2])
    # slam.gridMapping(gmap, pose, msg.ranges)
  oldOdom = control
  oldScan = msg.ranges

# only used for debugging purposes
# gets the current pose and uses it in mapping
def pose_cb(msg):
  global pose

  # convert msg.pose.pose.orientation to an euler angle
  quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)

  # create the pose
  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y
  t = euler[2]

  pose = (x, y, t)

def main():
  rp.init_node('aaa', anonymous=True)
  rp.Subscriber('/car_1/scan', LaserScan, laser_cb)
  # rp.Subscriber('/robot_0/base_pose_ground_truth', Odometry, pose_cb)

  global cmd_vel_pub
  cmd_vel_pub = rp.Publisher('/car_1/velocity', Twist, queue_size=1)

  rp.spin()

if __name__ == '__main__':
  try:
    main()
  except rp.ROSInterruptException:
    pass
