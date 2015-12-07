#!/usr/bin/python
 
import cv2
import numpy as np
 
# scanMsg: array of scans
# thetas: angle of the robot
# halfRange: half of the range of the sensor
def toLocal(scan, theta, halfRange = np.pi / 2):
  scanCount = len(scan)
  #scans = np.linspace(3, 5, num = scanCount, endpoint = True).reshape((scanCount, 1))
  scans = np.asarray(scan).reshape((scanCount, 1))
  angle = np.linspace(-halfRange, halfRange, num = scanCount, endpoint = True).reshape((scanCount, 1))

  vX = (np.cos(angle + theta) * scans).reshape((scanCount, theta.shape[0], 1))
  vY = (np.sin(angle + theta) * scans).reshape((scanCount, theta.shape[0], 1))
  laser = np.concatenate((vX, vY), axis = 2)

  return np.swapaxes(laser, 0, 1)

def draw(img, laser, color):
  for x, y in laser:
    newX = int((x * 50) + 256)
    newY = int((y * 50) + 256)
    cv2.circle(img, (newX, newY), 1, color)
