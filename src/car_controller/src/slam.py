#!/usr/bin/env python

from __future__ import division

import cv2
import numpy as np

import fov

# makes an array of cartesian coordinates over the given arrays
def cartesian2(arrays):
  arrays = [np.asarray(a) for a in arrays]
  maxes = [np.amax(a) for a in arrays]
  shape = (len(x) for x in arrays)

  ix = np.indices(shape, dtype = np.float32)
  ix = ix.reshape(len(arrays), -1).T

  for n, arr in enumerate(arrays):
    ix[:, n] = arrays[n][ix[:, n].astype(np.int)] / maxes[n]

  return ix

inSensor = cartesian2((range(-50, 51), range(-50, 51))) * 50

# update a map given a scan
def gridMapping(gmap, pose, scan):
  x, y, t = pose

  # determine which points are in the FOV of the sensor
  inFOV = isInSensor(pose[2])

  # run the inverse sensor model on the points
  logOdds = inverseSensor(inFOV, pose, scan)

  # set the appropriate points on the map
  cy, cx = gmap.shape
  gmap[inFOV[:,1] + int(y * 10) + cy, inFOV[:,0] + int(x * 10) + cx] += logOdds

  cv2.imshow('a', cv2.flip(gmap, 0))
  cv2.waitKey(1)

# given an angle, return an array of points in the grid map relative to the pose
#   that are in the field of view
def isInSensor(t):
  global inSensor
  rPos = np.asarray([50 * np.cos(t), 50 * np.sin(t)], dtype = np.float32)
  rDist = 50
  dDist = np.sqrt(inSensor[:,0]**2 + inSensor[:,1]**2)
  angle = np.arccos((inSensor[:,0] * rPos[0] + inSensor[:,1] * rPos[1]) / (rDist * dDist))
  test = np.logical_and(angle < np.pi / 2, dDist < 50)
  return inSensor[test].astype(int)

# given points in a scan, a pose and a scan, determine the associate log-odds
#   probability of that grid cell being either occupied (0) or unoccupied (1)
def inverseSensor(points, pose, scan):
  x, y, t = pose

  # associating every point in points with a distance to the origin of the scan
  #   and the sensor beam
  r = np.sqrt(np.sum(points**2, axis = 1))
  phi = (np.arctan2(points[:,1], points[:,0]) - t) + np.pi/2
  phi %= np.pi
  k = np.floor(phi * 180 / np.pi).astype(int)
  senses = (np.asarray(scan) * 10)[k]

  # this isn't exactly how you're supposed to do log odds, but it's a good enough
  #   approximation that it works just as well for the cases we need it to
  logOdds = np.zeros(r.shape)
  logOdds[np.logical_and(senses < 50, np.abs(r - senses) < 5)] = -0.2
  logOdds[r <= senses] = 0.2
  return logOdds

# give a new scan, old scan, and odometry; return a probability of new scan
#  given old scan and odometry
def scanMatch(scan, odom):
  1
# given some odometry and new/old scan data, determine the pose the maximizes
#  the probability of receiving the new scan
def scanMatchSample(newScan, oldScan, odometry):
  1

# determines p(z_t | x_t, m_t-1) p(x_t | u_t-1, x*_t-1)
def scanMatchProbability(oldXY, newXY):
  1
