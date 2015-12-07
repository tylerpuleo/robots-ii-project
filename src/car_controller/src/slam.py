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

# the field of values we use for doing grid mapping
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
  gmap[inFOV[:,1] + int(y * 10) + int(cy / 2), inFOV[:,0] + int(x * 10) + int(cx / 2)] += logOdds

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

# determine the best pose given the information from last frame
def scanMatch(gmap, pose, newScan, oldScan, oldOdom):
  # posesProjected = scanMatchSample(pose, odom, samples = 1)
  # return posesProjected[0]
  # for proj in posesProjected:
  #   scanMatchProbability(gmap, proj, scan)

  posesProjected = scanMatchSample(pose, oldOdom, samples = 20)
  return scanToScanMatchProabability(pose, posesProjected, oldScan, newScan, oldOdom)

# generate new poses from the old pose and the odometry from last frame
def scanMatchSample(pose, odom, samples = 50):
  o = np.asarray([odom.linear.x, odom.angular.z], dtype = np.float32).reshape((1, 2))
  o = np.repeat(o, samples, axis = 0)

  # clamp the values to their valid ranges
  o[:,0] = np.minimum(np.maximum(o[:,0], -4.0), 4.0) / 10
  o[:,1] = np.minimum(np.maximum(o[:,1], -6.28), 6.28) / 10

  # make some noise
  stddev = np.sqrt(o[:,0]**2 + o[:,1]**2) / 3
  o[:,0] = np.random.normal(o[:,0], stddev, size = samples)
  o[:,1] = np.random.normal(o[:,1], stddev, size = samples) % 6.28

  # adjust the old pose by the noise to make a bunch of new poses
  poses = np.repeat(np.asarray(pose, dtype = np.float32).reshape((1, 3)), samples, axis = 0)
  poses[:,2] = (poses[:,2] + o[:,1]) % 6.28
  poses[:,0] += np.cos(poses[:,2]) * o[:,0]
  poses[:,1] += np.sin(poses[:,2]) * o[:,0]
  return poses

# given a map, a pose and a scan, return like probability of that pose being the
#  true pose
# rays = np.asarray([-np.pi / 4, 0, np.pi / 4])
# def scanMatchProbability(gmap, pose, scan):
#   global rays
#   testLines = pose[2] + rays
#   tx = (pose[0] + np.cos(testLines) * 5).reshape(3, 1)
#   ty = (pose[1] + np.sin(testLines) * 5).reshape(3, 1)
#   t = np.concatenate((tx, ty), axis = 1)
#   for point in t:
#     print lineit.createLineIterator((pose * 10).astype(int), (point * 10).astype(int), gmap)

# pick the poses that best explains the new scan
def scanToScanMatchProabability(oldPose, newPoses, newScan, oldScan, odom):
  # compute how much to shift the new scan by for each particle
  # dPoses = oldPose - newPoses
  dPoses = newPoses - oldPose
  dDists = np.zeros((dPoses.shape[0], 2), dtype = np.float32)
  dDists[:,0] = dPoses[:,0] * np.cos(dPoses[:,2])
  dDists[:,1] = dPoses[:,1] * np.sin(dPoses[:,2])
  dDists = dDists.reshape((dPoses.shape[0], 1, 2))

  # compute points of laser hits
  oldPoints = fov.toLocal(oldScan, np.asarray([oldPose[2]]))
  newPoints = fov.toLocal(newScan, newPoses[:,2]) + dDists

  # find distance between every point in the scan, use SSE as cost function
  diffs = np.sum(np.sqrt(np.sum((oldPoints - newPoints)**2, axis = 2))**2, axis = 1)
  diffMin = np.argmin(diffs)
  diffMax = np.argmax(diffs)

  # debug stuff
  # print diffs[diffMin], diffs[diffMax], diffs[diffMax] - diffs[diffMin]
  # img = np.zeros((512, 512, 3), dtype = np.uint8)
  # fov.draw(img, oldPoints[0], (0, 0, 255))
  # fov.draw(img, newPoints[diffMin], (255, 0, 0))
  # fov.draw(img, newPoints[diffMax], (0, 255, 0))
  # cv2.imshow('a', img)
  # cv2.waitKey(1)

  choice = newPoses[diffMin]
  return choice
