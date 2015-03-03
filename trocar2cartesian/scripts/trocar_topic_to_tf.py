#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf
import numpy as np
from tf.transformations import *
from geometry_msgs.msg import Pose
from trocar2cartesian_msgs.msg import TrocarPose
from math import pi, sin, cos

trocar_tf = None
tf_broadcaster = None


def spherical2cartesianpose(r, theta, phi):
  translation = [r * sin(theta) * cos(phi),
                 r * sin(theta) * sin(phi),
                 r * cos(theta)]
  direction = np.eye(4)
  vecUp = np.array([0, 0, -1])
  vecZ = np.array(translation) # forward
  if (vector_norm(vecZ) == 0):
    print('Error |vecZ| == 0')
    return None
  vecZ /= vector_norm(vecZ)
  vecX = np.cross(vecUp, vecZ) # right
  vecX /= vector_norm(vecX)
  vecY = np.cross(vecZ, vecX) # down
  vecY /= vector_norm(vecY)
  direction[0:3,0:3] = np.transpose(np.array((vecX, vecY, vecZ)))
  pose = concatenate_matrices(translation_matrix(translation), direction)
  return pose


def homogeneous2translation_quaternion(homogeneous):
  """
  Translation: [x, y, z]
  Quaternion: [x, y, z, w]
  """
  translation = translation_from_matrix(homogeneous)
  quaternion = quaternion_from_matrix(homogeneous)
  return translation, quaternion


def on_trocar(trocar_msg):
  pose = spherical2cartesianpose(trocar_msg.r, trocar_msg.theta, trocar_msg.phi)
  translation, quaternion = homogeneous2translation_quaternion(pose)
  tf_broadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), trocar_msg.instrument_tip_frame, trocar_tf)


def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('trocar_tf', help='Trocar Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  global trocar_tf
  trocar_tf = args.trocar_tf

  rospy.init_node('trocar_topic_to_tf', anonymous=True)
  global tf_broadcaster
  tf_broadcaster = tf.TransformBroadcaster()
  trocar_sub = rospy.Subscriber('get_trocar', TrocarPose, on_trocar, queue_size = 1)

  rospy.loginfo('Spinning')
  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)
