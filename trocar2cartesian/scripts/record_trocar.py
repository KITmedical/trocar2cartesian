#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf
import numpy as np
from tf.transformations import *
from geometry_msgs.msg import Pose
from trocar2cartesian_msgs.srv import SetTrocarRequest
from math import pi



def translation_quaternion2pose_msg(translation, quaternion):
  pose = Pose()
  pose.position.x = translation[0]
  pose.position.y = translation[1]
  pose.position.z = translation[2]
  pose.orientation.x = quaternion[0]
  pose.orientation.y = quaternion[1]
  pose.orientation.z = quaternion[2]
  pose.orientation.w = quaternion[3]
  return pose



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('robot_base_tf', help='Robot Base Frame')
  parser.add_argument('instrument_tip_tf', help='Instrument Tip Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  robot_base_tf, instrument_tip_tf = args.robot_base_tf, args.instrument_tip_tf

  rospy.init_node('record_trocar', anonymous=True)
  tf_listener = tf.TransformListener()

  tf_listener.waitForTransform(robot_base_tf, instrument_tip_tf, rospy.Time(), rospy.Duration(4.0))
  position, quaternion = tf_listener.lookupTransform(robot_base_tf, instrument_tip_tf, rospy.Time())
  set_trocar_msg = SetTrocarRequest()
  set_trocar_msg.trocar_frame = robot_base_tf
  set_trocar_msg.instrument_tip_frame = instrument_tip_tf
  set_trocar_msg.trocar_pose = translation_quaternion2pose_msg(position, (0, 0, 0, 1)) # use world orientation
  set_trocar_msg.limits.r_min = 0.01
  set_trocar_msg.limits.r_max = 0.3
  set_trocar_msg.limits.theta_max = pi/2
  set_trocar_msg.limits.theta_max = pi
  set_trocar_msg.limits.phi_min = -pi;
  set_trocar_msg.limits.phi_max = pi;
  print('Now move robot into trocar and afterwards (perhaps change limits):')
  print("rosservice call /robots/lwr1/set_trocar '\n%s\n'" % set_trocar_msg)


if __name__ == '__main__':
  main(sys.argv)
