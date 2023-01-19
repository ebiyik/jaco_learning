#! /usr/bin/env python

import roslib; roslib.load_manifest('kinova_demo')

import rospy
import math
import sys, select, os
import time

from utils import ros_utils, experiment_utils
from utils.openrave_utils import *
from utils.environment import Environment
from controllers.pid_controller import PIDController
from planners.trajopt_planner import TrajoptPlanner

import time

import kinova_msgs.msg
from kinova_msgs.srv import *

import numpy as np
import pickle

class JointAngleRecorder(object):

	def __init__(self):
		# Create ROS node.
		rospy.init_node("final_position_recorder")

		# Load parameters and set up subscribers/publishers.
		self.load_parameters()
		self.register_callbacks()

		# Publish to ROS at 100hz.
		r = rospy.Rate(100)

		ros_utils.start_admittance_mode(self.prefix)

		self.data = []
		self.data_pose = []

		rospy.on_shutdown(self.shutdownhook)

		while not rospy.is_shutdown():

			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				line = raw_input()
				break

			self.vel_pub.publish(ros_utils.cmd_to_JointVelocityMsg(self.cmd))
			r.sleep()


	def shutdownhook(self):
		ros_utils.stop_admittance_mode(self.prefix)
		
		savefile = self.expUtil.get_unique_filepath("test", "dof")
		pickle.dump(np.array(self.data).squeeze(), open(savefile, "wb" ))
		savefile = self.expUtil.get_unique_filepath("test", "pose")
		pickle.dump(np.array(self.data_pose), open(savefile, "wb" ))
		
		print self.data[-1] / (math.pi/180.)
		print self.data_pose[-1]


	def load_parameters(self):
		"""
		Loading parameters and setting up variables from the ROS environment.
		"""

		# ----- General Setup ----- #
		self.prefix = rospy.get_param("setup/prefix")
		self.save_dir = rospy.get_param("setup/save_dir")
		
		# Openrave parameters for the environment.
		model_filename = rospy.get_param("setup/model_filename")
		object_centers = rospy.get_param("setup/object_centers")
		self.environment = Environment(model_filename, object_centers)
		
		# Constant zero input because human will be in full control.
		self.cmd = np.zeros((7,7))
		
		# Utilities for recording data.
		self.expUtil = experiment_utils.ExperimentUtils(self.save_dir)
	

	def register_callbacks(self):
		"""
		Sets up all the publishers/subscribers needed.
		"""
		
		# Create joint-velocity publisher.
		self.vel_pub = rospy.Publisher(self.prefix + '/in/joint_velocity', kinova_msgs.msg.JointVelocity, queue_size=1)

		# Create subscriber to joint_angles.
		rospy.Subscriber(self.prefix + '/out/joint_angles', kinova_msgs.msg.JointAngles, self.joint_angles_callback, queue_size=1)
	
	def joint_angles_callback(self, msg):
		"""
		Reads the latest position of the robot and publishes an
		appropriate torque command to move the robot to the target.
		"""

		# Read the current joint angles from the robot.
		self.curr_pos = np.array([msg.joint1,msg.joint2,msg.joint3,msg.joint4,msg.joint5,msg.joint6,msg.joint7]).reshape((7,1))

		# Convert to radians.
		self.curr_pos = self.curr_pos*(math.pi/180.0)
		
		self.data.append(self.curr_pos.squeeze())
		self.data_pose.append(self.config_to_pose(self.curr_pos))


	def config_to_pose(self, config):
		waypt = np.append(config.squeeze(), np.array([0,0,0]))
		waypt[2] += np.pi
		self.environment.robot.SetDOFValues(waypt)
		
		links = self.environment.robot.GetLinks()
		link = links[7] 
		tf = link.GetTransform()
		pose = tf[:3,3]

		return pose

		
if __name__ == '__main__':
	JointAngleRecorder()
