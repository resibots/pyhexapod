#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloise Dalin , eloise.dalin@inria.fr
#| Pierre Desreumaux , pierre.desreumaux@inria.fr
#|
#| Antoine Cully, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret.
#|"Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
import os
import time
import math
from timeit import default_timer as timer
import time
import numpy as np
from pycontrollers.controller import Controller

# from pylab import *
# def plot_control(hexapod_controller):
#     fig = figure()
#     ax = fig.add_subplot(111)
#     leg1 = hexapod_controller._control_signal(1, 0.5, 0.5)
#     ax.plot(np.arange(0, 100), leg1)
#     leg1 = hexapod_controller._control_signal(0.5, 0, 0.5)
#     ax.plot(np.arange(0, 100), leg1)
#     fig.savefig('legs.pdf')

class HexapodSimulator:
	def __init__(self, gui=False, urdf=(os.path.dirname(os.path.abspath(__file__))+'/urdf/pexod.urdf'), dt = 1e-3, control_dt=0.001,damage = False):
		self.GRAVITY = -9.8
		self.dt = dt
		self.control_dt = control_dt
		self.t = 0
		self.reward = 0
		self.safety_turnover = True

		if gui:
			self.physics = bc.BulletClient(connection_mode=p.GUI)
		else:
			self.physics = bc.BulletClient(connection_mode=p.DIRECT)
		self.physics.setAdditionalSearchPath(pybullet_data.getDataPath())
		self.physics.resetSimulation()
		self.physics.setGravity(0,0,self.GRAVITY)
		self.physics.setTimeStep(self.dt)
		self.physics.setPhysicsEngineParameter(fixedTimeStep=self.dt,
                            				   solverResidualThreshold=1 - 10,
                         					   numSolverIterations=50,
                         					   numSubSteps=4)
		self.planeId = self.physics.loadURDF("plane.urdf")
		start_pos = [0,0,0.15]
		start_orientation = self.physics.getQuaternionFromEuler([0.,0,0])

		if(damage):
			urdf=(os.path.dirname(os.path.abspath(__file__))+'/urdf/pexod_damaged.urdf')
		self.botId = self.physics.loadURDF(urdf, start_pos, start_orientation)
		self.joint_list = self._make_joint_list(self.botId)

		# #bullet links number corresponding to the legs
		self.leg_link_ids = [17, 14, 2, 5, 8, 11]
		self.descriptor = {17 : [], 14 : [],2 : [],5 : [],8 : [],11 : []}

		self.physics.setRealTimeSimulation(0)
		jointFrictionForce=1
		for joint in range (self.physics.getNumJoints(self.botId)):
			self.physics.setJointMotorControl2(self.botId, joint,
				p.POSITION_CONTROL,
				force=jointFrictionForce)
		for t in range(0, 100):
			self.physics.stepSimulation()
			self.physics.setGravity(0,0, self.GRAVITY)
		#self._init_state = self.physics.saveState()


	def destroy(self):
		try:
			self.physics.disconnect()
		except p.error as e:
			print("Warning (destructor of simulator):", e)


	def reset(self):
		assert(0), "not working for now"
		self.t = 0
		self.physics.resetSimulation()
#		self.physics.restoreState(self._init_state)
		

	def get_pos(self):
		'''
		Returns the position list of 3 floats and orientation as list of 4 floats in [x,y,z,w] order.
		Use p.getEulerFromQuaternion to convert the quaternion to Euler if needed.
		'''
		return self.physics.getBasePositionAndOrientation(self.botId)

	def step(self, controller):
		angles = controller.step(self.t)
		i = 0
		error = False
		#Check if roll pitch are not too high
		self.euler = self.physics.getEulerFromQuaternion(self.get_pos()[1])
		if(self.safety_turnover):
			if((abs(self.euler[1]) >= math.pi/2) or (abs(self.euler[0]) >= math.pi/2)):
				error = True

		missing_joint_count = 0
		for joint in self.joint_list:
			if(joint==1000):
				missing_joint_count += 1
			else:
				info = self.physics.getJointInfo(self.botId, joint)
				lower_limit = info[8]
				upper_limit = info[9]
				max_force = info[10]
				max_velocity = info[11]
				pos = min(max(lower_limit, angles[i]), upper_limit)
				self.physics.setJointMotorControl2(self.botId, joint,
					p.POSITION_CONTROL,
					targetPosition=pos,
					force=max_force,
					maxVelocity=max_velocity)
			i += 1

		#### DESCRIPTOR DUTY CYCLE HEXAPOD  ###################################
		#Get contact points between robot and world plane
		contact_points = self.physics.getContactPoints(self.botId,self.planeId)
		link_ids = [] #list of links in contact with the ground plane
		if(len(contact_points)>0):
			for cn in contact_points:
				linkid= cn[3] #robot link id in contact with world plane
				if linkid not in link_ids:
					link_ids.append(linkid)
		# num_leg_on_ground = 0
		for l in self.leg_link_ids:
			cns = self.descriptor[l]
			if l in link_ids:
				# num_leg_on_ground=num_leg_on_ground+1
				cns.append(1)
			else:
				cns.append(0)
			self.descriptor[l] = cns


		self.physics.setGravity(0,0,self.GRAVITY)
		self.physics.stepSimulation()
		self.t += self.control_dt
		self.reward = -self.get_pos()[0][0]
		return error

	def _make_joint_list(self, botId):
		joint_names = [b'body_leg_0', b'leg_0_1_2', b'leg_0_2_3',
		b'body_leg_1', b'leg_1_1_2', b'leg_1_2_3',
		b'body_leg_2', b'leg_2_1_2', b'leg_2_2_3',
		b'body_leg_3', b'leg_3_1_2', b'leg_3_2_3',
		b'body_leg_4', b'leg_4_1_2', b'leg_4_2_3',
		b'body_leg_5', b'leg_5_1_2', b'leg_5_2_3',
		]
		joint_list = []
		for n in joint_names:
			joint_found = False
			for joint in range (self.physics.getNumJoints(botId)):
				name = self.physics.getJointInfo(botId, joint)[1]
				if name == n:
					joint_list += [joint]
					joint_found = True
			if(joint_found==False):
				joint_list += [1000] #if the joint is not here (aka broken leg case) put 1000
		return joint_list


def eval_hexapod(ctrl,gui_eval = False,damage = False):
	simu = HexapodSimulator(gui=gui_eval,damage=damage)
	controller = Controller(ctrl,minitaur=False)
	for i in range(0,5000):
		simu.step(controller)
	keys = list(simu.descriptor.keys())
	desc=[]
	for k in keys:
		cns = simu.descriptor[k]
		d = round(sum(cns)/len(cns)*100.0)/100.0
		desc.append(d)
	reward = simu.reward
	simu.destroyed()
	return reward, np.array(desc)


if __name__ == "__main__":
	ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5]
	for k in range(0, 10):
		t0 = time.perf_counter()
		simu = HexapodSimulator(dt=0.01,gui=False)
		controller = Controller(ctrl,minitaur=False)
		for i in range(0, int(3./simu.dt)):
			simu.step(controller)
		print(time.perf_counter() - t0, " ms", simu.get_pos()[0])
		simu.destroy()
	# reward, desc = eval_hexapod(ctrl,True)
	# print("Final reward : ", reward)
	# print("Associated desc : ", desc)
