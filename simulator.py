import pybullet as p
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
			self.physicsClient = p.connect(p.GUI)
		else:
			self.physicsClient = p.connect(p.DIRECT)


		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.resetSimulation()
		p.setGravity(0,0,self.GRAVITY)
		p.setTimeStep(self.dt)
		self.planeId = p.loadURDF("plane.urdf")
		start_pos = [0,0,0.15]
		start_orientation = p.getQuaternionFromEuler([0.,0,0])

		if(damage):
			urdf=(os.path.dirname(os.path.abspath(__file__))+'/urdf/pexod_damaged.urdf')
		self.botId = p.loadURDF(urdf, start_pos, start_orientation)
		self.joint_list = self._make_joint_list(self.botId)

		# #bullet links number corresponding to the legs
		self.leg_link_ids = [17, 14, 2, 5, 8, 11]
		self.descriptor = {17 : [], 14 : [],2 : [],5 : [],8 : [],11 : []}

		p.setRealTimeSimulation(0)
		jointFrictionForce=1
		for joint in range (p.getNumJoints(self.botId)):
			p.setJointMotorControl2(self.botId, joint,
				p.POSITION_CONTROL,
				force=jointFrictionForce)
		for t in range(0, 100):
			p.stepSimulation()
			p.setGravity(0,0, self.GRAVITY)
		self._init_state = p.saveState()

	def reset(self):
		assert(0), "not working for now"
		self.t = 0
		#p.resetSimulation()
		p.restoreState(self._init_state)
		for joint in self.joint_list:
			if(joint!=1000):
				p.resetJointState(self.botId, joint, 0)

	def get_pos(self):
		'''
		Returns the position list of 3 floats and orientation as list of 4 floats in [x,y,z,w] order.
		Use p.getEulerFromQuaternion to convert the quaternion to Euler if needed.
		'''
		return p.getBasePositionAndOrientation(self.botId)

	def step(self, controller):
		angles = controller.step(self.t)
		i = 0
		error = False
		#Check if roll pitch are not too high
		self.euler = p.getEulerFromQuaternion(self.get_pos()[1])
		if(self.safety_turnover):
			if((abs(self.euler[1]) >= math.pi/2) or (abs(self.euler[0]) >= math.pi/2)):
				error = True

		missing_joint_count = 0
		for joint in self.joint_list:
			if(joint==1000):
				missing_joint_count += 1
			else:
				info = p.getJointInfo(self.botId, joint)
				lower_limit = info[8]
				upper_limit = info[9]
				max_force = info[10]
				max_velocity = info[11]
				pos = min(max(lower_limit, angles[i]), upper_limit)
				p.setJointMotorControl2(self.botId, joint,
					p.POSITION_CONTROL,
					targetPosition=pos,
					force=max_force,
					maxVelocity=max_velocity)
			i += 1

		#### DESCRIPTOR DUTY CYCLE HEXAPOD  ###################################
		#Get contact points between minitaur and world plane
		contact_points = p.getContactPoints(self.botId,self.planeId)
		link_ids = [] #list of links in contact with the ground plane
		if(len(contact_points)>0):
			for cn in contact_points:
				linkid= cn[3] #minitaur link id in contact with world plane
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

			
		p.setGravity(0,0,self.GRAVITY)
		p.stepSimulation()
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
			for joint in range (p.getNumJoints(botId)):
				name = p.getJointInfo(botId, joint)[1]
				if name == n:
					joint_list += [joint]
					joint_found = True
			if(joint_found==False):
				joint_list += [1000] #if the joint is not here (aka broken leg case) put 1000
		return joint_list

	def destroyed(self):
		p.disconnect()

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
	# for k in range(0, 10):
	# 	t0 = time.perf_counter()
	# 	simu = HexapodSimulator(gui=False)
	# 	controller = Controller(ctrl,minitaur=False)
	# 	for i in range(0, 3000):
	# 		simu.step(controller)
	# 	print(time.perf_counter() - t0, " ms", simu.get_pos()[0])
	# 	simu.destroyed()
	reward, desc = eval_hexapod(ctrl,True)
	print("Final reward : ", reward)
	print("Associated desc : ", desc)
