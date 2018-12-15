import pybullet as p
import pybullet_data
import os
import time
import math
from timeit import default_timer as timer
import time
from controller import Controller

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
    def __init__(self, gui=False, urdf='pexod.urdf', dt = 1e-3, control_dt=0.01):
        self.GRAVITY = -9.8
        self.dt = dt
        self.control_dt = control_dt
        self.t = 0
        if gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0,0,self.GRAVITY)
        p.setTimeStep(self.dt)
        planeId = p.loadURDF("plane.urdf")
        start_pos = [0,0,0.15]
        start_orientation = p.getQuaternionFromEuler([0.,0,0])
        self.botId = p.loadURDF(urdf, start_pos, start_orientation)
        self.joint_list = self._make_joint_list(self.botId)
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
        for joint in self.joint_list:
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
        p.setGravity(0,0,self.GRAVITY)
        p.stepSimulation()
        self.t += self.control_dt
       
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
            for joint in range (p.getNumJoints(botId)):
                name = p.getJointInfo(botId, joint)[1]
                if name == n:
                    joint_list += [joint]
        return joint_list


if __name__ == "__main__":
    ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5]
    for k in range(0, 10):
        t0 = time.perf_counter()
        simu = HexapodSimulator(gui=False)
        controller = Controller(ctrl)
        for i in range(0, 5000):
            simu.step(controller)
        print(time.perf_counter() - t0, " ms", simu.get_pos()[0])
  

