import pybullet as p
import numpy as np
import time
from scipy.integrate import odeint

g = 10
L = 0.5
m = 1
k = 0.5
f = 0.1
dt = 1/240 # pybullet simulation step
#q0 = np.pi - np.deg2rad(5)   # starting position (radian)
q0_1 = np.deg2rad(-15)   # starting position 1(radian)
q0_2 = np.deg2rad(-105)
jIdx_1 = 1
jIdx_2 = 3
maxTime = 5
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos_1 = np.zeros(sz)
logPos_2 = np.zeros(sz)
logPos_1[0] = q0_1
logPos_2[0] = q0_2
idx = 0

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
boxId = p.loadURDF("./double_pendulum.urdf", useFixedBase=True)

# turn off internal damping
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_1, targetPosition=q0_1, controlMode=p.POSITION_CONTROL)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_2, targetPosition=q0_2, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# while True:
#     p.stepSimulation()
#     time.sleep(dt)

def derivatives(state, L, m, g):
    # Извлекаем переменные состояния
    theta1, omega1, theta2, omega2 = state
    
    dth = theta1 - theta2

    '''
    0 = (tau2/L - tau1/(2*L)*np.cos(dth) - omega2*omega2/2*np.sin(dth)*np.cos(dth) + omega1*np.sin(dth)  
            - g/L*np.sin(theta2)) / (1 + 0.5*np.cos(dth)*np.cos(dth))
            
    0 = tau1/(2*L) - omega2*omega2/2*np.sin(dth) + g/L*np.sin(theta1)
    '''
    tau_1 = omega2*L*np.sin(dth)-2*g*np.sin(theta1)
    tau_2 = tau_1/2*np.cos(dth)+omega2*omega2*L/2*np.sin(dth)*np.cos(dth)-omega1*L*np.sin(dth) + g*np.sin(theta2)

    return [tau_1, tau_2]


# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_2, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

for t in logTime[1:]:
    p.stepSimulation()
    time.sleep(dt)

    jointState_1 = p.getJointState(boxId, jIdx_1)
    jointState_2 = p.getJointState(boxId, jIdx_2)
    th1 = jointState_1[0]
    th2 = jointState_2[0]
    om1 = jointState_1[1]
    om2 = jointState_2[1]
    idx += 1
    logPos_1[idx] = th1
    logPos_2[idx] = th2

    step = derivatives((th1, om1, th2, om2), L, m, g)

    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_1, 
        controlMode=p.TORQUE_CONTROL, 
        force=step[0]
    )
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_2, 
        controlMode=p.TORQUE_CONTROL, 
        force=step[1]
    )
    
    

import matplotlib.pyplot as plt

plt.grid(True)
plt.plot(logTime, logPos_1, label = "simPos_1")
plt.plot(logTime, logPos_2, label = "simPos_2")

#plt.plot(logTime, logLin, label = "logLin")
plt.legend()

plt.show()

p.disconnect()