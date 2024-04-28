import pybullet as p
import numpy as np

f = 0.1
dt = 1/240 # pybullet simulation step
# q0 = np.pi - np.deg2rad(5)   # starting position (radian)
# q0 = np.deg2rad(15)   # starting position (radian)
th0 = 0.0
thd = 0.5
jIdx = 1
kp = 350
ki = 450
kd = 50
e_int = 0
e_prev = 0
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = th0
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)
idx = 0
u = 0

physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
boxId = p.loadURDF("./pendulum.urdf", useFixedBase=True)

# turn off internal damping
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
for t in logTime[1:]:
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, controlMode=p.TORQUE_CONTROL, force=f)
    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    e = th1-thd
    ed = (e-e_prev)/dt
    e_int += e*dt
    e_prev = e
    u = -kp*e - ki*e_int - kd*ed
    logCtrl[idx] = u
    # p.setJointMotorControl2(
    #     bodyIndex=boxId, 
    #     jointIndex=jIdx, 
    #     controlMode=p.VELOCITY_CONTROL, 
    #     targetVelocity=u
    # )

    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx, 
        controlMode=p.TORQUE_CONTROL, 
        force=u
    )
    
    p.stepSimulation()

    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    logVel[idx] = dth1
    idx += 1
    logPos[idx] = th1

logVel[idx] = p.getJointState(boxId, jIdx)[1]
logCtrl[idx] = u

# setTempLvl(lvl) [0-100]
# t = getTemperature()
# e = t-t_d
# u = -10*e
# setTemplLvl(u)

import matplotlib.pyplot as plt

plt.subplot(3,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0],logTime[-1]],[thd,thd],'r', label='refPos')
plt.legend()

plt.subplot(3,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.subplot(3,1,3)
plt.grid(True)
plt.plot(logTime, logCtrl, label = "simCtrl")
plt.legend()

plt.show()

p.disconnect()

# 1 добавить второе звено такой же длины в urdf к маятнику
# и вывести уравнения его динамики
# валидация: начать в ненулевом положении, посчитать правую часть модели без f и подать ее f = -rp

# 2* подобрать параметры ПИД-регулятора, чтобы обеспечить переход в 10% область refPos с минимальным овершутом