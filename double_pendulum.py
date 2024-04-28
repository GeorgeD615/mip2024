import pybullet as p
import numpy as np
import time
from scipy.integrate import odeint

g = 10
L = 0.5
m = 1
k = 0.5
f = 0.1
tau1 = 0.1
tau2 = 0.1
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

# Определяем функцию, возвращающую правые части дифференциальных уравнений
def derivatives(state, t, L, m, g, tau1, tau2):
    """
    Функция, возвращающая производные переменных состояния системы:
    state: вектор состояния системы (theta1, omega1, theta2, omega2)
    """
    # Извлекаем переменные состояния
    theta1, omega1, theta2, omega2 = state
    
    # Вычисляем производные угловых скоростей
    dth = theta1 - theta2

    domega2 = (tau2/L - tau1/(2*L)*np.cos(dth) - omega2*omega2/2*np.sin(dth)*np.cos(dth) + omega1*np.sin(dth) 
            - g/L*np.sin(theta2)) / (1 + 0.5*np.cos(dth)*np.cos(dth))
            
    domega1 = tau1/(2*L) - domega2/2*np.cos(dth) - omega2*omega2/2*np.sin(dth) + g/L*np.sin(theta1)
    
    
    # Возвращаем производные переменных состояния
    return [omega1, domega1, omega2, domega2]


# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=tau1)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_2, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=tau2)

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

    step = derivatives((th1, om1, th2, om2), dt, L, m, g, tau1, tau2)

    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_1, 
        controlMode=p.TORQUE_CONTROL, 
        force=-step[1]
    )
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_2, 
        controlMode=p.TORQUE_CONTROL, 
        force=-step[3]
    )

    
    

import matplotlib.pyplot as plt

plt.grid(True)
plt.plot(logTime, logPos_1, label = "simPos_1")
plt.plot(logTime, logPos_2, label = "simPos_2")

#plt.plot(logTime, logLin, label = "logLin")
plt.legend()

plt.show()

p.disconnect()