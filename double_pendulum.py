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

# Определяем функцию, возвращающую правые части дифференциальных уравнений
def derivatives(state, t, L, m, g):
    """
    Функция, возвращающая производные переменных состояния системы:
    state: вектор состояния системы (theta1, omega1, theta2, omega2)
    """
    # Извлекаем переменные состояния
    theta1, omega1, theta2, omega2 = state
    
    # Вычисляем производные угловых скоростей
    domega1 = (-g*3*m*np.sin(theta1) - m*g*np.sin(theta1 - 2*theta2) - 2*np.sin(theta1-theta2)*m*(omega2*omega2*L + omega1 * omega1 
                * L*np.cos(theta1 - theta2))) / (L*(3*m - m*np.cos(2*theta1 - 2*theta2)))
    
    domega2 = (2 * np.sin(theta1 - theta2)*(omega1*omega1*L*2*m + g * 2*m*np.cos(theta1) + 
                omega2*omega2*L*m*np.cos(theta1 - theta2)))/(L*(3*m - m*np.cos(2*theta1 - 2*theta2)))
    
    # Возвращаем производные переменных состояния
    return [omega1, domega1, omega2, domega2]

# Определяем начальные условия и параметры системы
initial_state = [q0_1, 0, q0_2, 0]  # начальные углы и скорости

# Интегрируем систему дифференциальных уравнений
result = odeint(derivatives, initial_state, logTime, args=(L, m, g))

# Извлекаем значения углов и их скоростей из результата

theta1 = result[:, 0]
omega1 = result[:, 1]
theta2 = result[:, 2]
omega2 = result[:, 3]


result = odeint(derivatives, initial_state, logTime, args=(L, m, g))

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx_2, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

for t in logTime[1:]:

    '''
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_1, 
        controlMode=p.TORQUE_CONTROL, 
        force=-omega1[idx]*m*L
    )
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_2, 
        controlMode=p.TORQUE_CONTROL, 
        force=-omega2[idx]*m*L
    )
    '''
    p.stepSimulation()
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_1, 
        controlMode=p.TORQUE_CONTROL, 
        force=-omega1[0])
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx_2, 
        controlMode=p.TORQUE_CONTROL, 
        force=-omega2[0])
        
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

    
    

import matplotlib.pyplot as plt

plt.grid(True)
plt.plot(logTime, logPos_1, label = "simPos_1")
plt.plot(logTime, logPos_2, label = "simPos_2")
plt.plot(logTime, theta1, label = "theta1")
plt.plot(logTime, theta2, label = "theta2")
plt.plot(logTime, omega1, label = "om1")
plt.plot(logTime, omega2, label = "om2")

#plt.plot(logTime, logLin, label = "logLin")
plt.legend()

plt.show()

p.disconnect()