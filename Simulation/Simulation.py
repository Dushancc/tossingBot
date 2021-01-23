import numpy as np
import tensorflow as tf
import keras
import keras.backend as kb
from tensorflow.keras.optimizers import RMSprop
from numpy import zeros
import os
import pybullet as p
import pybullet_data
import time
import math
import pandas
import pickle
dis=30
filename = 'sgd100k.sav'
loaded_model = pickle.load(open(filename, 'rb'))
result = loaded_model.predict([dis])/10
velocity=result
x=0
xbase=-.12
throwed=0
t=45
s=0.3
g=9.82
u=math.sqrt((s*g)/(math.sin(2*t)))
force=20
#linvel=u
#vel=linvel/0.25
#velocity=vel
angle=-1.47
prev_pos=0
position1=[0,0,0]
p.connect(p.GUI)
urdfRootPath=pybullet_data.getDataPath()
planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])
pandaUid=p.loadURDF("C:\\Users\\Buddhitha\\Downloads\\G1\\G1\\urdf\\G1.urdf",[-0.045618, -0.051497, -0.10757], useFixedBase=1)
p.resetBasePositionAndOrientation(pandaUid, [0, 0, 0], [0, 0, 0,1])
position, orientation = p.getBasePositionAndOrientation(pandaUid)
tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[-0.5,0,-0.667])
objectUid = p.loadURDF("C:\\Users\\Buddhitha\\Desktop\\Lego\\Lego\\Lego.urdf", basePosition=[ xbase,0,0.01])
aUid = p.loadURDF(os.path.join(urdfRootPath, "tray/tray.urdf"),basePosition=[-(0.2+dis/100),0,-0.02],globalScaling=0.4)
position, orientation = p.getBasePositionAndOrientation(objectUid)
p.setGravity(0,0,-9.8)
p.changeDynamics(pandaUid,5,lateralFriction=1)
p.changeDynamics(pandaUid,6,lateralFriction=1)
p.changeDynamics(objectUid,-1,lateralFriction=1)
p.changeDynamics(pandaUid,2,lateralFriction=1)
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[-0.6,0.-0.55,0.2])
state_durations = [0.01,0.01,.001,900]
control_dt = 1./32000.
p.setTimestep = control_dt
state_t = 0.
current_state = 0
numJoints=p.getNumJoints(pandaUid)
pos = [-0.26, 0, 0.04]
jointPoses = p.calculateInverseKinematics(pandaUid,2, pos)
p.setJointMotorControl2(pandaUid, 3, p.POSITION_CONTROL, -1.5708)
i=[10,12,13,14,15]
stop=0
removed=0
train_steps=60
a_dis=zeros([train_steps])
a_vel=zeros([train_steps])
train_count=0
train=1
while True:
  time.sleep(0.01)
  if train==1:
    state_t += control_dt
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    joint_positions = [j[0] for j in p.getJointStates(pandaUid, range(6))]
    j2=joint_positions[2]
    if (j2 > angle ) & (current_state > 2):
     current_state=0
     state_t=0
     throwed=1
     position0  = p.getBasePositionAndOrientation(objectUid)
     initial=position0[0]
    if (j2 > angle) & (current_state > 2):
        current_state = 0
        state_t = 0
        throwed = 1
        position0 = p.getBasePositionAndOrientation(objectUid)
        initial = position0[0]
    if current_state == 0:
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=4, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.523599)
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=5, controlMode=p.POSITION_CONTROL,
                                targetPosition=-0.523599)
    if current_state == 1:
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=0, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.0209245185263742)
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=1, controlMode=p.POSITION_CONTROL,
                                targetPosition=-0.845398)
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=2, controlMode=p.POSITION_CONTROL,
                                targetPosition=-2.3561)
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=3, controlMode=p.POSITION_CONTROL,
                                targetPosition=-0.159)
    if current_state == 2:
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=4, controlMode=p.POSITION_CONTROL,
                                targetPosition=-0.874533, force=200)
        p.setJointMotorControl2(bodyUniqueId=pandaUid, jointIndex=5, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.874533, force=200)
    if current_state == 3:
        if stop == 0:
            # print(1)
            p.setJointMotorControl2(pandaUid, 2, p.VELOCITY_CONTROL, targetVelocity=velocity, force=force)
    # DONT REMOVE
    #DONT REMOVE
    #if stop==1:
     #   if j2> (angle -.10) :
      #     p.setJointMotorControl2(pandaUid, 2,
       #                         p.POSITION_CONTROL, angle+.10)
    if stop ==0:
       if state_t >state_durations[current_state]:
          current_state += 1
          if current_state >= len(state_durations):
             current_state = 0
             #time.sleep(10)
          state_t = 0
    p.stepSimulation()
    prev_pos = xbase
    #if removed ==0:
    position1, orientation = p.getBasePositionAndOrientation(objectUid)
    final=position1[0]
    if position1[2] < 0.0001:
      if throwed==1:
       x +=1
       if x == 2 :
        #print(train_count)
        stop=1
        distance= initial[0]-final
        #print(s,distance,'vel',linvel,vel)
        #print(distance,velocity)
        #removed=1
        if j2> (angle -.10) :
           p.setJointMotorControl2(pandaUid, 2,
                                p.POSITION_CONTROL, 1)
        #time.sleep(10)
        if train_count <train_steps :
         time.sleep(4)
         p.removeBody(objectUid)
         p.removeBody(aUid)
         objectUid = p.loadURDF("C:\\Users\\Buddhitha\\Desktop\\Lego\\Lego\\Lego.urdf", basePosition=[xbase, 0, 0.01])
         stop = 0
         #removed = 0
         throwed= 0
         x=0
         #s=s+0.01
        #u = math.sqrt((s * g) / (math.sin(2 * t)))
        #force = 6
        #linvel = u
        #vel = linvel / 0.25
        #velocity = vel
         #a_dis[train_count]=distance*100
         #a_vel[train_count]=dis
         train_count=train_count+1
         print(train_count,distance,dis)
         dis=dis+10
         aUid = p.loadURDF(os.path.join(urdfRootPath, "tray/tray.urdf"), basePosition=[-(dis/100 + 0.25), 0, -0.04],
                        globalScaling=0.4)
         velocity = (loaded_model.predict([dis]))/10