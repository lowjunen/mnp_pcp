#%% Generate Scene
import pybullet as p
import time
import pybullet_data
import math
import matplotlib.pyplot as plt
import numpy as np
import imageio
import csv  

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

p.setGravity(0,0,-10)

bunnyStartPos = [0,0,1]
bunnyStartOrientation = p.getQuaternionFromEuler([90,0,0])
bunnyId = p.loadURDF("bunny.urdf",bunnyStartPos, bunnyStartOrientation)

# %% Snap photos in a sphere

x_p,y_p,z_p = [],[],[]

# Fix projection parameters
projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# We use Fibonacci sphere to generate the x,y,z points for the camera with
# total count n_c and sphere radius r_c.
n_c = 10
r_c = 0.5
phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians

# Take photos (rgba,depth,seg)
for i in range(n_c):
    # Compute camera point
    y = 1 - (i / float(n_c - 1)) * 2
    radius = math.sqrt(1 - y * y)

    theta = phi * i  # golden angle increment

    y_c = y + bunnyStartPos[1]
    x_c = math.cos(theta) * radius + bunnyStartPos[0]
    z_c = math.sin(theta) * radius + bunnyStartPos[2]
    
    # Configure camera accordingly
    viewMatrix = p.computeViewMatrix(
        cameraEyePosition=[x_c,y_c,z_c],
        cameraTargetPosition=bunnyStartPos,
        cameraUpVector=[0,0,1])

    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=1080, 
        height=720,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix)
    
    # Save Stuff
    name = "images/" + str(i).zfill(3)
    
    imageio.imwrite(name + "_rgba.png", rgbImg)     # save rgb
    imageio.imwrite(name + "_depth.png", depthImg)  # save depth
    imageio.imwrite(name + "_seg.png", segImg)      # save seg


    # For debugging camera points
    x_p.append(x_c)
    y_p.append(y_c)
    z_p.append(z_c)

# # Quick debug of camera points
# fig = plt.figure(figsize = (10, 7)) 
# ax = plt.axes(projection ="3d") 
# ax.scatter3D(x_p, y_p, z_p, color = "green")
# plt.show() 

#%%
# t_step = 1./240.
# for i in range (240*5):
#     p.stepSimulation()
#     time.sleep(t_step)

# p.disconnect()
