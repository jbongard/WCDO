import pyrosim.pyrosim as pyrosim
import random
import numpy as np
import pybullet as p
import pybullet_data
import imageio_ffmpeg

def simulateCells():

   cam_target_pos = [0, 0, 0] #  -0.2, 0.2]
   cam_distance = 48 # 2.05
   cam_yaw, cam_pitch, cam_roll = -50, -40, 0
   cam_width, cam_height = 480*2, 368*2
   cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60

   physicsClient = p.connect(p.DIRECT)
   p.setAdditionalSearchPath(pybullet_data.getDataPath())
   #p.setGravity(0, 0, -10)
   plane_id = p.loadURDF("plane.urdf")

   vid = imageio_ffmpeg.write_frames('vid.mp4', (cam_width, cam_height), fps=30)
   vid.send(None) # The first frame of the video must be a null frame.

   objectIDs = p.loadSDF("box.sdf")

   for i in range(0,len(objectIDs)):
      p.changeVisualShape(objectIDs[i], -1, rgbaColor=[random.random(), random.random(), random.random(), 1])

   for t in range(0,1000):

      for i in range(0,len(objectIDs)):
         p.applyExternalForce(objectIDs[i], -1, [10*random.random()-5, 10*random.random()-5, 0], [0, 0, 0], p.WORLD_FRAME)

      if t%20==0:
         cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
         cam_projection_matrix = p.computeProjectionMatrixFOV(cam_fov, cam_width*1./cam_height, cam_near_plane, cam_far_plane)
         image = p.getCameraImage(cam_width, cam_height,cam_view_matrix, cam_projection_matrix)[2][:, :, :3]
         vid.send(np.ascontiguousarray(image))
         #cam_yaw = cam_yaw + 1

      p.stepSimulation()

   vid.close()
   p.disconnect()

def sprinkleCells(numCells):

   pyrosim.Start_SDF("box.sdf")

   for i in range(0,numCells):
      pyrosim.Send_Sphere(name="Sphere", pos=[50*random.random()-25,50*random.random()-25,0.5] , radius=0.5)

   pyrosim.End()
