import WCDO.constants as c
import pyrosim.pyrosim as pyrosim
import random
import numpy as np
import pybullet as p
import pybullet_data
import imageio_ffmpeg

def addMotility(numSeconds,strength):

   simulateCells(numSeconds,motilityStrength=strength)

def captureFrame(t,vid):

   if t%20==0:
      c.cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(c.cam_target_pos, c.cam_distance, c.cam_yaw, c.cam_pitch, c.cam_roll, c.cam_up_axis_idx)
      c.cam_projection_matrix = p.computeProjectionMatrixFOV(c.cam_fov, c.cam_width*1./c.cam_height, c.cam_near_plane, c.cam_far_plane)
      image = p.getCameraImage(c.cam_width, c.cam_height,c.cam_view_matrix, c.cam_projection_matrix)[2][:, :, :3]
      vid.send(np.ascontiguousarray(image))
      #c.cam_yaw = c.cam_yaw + 1

def prep():

   physicsClient = p.connect(p.DIRECT)
   p.setAdditionalSearchPath(pybullet_data.getDataPath())
   #p.setGravity(0, 0, -10)
   plane_id = p.loadURDF("plane.urdf")

   vid = imageio_ffmpeg.write_frames('vid.mp4', (c.cam_width, c.cam_height), fps=30)
   vid.send(None) # The first frame of the video must be a null frame.

   objectIDs = p.loadSDF("box.sdf")

   for i in range(0,len(objectIDs)):
      p.changeVisualShape(objectIDs[i], -1, rgbaColor=[random.random(), random.random(), random.random(), 1])

   return vid,objectIDs

def pullTogether(objectIDs):

   for i in range(0,len(objectIDs)):

      p.getLinkState(objectIDs[i], 0)
      pos = link_state[0]
      p.applyExternalForce(objectIDs[i], -1, [50*pos[0],50*pos[1],50*pos[2]], [0, 0, 0], p.WORLD_FRAME)

def push(objectIDs):

   for i in range(0,len(objectIDs)):

      p.applyExternalForce(objectIDs[i], -1, [10*random.random()-5, 10*random.random()-5, 0], [0, 0, 0], p.WORLD_FRAME)

def rebootMulticellularity(numSeconds,strength,loneliness):

   simulateCells(numSeconds,motilityStrength=strength,attractionStrength=loneliness)

def simulateCells(numSeconds,motilityStrength=0,attractionStrength=0):

   vid, objectIDs = prep()
 
   for t in range(0,1000*numSeconds):

      if motilityStrength>0:

         push(objectIDs)

      if attractionStrength>0:

         pullTogether(objectIDs)

      captureFrame(t,vid)

      p.stepSimulation()

   terminate_gracefully(vid)

def sprinkleCells(numCells):

   pyrosim.Start_SDF("box.sdf")

   for i in range(0,numCells):
      pyrosim.Send_Sphere(name="Sphere", pos=[50*random.random()-25,50*random.random()-25,0.5] , radius=0.5)

   pyrosim.End()

def terminate_gracefully(vid):

   vid.close()
   p.disconnect()
