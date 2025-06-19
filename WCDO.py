import WCDO.constants as c
import pyrosim.pyrosim as pyrosim
import random
import numpy as np
import pybullet as p
import pybullet_data
import imageio_ffmpeg

def addIntervention(numSeconds,intervention):

   simulateCells(numSeconds,motilityStrength=c.motilityStrength,attractionStrength=c.attractionStrength,intervention=intervention)

def addLoneliness(numSeconds):

   simulateCells(numSeconds,motilityStrength=c.motilityStrength,attractionStrength=c.attractionStrength)

def addMotility(numSeconds):

   simulateCells(numSeconds,motilityStrength=c.motilityStrength)

def captureFrame(t,vid):

   if t%20==0:
      c.cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(c.cam_target_pos, c.cam_distance, c.cam_yaw, c.cam_pitch, c.cam_roll, c.cam_up_axis_idx)
      c.cam_projection_matrix = p.computeProjectionMatrixFOV(c.cam_fov, c.cam_width*1./c.cam_height, c.cam_near_plane, c.cam_far_plane)
      image = p.getCameraImage(c.cam_width, c.cam_height,c.cam_view_matrix, c.cam_projection_matrix)[2][:, :, :3]
      vid.send(np.ascontiguousarray(image))
      #c.cam_yaw = c.cam_yaw + 1

def intervene(objectIDs,intervention):

   for objID in objectIDs:

      pos, orientation = p.getBasePositionAndOrientation(objID)

      fromX = pos[0]
      fromY = pos[1]
      fromZ = pos[2]

      toX = pos[0] + 2
      toY = pos[1] + 2
      toZ = pos[2] + 2

      line_id = p.addUserDebugLine([fromX,fromY,fromZ],[toX, toY, toZ])

      print(objID,line_id)

def prep():

   physicsClient = p.connect(p.DIRECT)
   p.setAdditionalSearchPath(pybullet_data.getDataPath())
   #p.setGravity(0, 0, -10)
   plane_id = p.loadURDF("plane.urdf")

   vid = imageio_ffmpeg.write_frames('vid.mp4', (c.cam_width, c.cam_height), fps=30)
   vid.send(None) # The first frame of the video must be a null frame.

   objectIDs = p.loadSDF("box.sdf")

   for objID in objectIDs:

      p.changeVisualShape(objID, -1, rgbaColor=[random.random(), random.random(), random.random(), 1])

   return vid,objectIDs

def pullTogether(objectIDs,attractionStrength):

   for objID in objectIDs:

      pos, orientation = p.getBasePositionAndOrientation(objID)

      x = 2 * attractionStrength * pos[0] - attractionStrength
      y = 2 * attractionStrength * pos[1] - attractionStrength
      z = 2 * attractionStrength * pos[2] - attractionStrength

      p.applyExternalForce(objID, -1, [ -x , -y , -z ], [0, 0, 0], p.WORLD_FRAME)

def push(objectIDs,motilityStrength):

   for objID in objectIDs:

      x = 2 * motilityStrength * random.random() - motilityStrength
      y = 2 * motilityStrength * random.random() - motilityStrength
      z = 2 * motilityStrength * random.random() - motilityStrength

      p.applyExternalForce(objID, -1, [x,y,z], [0, 0, 0], p.WORLD_FRAME)

def simulateCells(numSeconds, motilityStrength = 0 , attractionStrength = 0 , intervention = None):

   vid, objectIDs = prep()
 
   for t in range(0,625*numSeconds):

      if motilityStrength>0:

         push(objectIDs,motilityStrength)

      if attractionStrength>0:

         pullTogether(objectIDs,attractionStrength)

      if t==0 and intervention != None:

         intervene(objectIDs,intervention)

      captureFrame(t,vid)

      p.stepSimulation()

   terminate_gracefully(vid)

def sprinkleCells(numCells):

   pyrosim.Start_SDF("box.sdf")

   for i in range(0,numCells):

      x = c.petriDishWidth * random.random() - (c.petriDishWidth / 2.0)
      y = c.petriDishWidth * random.random() - (c.petriDishWidth / 2.0)
      z = c.cellRadius

      pyrosim.Send_Sphere(name="Sphere", pos=[x,y,z] , radius=c.cellRadius)

   pyrosim.End()

def terminate_gracefully(vid):

   vid.close()
   p.disconnect()
