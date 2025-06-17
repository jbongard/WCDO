import pyrosim.pyrosim as pyrosim
import random

def sprinkleCells(numCells=100):

   pyrosim.Start_SDF("box.sdf")

   for i in range(0,numCells):
      pyrosim.Send_Sphere(name="Sphere", pos=[50*random.random()-25,50*random.random()-25,0.5] , radius=0.5)

   pyrosim.End()
