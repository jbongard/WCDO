import pyrosim.pyrosim as pyrosim
import random

pyrosim.Start_SDF("box.sdf")

for i in range(0,99+1):
  #print(i)
  #pyrosim.Send_Cube(name="Box", pos=[random.random()*0.6-0.3 , random.random()*0.6-0.3 , 0.5 + i] , size=[1,1,1])
  pyrosim.Send_Sphere(name="Sphere", pos=[50*random.random()-25,50*random.random()-25,0.5] , radius=0.5)
pyrosim.End()
