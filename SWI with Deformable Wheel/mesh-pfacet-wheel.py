# -*- coding: utf-8 -*-
#*********************************************************************************************************
#*********************************************************************************************************
#Copyright 2023 Blueshift, LLC
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, #including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to #do so, subject to the following conditions:
         #The Software is subject to all use, distribution, modification, sales, and other restrictions applicable to the software-as-a-service product specified in the Agreement.
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND #NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR #IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#*********************************************************************************************************
from yade import qt,plot
from yade.gridpfacet import *
from yade import ymport

##################
### PARAMETERS ###
##################
phi=30.
E=3.*1e8
color=[255./255.,102./255.,0./255.]
r=0.0001

# position of imported mesh
xpafet=0.22
ypafet=0.05


################
### ENGINES  ###
################
O.engines=[
	ForceResetter(),
	InsertionSortCollider([Bo1_PFacet_Aabb(),Bo1_Sphere_Aabb(),Bo1_Box_Aabb()]),
	InteractionLoop(
		[Ig2_GridNode_GridNode_GridNodeGeom6D(),Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Sphere_PFacet_ScGridCoGeom(),Ig2_PFacet_PFacet_ScGeom()],
    [Ip2_CohFrictMat_CohFrictMat_CohFrictPhys(setCohesionNow=True,setCohesionOnNewContacts=True),Ip2_FrictMat_FrictMat_FrictPhys()],
	  [Law2_ScGeom6D_CohFrictPhys_CohesionMoment(),Law2_ScGeom_FrictPhys_CundallStrack(),Law2_ScGridCoGeom_FrictPhys_CundallStrack()],
	),
  #GlobalStiffnessTimeStepper(timestepSafetyCoefficient=0.1,label='ts'), 
	NewtonIntegrator(gravity=(0,0,-9.81),damping=0.7,label='newton'),
	PyRunner(iterPeriod=200,command='history()'),
]


O.dt=1.0e-5
################
### MATERIAL ###
################
O.materials.append( CohFrictMat( young=E,poisson=0.3,density=2650,frictionAngle=radians(phi),normalCohesion=3e100,shearCohesion=3e100,momentRotationLaw=True,label='gridNodeMat'))  # material to create the gridConnections
O.materials.append( FrictMat( young=E,poisson=0.3,density=2650,frictionAngle=radians(phi),label='pFacetMat'))  # material for general interactions


########################################
### GENERATE THE WALL AND THE SPHERE ###
########################################
#IMPORT MESH
oriBody = Quaternion(Vector3(0,0,1),pi/2.)
nodesIds,cylIds,pfIds = gmshPFacet( meshfile='/home/ngoudarzi/Desktop/SWI with Deformable Wheel/CylinderWheel.mesh', shift=Vector3(0,0,0.003), scale=0.001,radius=r, wire=False, fixed=False, materialNodes='gridNodeMat', material='pFacetMat', color=[1,0,0] )

O.bodies.append(ymport.text("/home/ngoudarzi/Desktop/SWI with Deformable Wheel/HW_Deposited_Bed.txt",shift=Vector3(0,0,0)))
minX_bed=min([b.state.pos[0]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
maxX_bed=max([b.state.pos[0]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
minY_bed=min([b.state.pos[1]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
maxY_bed=max([b.state.pos[1]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
minZ_bed=min([b.state.pos[2]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
maxZ_bed=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
sampleLength_bed=abs(maxX_bed-minX_bed)
sampleWidth_bed=abs(maxY_bed-minY_bed)
sampleHeight_bed=abs(maxZ_bed-minZ_bed)
print ("minX_bed:",minX_bed,"maxX_bed:",maxX_bed,"minY_bed:",minY_bed,"maxY_bed:",maxY_bed,"minZ_bed:",minZ_bed,"maxZ_bed:",maxZ_bed)
print ("sampleLength_bed:",sampleLength_bed,"sampleWidth_bed:",sampleWidth_bed,"sampleHeight_bed:",sampleHeight_bed)

mn,mx=Vector3(minX_bed,minY_bed,minZ_bed),Vector3(maxX_bed,maxY_bed,5*maxZ_bed)
walls=aabbWalls([mn,mx],thickness=0)
wallIds=O.bodies.append(walls)

############
### PLOT ###
############
def history():
  xyz=[]
  for k in [0,1,2]:
    ksum=0
    for i in nodesIds:
      ksum+=O.bodies[i].state.pos[k]
    xyz.append(ksum/len(nodesIds))  # take average value as reference
  plot.addData(i=O.iter,t=O.time,x=xyz[0],y=xyz[1],z=xyz[2])
plot.plots={'x':'z'}
plot.plot()


##########
## VIEW ##
##########

qt.Controller()
qtv = qt.View()
qtr = qt.Renderer()
qtr.light2=True
qtr.lightPos=Vector3(1200,1500,500)
qtr.bgColor=[1,1,1]

O.saveTmp()
