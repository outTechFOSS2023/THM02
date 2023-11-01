# -*- coding: utf-8 -*-
#*********************************************************************************************************
#*********************************************************************************************************
#Copyright 2023 Blueshift, LLC
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, #including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to #do so, subject to the following conditions:
         #The Software is subject to all use, distribution, modification, sales, and other restrictions applicable to the software-as-a-service product specified in the Agreement.
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND #NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR #IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#*********************************************************************************************************

from builtins import range
from yade import qt,plot
from yade.gridpfacet import *

phi=20.
E=3.*1e8
r=0.005

O.engines=[
  ForceResetter(),
  InsertionSortCollider([
    Bo1_Sphere_Aabb(),
    Bo1_GridConnection_Aabb(),
    Bo1_PFacet_Aabb(),
  ]),
  InteractionLoop([
    Ig2_Sphere_Sphere_ScGeom(),
    Ig2_Sphere_GridConnection_ScGridCoGeom(),
    Ig2_Sphere_PFacet_ScGridCoGeom(),
    Ig2_GridNode_GridNode_GridNodeGeom6D(),
    Ig2_GridConnection_GridConnection_GridCoGridCoGeom(),
    Ig2_GridConnection_PFacet_ScGeom(),
    Ig2_PFacet_PFacet_ScGeom(),
  ],
  [
    Ip2_CohFrictMat_CohFrictMat_CohFrictPhys(setCohesionNow=True,setCohesionOnNewContacts=True),
    Ip2_FrictMat_FrictMat_FrictPhys()
  ],
  [
    Law2_ScGeom6D_CohFrictPhys_CohesionMoment(),
    Law2_ScGeom_FrictPhys_CundallStrack(),
    Law2_ScGridCoGeom_FrictPhys_CundallStrack(),
    Law2_GridCoGridCoGeom_FrictPhys_CundallStrack(),
  ]
  ),
  GlobalStiffnessTimeStepper(timestepSafetyCoefficient=0.1,label='ts'),
  NewtonIntegrator(gravity=(0,-9.81,0),damping=.5,label='newton'),
]

O.materials.append( CohFrictMat( young=0.05*E,poisson=0.3,density=2650,frictionAngle=radians(phi),normalCohesion=3e100,shearCohesion=3e100,momentRotationLaw=True,label='cohfrictMat' ) )
O.materials.append( FrictMat( young=E,poisson=0.5,density=260050,frictionAngle=radians(phi),label='frictMat' ) )

# SPHERES
O.bodies.append( sphere(center=(Vector3(0.045,-0.035,0.015)),radius=.005,material='frictMat',fixed=False) )
O.bodies.append( sphere(center=(Vector3(0.045,-0.025,0.015)),radius=.005,material='frictMat',fixed=False) )
O.bodies.append( sphere(center=(Vector3(0.05,-0.035,0.025)),radius=.005,material='frictMat',fixed=False) )

# GRIDS
L=0.1 #length
l=0.05 #width
nbL=10 #number of nodes for the length
nbl=5 #number of nodes for the width
r=L/100. #radius
colorTG=[40./255.,102./255.,50./255.]
nodesTG=[]
colorBG=[153./255.,194./255.,151./255.]
nodesBG=[]
#Top grid:
#node
for i in range(0,nbL):
  for j in range(0,nbl):
    nodesTG.append( O.bodies.append(gridNode([i*L/nbL,-0.1,j*l/nbl],r,wire=False,fixed=False,material='cohfrictMat',color=colorTG)) )
#connection
for i in range(0,len(nodesTG)):
  for j in range(i+1,len(nodesTG)):
    dist=(O.bodies[nodesTG[i]].state.pos - O.bodies[nodesTG[j]].state.pos).norm()
    if(dist<=L/nbL*1.01):
      O.bodies.append( gridConnection(nodesTG[i],nodesTG[j],r,material='frictMat',color=colorTG) )
#PFacet connection
O.bodies.append( gridConnection(nodesTG[21],nodesTG[27],r,material='frictMat',color=colorTG) )
#PFacet
O.bodies.append( pfacet(nodesTG[21],nodesTG[27],nodesTG[26],wire=False,material='frictMat',color=colorTG) )
O.bodies.append( pfacet(nodesTG[21],nodesTG[22],nodesTG[27],wire=False,material='frictMat',color=colorTG) )
# constraining the ends of the top grid
for i in range(nodesTG[0],nodesTG[5]):
  O.bodies[i].state.blockedDOFs='xyzXZ'
  for j in range(nodesTG[-1]-(4),nodesTG[-1]+1):
    O.bodies[j].state.blockedDOFs='xyzXZ'
#Bottom grid:
#node
for i in range(0,nbL):
  for j in range(0,nbl):
    nodesBG.append( O.bodies.append(gridNode([(i*L/nbL)+0.005,-0.11,j*l/nbl],r,wire=False,fixed=False,material='cohfrictMat',color=colorBG)) )
#connection
for i in range(0,len(nodesBG)):
  for j in range(i+1,len(nodesBG)):
    dist=(O.bodies[nodesBG[i]].state.pos - O.bodies[nodesBG[j]].state.pos).norm()
    if(dist<=L/nbL*1.01):
      O.bodies.append( gridConnection(nodesBG[i],nodesBG[j],r,material='frictMat',color=colorBG) )
#PFacet connection
O.bodies.append( gridConnection(nodesBG[21],nodesBG[27],r,material='frictMat',color=colorBG) )
O.bodies.append( gridConnection(nodesBG[22],nodesBG[28],r,material='frictMat',color=colorBG) )
#PFacet
O.bodies.append( pfacet(nodesBG[21],nodesBG[27],nodesBG[26],wire=False,material='frictMat',color=colorBG) )
O.bodies.append( pfacet(nodesBG[21],nodesBG[22],nodesBG[27],wire=False,material='frictMat',color=colorBG) )
O.bodies.append( pfacet(nodesBG[22],nodesBG[28],nodesBG[27],wire=False,material='frictMat',color=colorBG) )
O.bodies.append( pfacet(nodesBG[22],nodesBG[23],nodesBG[28],wire=False,material='frictMat',color=colorBG) )
# constraining the bottom grid
for i in nodesBG:
  O.bodies[i].state.blockedDOFs='xyzXZ'

qt.Controller()
qtv = qt.View()
qtr = qt.Renderer()
qtr.bgColor=[1,1,1]
Gl1_Sphere.stripes=1
O.saveTmp()
