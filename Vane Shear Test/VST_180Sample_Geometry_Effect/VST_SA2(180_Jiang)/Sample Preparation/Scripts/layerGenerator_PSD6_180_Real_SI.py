# -*- coding: utf-8 -*-
#*********************************************************************************************************
#*********************************************************************************************************
#Copyright 2023 Blueshift, LLC
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, #including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to #do so, subject to the following conditions:
         #The Software is subject to all use, distribution, modification, sales, and other restrictions applicable to the software-as-a-service product specified in the Agreement.
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND #NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR #IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#*********************************************************************************************************
from __future__ import print_function

from yade import pack,export
from pylab import rand
from yade import qt
from yade import ymport
import numpy
#############################################
####           INPUT PARAMETERS           ###
#############################################
## ANALYSIS CONSTANT
gravityAcc=-9.81            # GRAVITY ACCELERATION 
numDamp=0.4                 # NUMERICAL DAMPING 
normalDamp=0.7              # NORMAL VISCOUS DAMPING
shearDamp=0.7               # SHEAR VISCOUS DAMPING
#############################################

## MATERIAL PROPERTIES
# REGOLITH 
regolithFrictDegree=0.0     # INTERPARTICLE FRICTION DEGREE
regolithYoung=0.88e8        # CONTACT STIFFNESS
regolithPoisson=0.42        # POISSON RATIO
regolithDensity=1774        # DENSITY
#############################################

# CONTAINER AND VANE 
stiffFrictDegree=0.0        # REGOLITH-CONTAINER AND REGOLITH-VANE FRICTION DEGREE (FRICTIONLESS)
stiffYoung=210e9            # CONTACT STIFFNESS
stiffPoisson=0.25           # POISSON RATIO
stiffDensity=7850           # DENSITY
#############################################
## IF PSD IS REQUIRED 
# n_band = 8
# psdSizes=[0.11e-2,0.14e-2,0.16e-2,0.21e-2,0.275e-2,0.33e-2,0.42e-2,0.535e-2]
# psdCumm=[0.47,0.525,0.580,0.645,0.695,0.760,0.820,1.0]

n_band = 3
psdSizes=[0.33e-2,0.42e-2,0.535e-2]
psdCumm=[0.760,0.820,1.0]

#############################################
################   MATERIALS   ##############
#############################################
Regolith=O.materials.append(FrictMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity))
#Stiff=O.materials.append(JiangMat(young=regolithYoung,poisson=stiffPoisson,frictionAngle=stiffFrictDegree,density=stiffDensity,xIc=stiffCrushing,beta=stiffBeta))


###################################
#####   CREATING GEOMETRIES   #####
############################################
cylinderCenterX=0.0000
cylinderCenterY=0.0000
cylinderCenterZ=1.2000
cylinderRadius=0.05080
cylinderHeight=2.40000


bottomBoundaryCenterX=0.0000
bottomBoundaryCenterY=0.0508
bottomBoundaryCenterZ=1.2000

bottomBoundaryLengthX=0.0508
bottomBoundaryLengthY=0.0508
bottomBoundaryLengthZ=1.2000



O.bodies.append(yade.geom.facetCylinder((cylinderCenterX,cylinderCenterY,cylinderCenterZ), radius=cylinderRadius, height=cylinderHeight,segmentsNumber=60, wallMask=6, color=Vector3(0.8,1,1)))
O.bodies.append(yade.geom.facetBox((bottomBoundaryCenterX,bottomBoundaryCenterY,bottomBoundaryCenterZ), (bottomBoundaryLengthX,bottomBoundaryLengthY,bottomBoundaryLengthZ),wallMask=4,color=Vector3(0.8,1,0)))

EO = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/VST_180Sample_Geometry_Effect/VST_SA2(180_Jiang)/Cad/SA_Vane_Zero_Thickness.stl',shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
Vane= O.bodies.append(EO)


mn,mx=Vector3(-0.0508,-0.0508,0.14),Vector3(0.0508,0.0508,2.35) # CORNERS OF THE INITIAL PACKING.
sp = pack.SpherePack()
sp.makeCloud(mn,mx,psdSizes=psdSizes,psdCumm=psdCumm,num=20000,distributeMass=1)
sp.toSimulation(material=Regolith,color=Vector3(0.9,0.0,0.4))





for b in O.bodies:
  if isinstance(b.shape,Sphere):
    R_1=((b.state.pos[0]+b.shape.radius)**2+(b.state.pos[1]+b.shape.radius)**2)**0.5
    R_2=((b.state.pos[0]-b.shape.radius)**2+(b.state.pos[1]+b.shape.radius)**2)**0.5 
    R_3=((b.state.pos[0]+b.shape.radius)**2+(b.state.pos[1]-b.shape.radius)**2)**0.5
    R_4=((b.state.pos[0]-b.shape.radius)**2+(b.state.pos[1]-b.shape.radius)**2)**0.5        
    if R_1<=-0.0508 or R_1>=0.0508 or R_2<=-0.0508 or R_2>=0.0508 or  R_3<=-0.0508 or R_3>=0.0508 or R_4<=-0.0508 or R_4>=0.0508:
      O.bodies.erase(b.id)
for b in O.bodies:
  if isinstance(b.shape,Sphere):
    if (b.state.pos[1]-b.shape.radius)<0:
      O.bodies.erase(b.id)

############################
###   DEFINING ENGINES   ###
############################
Newton=NewtonIntegrator(damping=numDamp,gravity=Vector3(0, 0,gravityAcc))
O.engines=[
  ForceResetter(),
  InsertionSortCollider([Bo1_Facet_Aabb(),Bo1_Sphere_Aabb()]),
  InteractionLoop(
    [Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
    [Ip2_FrictMat_FrictMat_FrictPhys()],
    [Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  Newton,
  PyRunner(command='layerGeneration()',iterPeriod=500,label='checker'),
]
O.dt=1.0e-5

######################################
##   GENERATION OF THE 1ST LAYER   ###
######################################
def layerGeneration():
  unb=unbalancedForce()
  maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
  print ('unbalanced force:',unb,' maxZ: ',maxZ)
  if O.iter>200000 and unb<0.01:
    print ("###      EQUILIBRIUM ACHIEVED       ###")
    export.text('/home/ngoudarzi/Desktop/Vane Shear Test/VST_180Sample_Geometry_Effect/VST_SA2(180_Jiang)/Sample Preparation/Samples/layer_PSD6_180_Real_SI_Uncut.txt')
    O.pause

 

