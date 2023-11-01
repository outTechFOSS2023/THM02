# -*- coding: utf-8 -*-
#*********************************************************************************************************
#*********************************************************************************************************
#Copyright 2023 Blueshift, LLC
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, #including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to #do so, subject to the following conditions:
         #The Software is subject to all use, distribution, modification, sales, and other restrictions applicable to the software-as-a-service product specified in the Agreement.
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND #NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR #IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#*********************************************************************************************************
from yade import pack, export
from yade import pack, plot 
from yade import ymport
import sys
import os
import os.path
import numpy
import math

## UNITS:SI
############################################
###           INPUT PARAMETERS           ###
############################################
## ANALYSIS CONSTANT
gravityAcc=-9.81             # GRAVITY ACCELERATION 
numDamp=0.4                  # NUMERICAL DAMPING 
normalDamp=0.7               # NORMAL VISCOUS DAMPING
shearDamp=0.7                # SHEAR VISCOUS DAMPING
############################################

## MATERIAL PROPERTIES
# REGOLITH 
regolithFrictDegree=25      # INTERPARTICLE FRICTION DEGREE
regolithYoung=0.88e8        # CONTACT STIFFNESS
regolithPoisson=0.42        # POISSON RATIO
regolithDensity=1774        # DENSITY
############################################

# CONTAINER AND VANE 
stiffFrictDegree=0.0        # REGOLITH-CONTAINER AND REGOLITH-VANE FRICTION DEGREE (FRICTIONLESS)
stiffYoung=210e9            # CONTACT STIFFNESS
stiffPoisson=0.25           # POISSON RATIO
stiffDensity=7850           # DENSITY
############################################

## MOTION RELATED CONSTANTS
vanePenetVel=0.25e-1        # VANE PENETRATION VELOCITY (m/s). NOTE: THE EXPERIMENTAL VELOCITY IS 0.25 mm/s.SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION(IF PENETRATION REQUIRED).
vaneRotVel=0.1750           # VANE ROTATION VELOCITY AFTER PENETRATION (rad/s). NOTE: THE EXPERIMENTAL ANGULAR VELOCITY IS 0.1 deg/s OR 0.00175 rad/s. SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.
VanePenetDepth= 0.0635      # VANE PENETRATION DEPTH(IF PENETRATION REQUIRED).
VaneDist=0.0                # DISTANCE OF THE VANE BOTTOM TO THE REGOLITH TOP (IF PENETRATION REQUIRED).
VaneCenToVaneBotZ= 0.0635   # THIS IS THE LENGTH OF THE VANE (2.5"). THE LENGTH OF THE ROD ABOVE THE VANE IS ALSO 2.5". 
VaneBotToContBotZ= 0.0635   # THIS IS THE DISTANCE FROM THE BOTTOM OF THE CONTAINER TO THE BOTTOM OF THE PENETRATED VANE. 
XcenterofVane= 0.0          # X COORDINATE OF THE VANE CENTER 
YcenterofVane= 0.0          # Y COORDINATE OF THE VANE CENTER
ZcenterofVane= 0.07620      # INITIAL Z CCORDINATE OF THE VANE CENTER-- THIS IS THE Z COORDINATE OF THE VANE IN ITS ORIGINAL LOCATION (EQUALS TO VaneCenToVaneBotZ(=0.0635 m)+VaneBotToContBotZ(=12.7e-3 m)

############################################
###############   MATERIALS   ##############
############################################
## JIANG
Regolith=O.materials.append(FrictMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity))
#Stiff=O.materials.append(JiangMat(young=regolithYoung,poisson=stiffPoisson,frictionAngle=stiffFrictDegree,density=stiffDensity,xIc=stiffCrushing,beta=stiffBeta))# IGNORING MATERIAL PROPERTIES FOR BOUNDARIES RESULTS IN MORE RELIBALE CURVES

# ###################################
# #####   CREATING GEOMETRIES   #####
# ###################################
## IMPORTING THE HOMOGENIOUS CYLINDRICAL SAMPLE
O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Vane Shear Test/VST-fullSizeSample_Velocity_Effect/Samples/layer_SA3_Full_Real_SI_Uncut.txt',material=Regolith,color=Vector3(1.0, 0.1, 0.1)))
maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
print ("maxZ:",maxZ)

## CUTTING TO THE DESIRED HEIGHT
for b in O.bodies:
  if isinstance(b.shape,Sphere):
    if b.state.pos[2]+b.shape.radius>0.0889:# THIS IS THE EXPERIMENTAL HEIGHT OF THE REGOLITH (88.9e-3 m)
      O.bodies.erase(b.id)
maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
print ("maxZ:",maxZ)
## SAVING THE DESIRED HEIGHT
export.text('/home/ngoudarzi/Desktop/Vane Shear Test/VST-fullSizeSample_Velocity_Effect/Samples/layer_SA3_Full_Real_SI_Cut.txt')

## POROSITY CALCULATION
sampleHeight=maxZ
sampleRadius=0.0508
totalVolume=((math.pi*pow(sampleRadius,2)))*sampleHeight
solidVolume=utils.getSpheresVolume()
samplePorosity=1-(solidVolume/totalVolume)
print ("samplePorosity:",samplePorosity)

## IMPORTING THE VANE
## NOTE: THE VANE IS ALREADY IN THE CORRECT POSITION IN SPACE
EO = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/VST-fullSizeSample_Velocity_Effect/Cad/SA_Vane_Zero_Thickness.stl',shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
for i in EO:
  i.state.mass = 1
  i.state.inertia = (1,1,1)
clumpId,facetsId = O.bodies.appendClumped(EO)
s = O.bodies[clumpId].state
s.blockedDOFs = 'xyzXYZ' ## NOTE: ONLY ROTATION ABOUT Z IS ALLOWED

#Vane= O.bodies.append(EO)
for i in [clumpId]:
  shearVane= O.bodies[i] # FOR MEASUREMENT OF ROTATION ONLY
  # shearVane.dynamic=True
  # shearVane.state.blockedDOFs='xyzXYZ' # THE ONLY FREE DOFs ARE TRANSLATION ALONG AND ROTATION ABOUT Z. AT FIRST ALL DOFs ARE RESTRIANED. MOVEMENT (USING CombinedKinematicEngine)
# WILL FREE DOFs IN THE DIRECTION OF MOVEMENT. ALL OTHER DOFs ARE STILL CONSTRAINED TO AVOID DEVIATION OF THE TOOL FROM Z AXIS.  
#VaneID = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST OF FACETS IN THE IMPORTED VANE

## CYLINDRICAL CONTAINER
## GENERATING THE CYLINDRICAL CONTAINER 
cylinderCenterX=0.0
cylinderCenterY=0.0
cylinderCenterZ=0.1
cylinderRadius=0.05080
cylinderHeight=0.2

leftBoxCenterX=0.05080/2
leftBoxCenterY=0.05080/2
leftBoxCenterZ=0.1

leftBoxLength=0.05080
leftBoxWidth=0.05080
leftBoxHeight=0.2
O.bodies.append(yade.geom.facetCylinder((cylinderCenterX,cylinderCenterY,cylinderCenterZ), radius=cylinderRadius, height=cylinderHeight,segmentsNumber=60, wallMask=6,color=Vector3(0.8,1,1)))


############################
###   DEFINING ENGINES   ###
############################
O.engines=[
#TorqueRecorder(ids=[clumpId],rotationAxis=(XcenterofVane,YcenterofVane,ZcenterofVane),file='/home/SA-1/Desktop/SA-1/Main Test/Output/VS_SA1_90_HM_Torque_SI.txt',iterPeriod=50),
ForceResetter(),
InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb()], label="collider"),
InteractionLoop(
[Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
[Ip2_FrictMat_FrictMat_FrictPhys()],
[Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  CombinedKinematicEngine(ids=[clumpId],label='combEngine') + TranslationEngine(translationAxis=(0,0,-1),velocity=vanePenetVel) +\
  RotationEngine(rotationAxis=(0,0,1), angularVelocity=0, rotateAroundZero=True, zeroPoint=(XcenterofVane,YcenterofVane,ZcenterofVane)),
  NewtonIntegrator(damping=numDamp,gravity=(0,0,gravityAcc)),
  PyRunner(iterPeriod=1000, command="vaneShear()" ,label='checker'),
  PyRunner(iterPeriod=50,command='history()',label='recorder'),
  ]
O.dt = 7e-7 #SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.

## GETTING TranslationEngine and RotationEngine FROM CombinedKinematicEngine
transEngine, rotEngine = combEngine.comb[0], combEngine.comb[1]

## PENETRATION IS NOT MODELED HERE
# def vanePenetration():
#   transEngine.velocity = vanePenetVel
#   rotEngine.zeroPoint += Vector3(0,0,-1)*transEngine.velocity*O.dt
#   vaneBottomCoord= rotEngine.zeroPoint[2]-VaneCenToVaneBotZ 
#   print ("vaneBottomCoord:", vaneBottomCoord) 
#   if (vaneBottomCoord)<=(maxZ-VanePenetDepth):
#     checker.command='vaneShear()'

def vaneShear():
  # global vaneTz,vaneRotZ
  # vaneTz=0
  # RXFY=0
  # RyFx=0
  transEngine.velocity = 0
  rotEngine.angularVelocity = vaneRotVel
  vaneTz=abs(O.forces.t(clumpId,sync=True)[2])
  vaneRotZ=(shearVane.state.rot().norm())*(180/math.pi) # GETTING THE ROTATION OF THE VANE (DIFFERENCE BETWEEN refOri AND ori AND CONVERSION TO DEGREES)
  ############# ALTERNATIVE APPROACH FOR CALCULATION OF FORCES AND TORQUES (NOT RECOMMENDED) ############
  # for b in VaneID:
  #   RxFy=(b.state.pos[0]-XcenterofVane)*O.forces.f(b.id,sync=True)[1]
  #   RyFx=(b.state.pos[1]-YcenterofVane)*O.forces.f(b.id,sync=True)[0]
  #   vaneTz+=(RxFy-RyFx)
  #######################################################################################################
  print ("vaneRotZ:", vaneRotZ,"vaneTz:", vaneTz) 
  if vaneRotZ>=90:
    rotEngine.angularVelocity = 0.0
    O.pause() # EXPERMENT IS STOPPED AFTER 90 degrees OF ROTATION

def history():
  global vaneFx,vaneFy,vaneFz,vaneTx,vaneTy,vaneTz,vaneDx,vaneDy,vaneDz,vaneRotZ
  vaneFx=0
  vaneFy=0
  vaneFz=0
  vaneTx=0
  vaneTy=0
  vaneTz=0
  vaneFx=abs(O.forces.f(clumpId,sync=True)[0])
  vaneFy=abs(O.forces.f(clumpId,sync=True)[1])
  vaneFz=abs(O.forces.f(clumpId,sync=True)[2])
  vaneTx=abs(O.forces.t(clumpId,sync=True)[0])
  vaneTy=abs(O.forces.t(clumpId,sync=True)[1])
  vaneTz=abs(O.forces.t(clumpId,sync=True)[2])
  vaneDx=rotEngine.zeroPoint[0]
  vaneDy=rotEngine.zeroPoint[1]
  vaneDz=rotEngine.zeroPoint[2]
  vaneRotZ=(shearVane.state.rot().norm())*(180/math.pi) 
  yade.plot.addData({'vaneRotZ':vaneRotZ,'vaneTz':vaneTz,})
  ## SAVING THE DATA TO A TEXT FILE AT THE INTERVALS DEFINED FOR history() FUNCTION
  plot.saveDataTxt('/home/ngoudarzi/Desktop/Vane Shear Test/VST-fullSizeSample_Velocity_Effect/Main Tests/Output/VS_SA3_V6_Full_SI.txt')
  
## PLOTTING ON THE SCREEN
plot.plots={'vaneRotZ':('vaneTz')}
plot.plot()

############# ALTERNATIVE APPROACH FOR CALCULATION OF FORCES AND TORQUES (NOT RECOMMENDED) ############
  # global vaneFx,vaneFy,vaneFz,vaneTx,vaneTy,vaneTz,vaneDx,vaneDy,vaneDz,vaneRotZ
  # vaneFx=0
  # vaneFy=0
  # vaneFz=0
  # vaneTx=0
  # vaneTy=0
  # vaneTz=0
  # RXFY=0
  # RyFx=0
  # for b in VaneID:
  #   vaneFx+=O.forces.f(b.id,sync=True)[0]
  #   vaneFy+=O.forces.f(b.id,sync=True)[1]
  #   vaneFz+=O.forces.f(b.id,sync=True)[2]
  #   RxFy=(b.state.pos[0]-XcenterofVane)*O.forces.f(b.id,sync=True)[1]
  #   RxFz=(b.state.pos[0]-XcenterofVane)*O.forces.f(b.id,sync=True)[2]    
  #   RyFx=(b.state.pos[1]-YcenterofVane)*O.forces.f(b.id,sync=True)[0]
  #   RyFz=(b.state.pos[1]-YcenterofVane)*O.forces.f(b.id,sync=True)[2]
  #   RzFx=(b.state.pos[2])*O.forces.f(b.id,sync=True)[0]    
  #   RzFy=(b.state.pos[2])*O.forces.f(b.id,sync=True)[1]          
  #   vaneTx+=(RyFz-RzFy)
  #   vaneTy+=(RxFz-RzFx)
  #   vaneTz+=(RxFy-RyFx)
  # vaneDx=rotEngine.zeroPoint[0]
  # vaneDy=rotEngine.zeroPoint[1]
  # vaneDz=rotEngine.zeroPoint[2]
  # vaneRotZ=(shearVane.state.rot().norm())*(180/math.pi) 
#######################################################################################################
O.run(2000000000000,True)
