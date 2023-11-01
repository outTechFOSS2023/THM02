# -*- coding: utf-8 -*-

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
regolithBeta=0.27           # JIANG'S SHAPE PARAMETER
regolithCrushing=2.1        # JIANG'S CRUSHING PARAMETER
regolithYoung=0.88e8        # CONTACT STIFFNESS
regolithPoisson=0.42        # POISSON RATIO
regolithDensity=1774        # DENSITY
RegolithEta=0.4             # ROLLING FRICTION COEFFICENT IN HM
############################################

# CONTAINER AND VANE 
stiffFrictDegree=0.0        # REGOLITH-CONTAINER AND REGOLITH-VANE FRICTION DEGREE (FRICTIONLESS)
stiffBeta=0.0               # JIANG'S SHAPE PARAMETER (NO ROLLING RESISTANCE FOR REGOLITH-CONTAINER AND REGOLITH-VANE INTERACTION)
stiffCrushing=2.1           # JIANG'S CRUSHING PARAMETER (2.1 IS THE INITIAL VALUE.NO ROLLING RESISTANCE FOR REGOLITH-CONTAINER AND REGOLITH-VANE INTERACTION)
stiffYoung=210e9            # CONTACT STIFFNESS
stiffPoisson=0.25           # POISSON RATIO
stiffDensity=7850           # DENSITY
############################################

## MOTION RELATED CONSTANTS
vanePenetVel=0.1            # VANE PENETRATION VELOCITY (m/s). NOTE: THE EXPERIMENTAL VELOCITY IS 0.25 mm/s.SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION(IF PENETRATION REQUIRED).
vaneRotVel=0.175            # VANE ROTATION VELOCITY AFTER PENETRATION (rad/s). NOTE: THE EXPERIMENTAL ANGULAR VELOCITY IS 0.1 deg/s OR 0.00175 rad/s. SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.
VanePenetDepth= 0.0762      # VANE PENETRATION DEPTH(IF PENETRATION REQUIRED).
VaneDist=0.0                # DISTANCE OF THE VANE BOTTOM TO THE REGOLITH TOP (IF PENETRATION REQUIRED).
VaneCenToVaneBotZ= 0.0635   # THIS IS THE LENGTH OF THE VANE (2.5"). THE LENGTH OF THE ROD ABOVE THE VANE IS ALSO 2.5". 
VaneBotToContBotZ= 12.7e-3  # THIS IS THE DISTANCE FROM THE BOTTOM OF THE CONTAINER TO THE BOTTOM OF THE PENETRATED VANE. 
XcenterofVane= 0.0635/2     # X COORDINATE OF THE VANE CENTER 
YcenterofVane= 0.0635/2     # Y COORDINATE OF THE VANE CENTER
ZcenterofVane= 0.0862       # INITIAL Z CCORDINATE OF THE VANE CENTER-- THIS IS THE Z COORDINATE OF THE VANE IN ITS ORIGINAL LOCATION (EQUALS TO VaneCenToVaneBotZ(=0.0635 m)+VaneBotToContBotZ(=12.7e-3 m)

############################################
## PSD 
# n_band = 8
# psdSizes=[0.11e-2,0.14e-2,0.16e-2,0.21e-2,0.275e-2,0.33e-2,0.42e-2,0.535e-2]
# psdCumm=[0.47,0.525,0.580,0.645,0.695,0.760,0.820,1.0]
n_band = 4
psdSizes=[0.275e-2,0.33e-2,0.42e-2,0.535e-2]
psdCumm=[0.695,0.760,0.820,1.0]

############################################
## PERIODICITY 
O.periodic=True
length=0.0635
height=2.1
width=0.0635

O.cell.hSize=Matrix3(length, 0, 0,
      0 ,width, 0,
      0, 0, height)


############################################
###############   MATERIALS   ##############
############################################
## JIANG
Regolith=O.materials.append(JiangMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity,xIc=regolithCrushing,beta=regolithBeta))
#Stiff=O.materials.append(JiangMat(young=regolithYoung,poisson=stiffPoisson,frictionAngle=stiffFrictDegree,density=stiffDensity,xIc=stiffCrushing,beta=stiffBeta))# IGNORING MATERIAL PROPERTIES FOR BOUNDARIES RESULTS IN MORE RELIBALE CURVES

# ###################################
# #####   CREATING GEOMETRIES   #####
# ###################################
## IMPORTING THE HOMOGENIOUS CYLINDRICAL SAMPLE
O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Sample Preparation/Samples/layer_SA5_Regular_Periodic_Real_SI_Uncut.txt',material=Regolith,color=Vector3(1.0, 0.1, 0.1)))
maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
print ("maxZ:",maxZ)

## CUTTING TO THE DESIRED HEIGHT
for b in O.bodies:
  if isinstance(b.shape,Sphere):
    if b.state.pos[2]+b.shape.radius>0.01+0.0889:# THIS IS THE EXPERIMENTAL HEIGHT OF THE REGOLITH (88.9e-3 m). 0.01 m IS THE OFFSET FROM THE BOTTOM OF THE PERIODIC CELL
      O.bodies.erase(b.id)
maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
print ("maxZ:",maxZ)
## SAVING THE DESIRED HEIGHT
export.text('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Sample Preparation/Samples/layer_SA5_Regular_Periodic_Real_SI_Cut.txt')

## POROSITY CALCULATION
sampleHeight=(maxZ-0.01) # 0.01 m IS THE OFFSET FROM THE BOTTOM OF THE PERIODIC CELL
totalVolume=length*maxZ*width
solidVolume=utils.getSpheresVolume()
samplePorosity=1-(solidVolume/totalVolume)
print ("samplePorosity:",samplePorosity)

## IMPORTING THE VANE
## NOTE: THE VANE IS ALREADY IN THE CORRECT POSITION IN SPACE
EO = ymport.stl('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Cad/Zero_Thickness_Vane_Regular_Periodic.stl',shift=Vector3(0,0,0),wire=True,color=Vector3(0.5,0.6,0.8))
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

# BOTTOM BOUNDARY
boxCenterX=0.0635/2
boxCenterY=0.0635/2
boxCenterZ=0.07+0.01
boxLengthX=0.0635/2-0.003
boxLengthY=0.0635/2-0.003
boxLengthZ=0.07
## BOTTOM BOUNDARY
O.bodies.append(yade.geom.facetBox((boxCenterX,boxCenterY,boxCenterZ), (boxLengthX,boxLengthY,boxLengthZ), wallMask=16, wire=False,color=Vector3(0.5,0.6,0.8)))


############################
###   DEFINING ENGINES   ###
############################
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Main Test/VTK/VS_SA5_Regular_Periodic_Real_SI/VS_SA5_Regular_Periodic_Real_SI' ,recorders=['all'],iterPeriod=250000),
#TorqueRecorder(ids=[clumpId],rotationAxis=(XcenterofVane,YcenterofVane,ZcenterofVane),file='/home/SA-1/Desktop/SA-1/Main Test/Output/VS_SA1_90_HM_Torque_SI.txt',iterPeriod=50),
ForceResetter(),
InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb()], allowBiggerThanPeriod=True,label="collider"),
InteractionLoop(
## JIANG
[Ig2_Sphere_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
[Ip2_JiangMat_JiangMat_JiangPhys(label='ContactModel')],
[Law2_ScGeom_JiangPhys_Jiang(includeRollResistMoment=True,includeTwistResistMoment=True,label='rollingResistance')]
  ),
  CombinedKinematicEngine(ids=[clumpId],label='combEngine') + TranslationEngine(translationAxis=(0,0,-1),velocity=vanePenetVel) +\
  RotationEngine(rotationAxis=(0,0,1), angularVelocity=0, rotateAroundZero=True, zeroPoint=(XcenterofVane,YcenterofVane,ZcenterofVane)),
  NewtonIntegrator(damping=numDamp,gravity=(0,0,gravityAcc)),
  PyRunner(iterPeriod=1, command="vaneShear()" ,label='checker'),
  PyRunner(iterPeriod=50,command='history()',label='recorder'),
  ]
O.dt = 7e-7 #SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.

## GETTING TranslationEngine and RotationEngine FROM CombinedKinematicEngine
transEngine, rotEngine = combEngine.comb[0], combEngine.comb[1]

# PENETRATION IS NOT MODELED HERE
# def vanePenetration():
#   transEngine.velocity = vanePenetVel
#   rotEngine.zeroPoint += Vector3(0,0,-1)*transEngine.velocity*O.dt
#   vaneBottomCoord= rotEngine.zeroPoint[2]-VaneCenToVaneBotZ 
#   print ("vaneBottomCoord:", vaneBottomCoord) 
#   if (vaneBottomCoord)<=VaneBotToContBotZ:
#     checker.command='vaneShear()'

def vaneShear():
  # global vaneTz,vaneRotZ
  # vaneTz=0
  # RXFY=0
  # RyFx=0
  transEngine.velocity = 0
  rotEngine.angularVelocity = vaneRotVel
  vaneTz=abs(O.forces.t(clumpId,sync=True)[2])
  vaneRotZ=-((shearVane.state.rot().norm())*(180/math.pi)-180) # GETTING THE ROTATION OF THE VANE (DIFFERENCE BETWEEN refOri AND ori AND CONVERSION TO DEGREES)
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
  vaneRotZ=-((shearVane.state.rot().norm())*(180/math.pi)-180)
  yade.plot.addData({'vaneRotZ':vaneRotZ,'vaneTz':vaneTz,})
  ## SAVING THE DATA TO A TEXT FILE AT THE INTERVALS DEFINED FOR history() FUNCTION
  plot.saveDataTxt('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Main Test/Output/VS_SA5_Regular_Periodic_Real_SI.txt')
  
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
# O.run(2000000000000,True)
