#!/usr/bin/python
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
regolithBeta=0.0            # JIANG'S SHAPE PARAMETER
regolithCrushing=2.1        # JIANG'S CRUSHING PARAMETER
regolithYoung=0.88e8        # CONTACT STIFFNESS
regolithPoisson=0.42        # POISSON RATIO
regolithDensity=1774        # DENSITY
#############################################

# CONTAINER AND VANE 
stiffFrictDegree=0.0        # REGOLITH-CONTAINER AND REGOLITH-VANE FRICTION DEGREE (FRICTIONLESS)
stiffBeta=0.0               # JIANG'S SHAPE PARAMETER (NO ROLLING RESISTANCE FOR REGOLITH-CONTAINER AND REGOLITH-VANE INTERACTION)
stiffCrushing=2.1           # JIANG'S CRUSHING PARAMETER (2.1 IS THE INITIAL VALUE.NO ROLLING RESISTANCE FOR REGOLITH-CONTAINER AND REGOLITH-VANE INTERACTION)
stiffYoung=210e9            # CONTACT STIFFNESS
stiffPoisson=0.25           # POISSON RATIO
stiffDensity=7850           # DENSITY
#############################################
## IF PSD IS REQUIRED 
# n_band = 8
# psdSizes=[0.11e-2,0.14e-2,0.16e-2,0.21e-2,0.275e-2,0.33e-2,0.42e-2,0.535e-2]
# psdCumm=[0.47,0.525,0.580,0.645,0.695,0.760,0.820,1.0]
n_band = 4
psdSizes=[0.275e-2,0.33e-2,0.42e-2,0.535e-2]
psdCumm=[0.695,0.760,0.820,1.0]

O.periodic=True
length=0.0635
height=2.1
width=0.0635

O.cell.hSize=Matrix3(length, 0, 0,
      0 ,width, 0,
      0, 0, height)

#############################################
################   MATERIALS   ##############
#############################################
Regolith=O.materials.append(JiangMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity,xIc=regolithCrushing,beta=regolithBeta))
#Stiff=O.materials.append(JiangMat(young=regolithYoung,poisson=stiffPoisson,frictionAngle=stiffFrictDegree,density=stiffDensity,xIc=stiffCrushing,beta=stiffBeta))


###################################
#####   CREATING GEOMETRIES   #####
############################################
boxCenterX=0.0635/2
boxCenterY=0.0635/2
boxCenterZ=0.07+0.01
boxLengthX=0.0635
boxLengthY=0.0635
boxLengthZ=0.07

## BOTTOM BOUNDARY
O.bodies.append(yade.geom.facetBox((boxCenterX,boxCenterY,boxCenterZ), (boxLengthX,boxLengthY,boxLengthZ), wallMask=16, wire=False,color=Vector3(0.5,0.6,0.8)))



EO = ymport.stl('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Cad/Zero_Thickness_Regular_Periodic_Vane.stl',shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
for i in EO:
  i.state.mass = 1
  i.state.inertia = (1,1,1)
clumpId,facetsId = O.bodies.appendClumped(EO)
s = O.bodies[clumpId].state
s.blockedDOFs = 'xyzXYZ' ## NOTE: ONLY ROTATION ABOUT Z IS ALLOWED

mn1,mx1=Vector3(0.0,0.0,(0.14+0.01)),Vector3(0.0635-0.001,0.0635-0.001,2.0) # CORNERS OF THE INITIAL PACKING.
sp1 = pack.SpherePack()
sp1.makeCloud(mn1,mx1,psdSizes=psdSizes,psdCumm=psdCumm,num=15000,distributeMass=1)
sp1.toSimulation(material=Regolith,color=Vector3(0.9,0.0,0.4))

############################
###   DEFINING ENGINES   ###
############################
Newton=NewtonIntegrator(damping=numDamp,gravity=Vector3(0, 0,gravityAcc)) #FAST DEPOSITION FOR SAMPLE PREPARATION. THE FINAL SAMPLE MUST COME TO EQUILIBRIUM UNDER g=-900.81 cm/s^2
O.engines=[
  ForceResetter(),
  InsertionSortCollider([Bo1_Facet_Aabb(),Bo1_Sphere_Aabb()],allowBiggerThanPeriod=True),
  InteractionLoop(
    [Ig2_Sphere_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
    [Ip2_JiangMat_JiangMat_JiangPhys()],
    [Law2_ScGeom_JiangPhys_Jiang(includeRollResistMoment=False,includeTwistResistMoment=False,label='rollingResistance')]
  ),
  Newton,
  PyRunner(command='layerGeneration()',iterPeriod=200,label='checker'),
]
O.dt=1.0e-5
#DISPLAY SPHERES WITH 2 COLORS FOR BETTER VISIBILITY
#Gl1_Sphere.stripes=1
######################################
##   GENERATION OF THE 1ST LAYER   ###
######################################
def layerGeneration():
  unb=unbalancedForce()
  maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
  print ('unbalanced force:',unb,' maxZ: ',maxZ)
  if O.iter>200000 and unb<0.01:
    print ("###      EQUILIBRIUM ACHIEVED       ###")
    export.text('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Sample Preparation/Samples/layer_SA5_Regular_Periodic_Real_SI_Uncut.txt')
    O.pause

 

