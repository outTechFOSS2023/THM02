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
numDamp=0.2                  # NUMERICAL DAMPING 
normalDamp=0.7               # NORMAL VISCOUS DAMPING
shearDamp=0.7                # SHEAR VISCOUS DAMPING
############################################

## MATERIAL PROPERTIES
# REGOLITH 
regolithFrictDegree=20      # INTERPARTICLE FRICTION DEGREE
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
vanePenetVel=0.25e-1        # VANE PENETRATION VELOCITY (m/s). NOTE: THE EXPERIMENTAL VELOCITY IS 0.25 mm/s.SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.
vaneRotVel=17.5          # VANE ROTATION VELOCITY AFTER PENETRATION (rad/s). NOTE: THE EXPERIMENTAL ANGULAR VELOCITY IS 0.1 deg/s OR 0.00175 rad/s. SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.
VanePenetDepth= 0.0635      # VANE PENETRATION DEPTH.
VaneDist=0.0                # DISTANCE OF THE VANE BOTTOM TO THE REGOLITH TOP
VaneCenToVaneBotZ= 0.0635   # THIS IS THE LENGTH OF THE VANE (2.5"). THE LENGTH OF THE ROD ABOVE THE VANE IS ALSO 2.5". 
XcenterofVane= 0.0508       # X COORDINATE OF THE VANE CENTER 
YcenterofVane= 0.0508       # Y COORDINATE OF THE VANE CENTER
iniZcenterofVane= 0.2435    # INITIAL Z CCORDINATE OF THE VANE CENTER-- FROM RHINO 7
############################################
###############   MATERIALS   ##############
############################################
Regolith=O.materials.append(FrictMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity))
Stiff=O.materials.append(FrictMat(young=regolithYoung,poisson=stiffPoisson,frictionAngle=stiffFrictDegree,density=stiffDensity))

# ###################################
# #####   CREATING GEOMETRIES   #####
# ###################################
# O.periodic=True
# O.cell.hSize=Matrix3(10,0,0, 0,10,0, 0,0,10)

sp = pack.SpherePack()
# generate randomly spheres with uniform radius distribution
sp.makeCloud((0.01, 0.01, 0.5), (5.08, 5.08, 5.5), rMean=.20, rRelFuzz=.1)
# add the sphere pack to the simulation
sp.toSimulation()
print(O.bodies[-1].mask)

# EO = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Final_Vane.stl',material=Stiff,shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# for i in EO:
#   i.state.mass = 1
#   i.state.inertia = (1,1,1)
# clumpId,facetsId = O.bodies.appendClumped(EO)
# s = O.bodies[clumpId].state
# s.blockedDOFs = 'xyzXY'

# Vane= O.bodies.append(EO)
# for i in Vane:
#   shearVane= O.bodies[i]
#   shearVane.dynamic=True
#   shearVane.state.blockedDOFs='xyzXYZ' # THE ONLY FREE DOFs ARE TRANSLATION ALONG AND ROTATION ABOUT Z. AT FIRST ALL DOFs ARE RESTRIANED. MOVEMENT (USING CombinedKinematicEngine)
# # WILL FREE DOFs IN THE DIRECTION OF MOVEMENT. ALL OTHER DOFs ARE STILL CONSTRAINED TO AVOID DEVIATION OF THE TOOL FROM Z AXIS.  
# VaneID = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST OF FACETS IN THE IMPORTED VANE
## IMPORTING THE RADIAL BOUNDARIES
# E1 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Radial_Periodic_Boundary.stl',material=Stiff,shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Radial= O.bodies.append(E1)

E1 = ymport.stl('/home/ngoudarzi/Desktop/Radial Periodic Boundary Condition for Vane Shear Test/Trial1-Full_Boundary.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
Radial= O.bodies.append(E1)
O.bodies[-1].groupMask=3
print(O.bodies[-1].mask)

E2 = ymport.stl('/home/ngoudarzi/Desktop/Radial Periodic Boundary Condition for Vane Shear Test/Trial1-Simple_Vane-tenDegrees.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
Vane= O.bodies.append(E2)
O.bodies[-1].groupMask=3
print(O.bodies[-1].mask)

## IMPORTING THE BOTTOM BOUNDARY
# E2 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Bottom_Plane_Deposition.stl',material=Stiff,shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Bottom= O.bodies.append(E2)

## IMPORTING THE LEFT BOUNDARY
# E3 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Left_Boundary.stl',material=Stiff,shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Left= O.bodies.append(E3)
for b in O.bodies:
  if isinstance(b.shape,Sphere):
    R_1=((b.state.pos[0]+b.shape.radius)**2+(b.state.pos[1]+b.shape.radius)**2)**0.5
    R_2=((b.state.pos[0]-b.shape.radius)**2+(b.state.pos[1]+b.shape.radius)**2)**0.5 
    R_3=((b.state.pos[0]+b.shape.radius)**2+(b.state.pos[1]-b.shape.radius)**2)**0.5
    R_4=((b.state.pos[0]-b.shape.radius)**2+(b.state.pos[1]-b.shape.radius)**2)**0.5        
    if R_1<=3.08 or R_1>=5.08 or R_2<=3.08 or R_2>=5.08 or  R_3<=3.08 or R_3>=5.08 or R_4<=3.08 or R_4>=5.08:
      O.bodies.erase(b.id)
maxR=max([b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
minR=min([b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
Offset=minR
############################
###   DEFINING ENGINES   ###
############################
O.engines=[
ForceResetter(),
InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb()], label="collider"),
InteractionLoop(
[Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
[Ip2_FrictMat_FrictMat_FrictPhys()],
[Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  NewtonIntegrator(damping=numDamp,gravity=(0,0,gravityAcc)),
  PyRunner(iterPeriod=1, command="grainDeposition()" ,label='checker'),
  #PyRunner(iterPeriod=50,command='history()',label='recorder'),
  ]
O.dt = 6e-5 #SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.

def grainDeposition():
  #collider.avoidSelfInteractionMask = 2
  global sphereRadii, nodelist, sphereids
  import numpy as numpy
  teta=math.pi/2
  eta=-math.pi/2
  cosTeta=numpy.cos(teta)
  sinTeta=numpy.sin(teta)
  cosEta=numpy.cos(eta)
  sinEta=numpy.sin(eta)

  maxX=max([b.state.pos[0]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
  minX=min([b.state.pos[0]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
  maxY=max([b.state.pos[1]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
  minY=max([b.state.pos[1]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
  pos_x=[]
  pos_y=[]
  pos_xy=[]
  sphereids6=[]
  sphereids9=[]
  sphereRadii=[]
  nodelist=[]
  sphereids=[]

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if b.mask==6:
        sphereids6.append([b.id])
      if b.mask==9:
        sphereids9.append([b.id])        
      if b.mask==6 or b.mask==9:
        sphereRadii.append([b.shape.radius])
        nodelist.append([b.state.pos[0],b.state.pos[1],b.state.pos[2]])
        sphereids.append([b.id])  

  sphereids6 = numpy.array(sphereids6).reshape(-1,1)  
  sphereids9 = numpy.array(sphereids9).reshape(-1,1)  
  sphereRadii = numpy.array(sphereRadii).reshape(-1,1)  
  nodelist = numpy.array(nodelist).reshape(-1,3) 
  sphereids = numpy.array(sphereids).reshape(-1,1)  

  for b in O.bodies:
    b.groupMask=3
    if isinstance(b.shape,Sphere):

      if ((b.state.pos[0])<0) and (b.state.vel[0]<0):
        pos_x.append([b.id])

      if ((b.state.pos[1])<0) and (b.state.vel[1]<0):
        pos_y.append([b.id])

  pos_x = numpy.array(pos_x).reshape(-1,1)
  pos_y = numpy.array(pos_y).reshape(-1,1)
  pos_xy=numpy.concatenate([numpy.array([pos_x]).reshape(-1,1),numpy.array([pos_y]).reshape(-1,1)])

  all_maskids=[]

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if (sphereRadii.size!=0):      
        if b.id in sphereids:
          center=numpy.array([b.state.pos[0],b.state.pos[1],b.state.pos[2]])
          center=center.reshape(1,3)
          radius=b.shape.radius 

          # Obtain extents for floating bin
          binMin = numpy.array(([center[0,0]-radius-maxR-Offset,\
            center[0,1]-radius-maxR-Offset,center[0,2]-\
            radius-maxR-Offset]))
          binMax = numpy.array(([center[0,0]+radius+maxR+Offset,\
            center[0,1]+radius+maxR+Offset,center[0,2]+\
            radius+maxR+Offset]))

          maskids = checkOverlap(center,radius,binMin,binMax,Offset,sphereRadii,nodelist,sphereids)

          if (maskids.size>1) or ((maskids.size==1) and (b.id not in maskids)):
            if b.id in sphereids6:
              b.groupMask=6
            if b.id in sphereids9:
              b.groupMask=9
            if numpy.array(all_maskids).size==0:
              all_maskids=maskids           

            all_maskids=numpy.concatenate([numpy.array([all_maskids]).reshape(-1),numpy.array([maskids]).reshape(-1)])
  
  if numpy.array(all_maskids).size!=0:
    all_maskids=numpy.unique(numpy.array(all_maskids)).reshape(-1,1) 
      
    for b in O.bodies:
      if isinstance(b.shape,Sphere):
        if (b.id in all_maskids) and (b.id in sphereids6):
          b.groupMask=6
        if (b.id in all_maskids) and (b.id in sphereids9):
          b.groupMask=9           

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if b.id in pos_x:
        b.state.pos_x=(b.state.pos[0]*cosTeta)+(b.state.pos[1]*sinTeta)
        b.state.pos_y=(-(b.state.pos[0]*sinTeta))+(b.state.pos[1]*cosTeta)
        #b.state.pos_y=-b.shape.radius
        b.state.pos_z=b.state.pos[2]
        b.state.vel_x=(b.state.vel[0]*cosTeta)+(b.state.vel[1]*sinTeta)
        b.state.vel_y=(-(b.state.vel[0]*sinTeta))+(b.state.vel[1]*cosTeta)
        b.state.vel_z=b.state.vel[2]
        f_x=(O.forces.f(b.id)[0]*cosTeta)+(O.forces.f(b.id)[1]*sinTeta)
        f_y=(-(O.forces.f(b.id)[0]*sinTeta))+(O.forces.f(b.id)[1]*cosTeta)
        f_z=O.forces.f(b.id)[2]   

      if b.id in pos_y:
        b.state.pos_x=(b.state.pos[0]*cosEta)+(b.state.pos[1]*sinEta)
        #b.state.pos_x=-b.shape.radius     
        b.state.pos_y=(-(b.state.pos[0]*sinEta))+(b.state.pos[1]*cosEta)
        b.state.pos_z=b.state.pos[2]
        b.state.vel_x=(b.state.vel[0]*cosEta)+(b.state.vel[1]*sinEta)
        b.state.vel_y=(-(b.state.vel[0]*sinEta))+(b.state.vel[1]*cosEta)
        b.state.vel_z=b.state.vel[2]
        f_x=(O.forces.f(b.id)[0]*cosEta)+(O.forces.f(b.id)[1]*sinEta)
        f_y=(-(O.forces.f(b.id)[0]*sinEta))+(O.forces.f(b.id)[1]*cosEta)
        f_z=O.forces.f(b.id)[2]

      if b.id in pos_xy:

        center=numpy.array([b.state.pos_x,b.state.pos_y,b.state.pos_z])
        center=center.reshape(1,3)
        radius=b.shape.radius     

        sphereRadii=[]
        nodelist=[]
        sphereids=[]
        nodelist, sphereRadii, sphereids = updatedParticles(nodelist,sphereRadii,sphereids)
        maskids=[]

        # Obtain extents for floating bin
        binMin = numpy.array(([center[0,0]-radius-maxR-Offset,\
          center[0,1]-radius-maxR-Offset,center[0,2]-\
          radius-maxR-Offset]))
        binMax = numpy.array(([center[0,0]+radius+maxR+Offset,\
          center[0,1]+radius+maxR+Offset,center[0,2]+\
          radius+maxR+Offset]))

        # Check if new position of particle overlapping any existing particles in the other side
        maskids = checkOverlap(center,radius,binMin,binMax,Offset,sphereRadii,nodelist,sphereids)

        if (maskids.size!=0):
          if ((b.state.pos[0])<2*maxR) and (b.mask==3):
            b.groupMask=6
          if ((b.state.pos[1])<2*maxR) and (b.mask==3):
            b.groupMask=9

          for body in O.bodies:
            if isinstance(body.shape,Sphere):           
              if body.id in maskids:
                if ((body.state.pos[0])<2*maxR) and (body.mask==3):
                  body.groupMask=6
                if ((body.state.pos[1])<2*maxR) and (body.mask==3):
                  body.groupMask=9

        #collider.avoidSelfInteractionMask = 2
        b.state.pos=Vector3(b.state.pos_x,b.state.pos_y,b.state.pos_z)
        b.state.vel=Vector3(b.state.vel_x,b.state.vel_y,b.state.vel_z)
        O.forces.f=Vector3(f_x,f_y,f_z)

  #collider.avoidSelfInteractionMask = 2
  # unb=unbalancedForce()
  # increments=O.iter
  # if unb<0.01 and increments>50000:
  #   export.text('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Rep_Sample.txt')
  #   O.pause()

def checkOverlap(center,radius,binMin,binMax,Offset,sphereRadii,nodelist,sphereids):

  binTestParticles = numpy.all([(nodelist[:,0] > binMin[0]) , \
    (nodelist[:,0] < binMax[0]) , (nodelist[:,1] > binMin[1]) , \
    (nodelist[:,1] < binMax[1]) , (nodelist[:,2] > binMin[2]) , \
    (nodelist[:,2] < binMax[2])],axis=0)
  existingNodes = nodelist[binTestParticles,:]
  existingR = sphereRadii[binTestParticles]
  existingid = sphereids[binTestParticles]

  # Compute distance between particles 
  if len(existingNodes>0):
    nodalDistance = numpy.linalg.norm(center-existingNodes, axis=1)
    OffsetDist = nodalDistance - radius - existingR - Offset
  else: 
    OffsetDist = numpy.array(([1]))

  overlapParticles=numpy.all(OffsetDist<0,axis=0)
  maskids=existingid[overlapParticles]


  # Kill and return if overlap
  #if (OffsetDist<0).any():
    #print('true')    
    #return True

  #return False
  return maskids  

def updatedParticles(nodelist,sphereRadii,sphereids):

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      sphereRadii.append([b.shape.radius])
      nodelist.append([b.state.pos[0],b.state.pos[1],b.state.pos[2]])
      sphereids.append([b.id])
  sphereRadii = numpy.array(sphereRadii).reshape(-1,1)  
  nodelist = numpy.array(nodelist).reshape(-1,3) 
  sphereids = numpy.array(sphereids).reshape(-1,1) 

  return nodelist,sphereRadii,sphereids

