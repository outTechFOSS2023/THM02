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
vaneRotVel=1.75    # VANE ROTATION VELOCITY AFTER PENETRATION (rad/s). NOTE: THE EXPERIMENTAL ANGULAR VELOCITY IS 0.1 deg/s OR 0.00175 rad/s. SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.
VanePenetDepth= 0.0635      # VANE PENETRATION DEPTH.
VaneDist=0.0                # DISTANCE OF THE VANE BOTTOM TO THE REGOLITH TOP
VaneCenToVaneBotZ= 0.0635   # THIS IS THE LENGTH OF THE VANE (2.5"). THE LENGTH OF THE ROD ABOVE THE VANE IS ALSO 2.5". 
XcenterofVane= 0.0508       # X COORDINATE OF THE VANE CENTER 
YcenterofVane= 0.0508       # Y COORDINATE OF THE VANE CENTER
iniZcenterofVane= 0.2435    # INITIAL Z CCORDINATE OF THE VANE CENTER-- FROM RHINO 7
Offset=0.001
############################################
######-#########   MATERIALS   ##############
############################################
Regolith=O.materials.append(FrictMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity))
Stiff=O.materials.append(FrictMat(young=regolithYoung,poisson=stiffPoisson,frictionAngle=stiffFrictDegree,density=stiffDensity))

# ###################################
# #####   CREATING GEOMETRIES   #####
# ###################################
# O.periodic=True
# O.cell.hSize=Matrix3(11,0,0, 0,11,0, 0,0,10)
#O.cell.hSize=Matrix3(-5.080,0,0, 0,-5.080,0, 0,0,10)

## GENERATING TWO SPHERES 

# O.bodies.append([sphere(center=(0.646,4.029,3.000), radius=.05),sphere((0.646,4.029,1.750), .2)])
# O.bodies.append([sphere(center=(4.075,0.21,1.750), radius=.2),sphere((4.075,0.21,3.000), .05)])
# O.bodies.append([sphere(center=(0.323,4.095,1.500), radius=0.025),sphere((0.323,4.095,0.875), .1)])

O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Radial Periodic Boundary Condition for Vane Shear Test/Rep_Sample.txt'))

# IMPORTING THE VANE
# NOTE: THE VANE IS ALREADY IN THE CORRECT POSITION IN SPACE
# E0 = ymport.stl('/home/THM_Works/Desktop/Radial PBC/Trial1-Simple_Vane-tenDegrees.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Vane= O.bodies.append(E0)
# for i in Vane:
#   shearVane= O.bodies[i]
#   shearVane.dynamic=True
#   shearVane.state.blockedDOFs='xyzXYZ' # THE ONLY FREE DOFs ARE TRANSLATION ALONG AND ROTATION ABOUT Z. AT FIRST ALL DOFs ARE RESTRIANED. MOVEMENT (USING CombinedKinematicEngine)
# # WILL FREE DOFs IN THE DIRECTION OF MOVEMENT. ALL OTHER DOFs ARE STILL CONSTRAINED TO AVOID DEVIATION OF THE TOOL FROM Z AXIS.  
# VaneID = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST OF FACETS IN THE IMPORTED VANE

E0 = ymport.stl('home/ngoudarzi/Desktop/Radial Periodic Boundary Condition for Vane Shear Test/Trial1-Simple_Vane-tenDegrees.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
for i in E0:
  i.state.mass = 1
  i.state.inertia = (1,1,1)
clumpId,facetsId = O.bodies.appendClumped(E0)
s = O.bodies[clumpId].state
s.blockedDOFs = 'xyzXY'

# IMPORTING THE RADIAL BOUNDARIES
E1 = ymport.stl('/home/ngoudarzi/Desktop/Radial Periodic Boundary Condition for Vane Shear Test/Trial1-Full_Boundary.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
Radial= O.bodies.append(E1)

# E1 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Radial_Periodic_Boundary.stl',material=Stiff,shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Radial= O.bodies.append(E1)

# E1 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Full_Boundary.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Radial= O.bodies.append(E1)

#IMPORTING THE BOTTOM BOUNDARY
# E2 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Full_Bottom_Boundary.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Bottom= O.bodies.append(E2)


# E2 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Bottom_Plane.stl',material=Stiff,shift=Vector3(5.080,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Bottom= O.bodies.append(E2)
#PreventiveBoundary=O.bodies.append(yade.geom.facetBox((5.08,5.08,3), (5.08-0.05,5.08-0.05,2.5), orientation=Quaternion((1, 0, 0), 0), wallMask=26,wire=False))

maxY=max([b.state.pos[1]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
minY=min([b.state.pos[1]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
print ("minY:",minY,"maxY:",maxY)

maxR=max([b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
minR=min([b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])

for b in O.bodies:
  if isinstance(b.shape,Facet):
    b.groupMask=3
# E3 = ymport.stl('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Radial Periodic Boundary Conditions/Cad/Trial1-Preventive_Boundary.stl',material=Stiff,shift=Vector3(0,0,0),wire=False,color=Vector3(0.5,0.6,0.8))
# Preventive= O.bodies.append(E3)
############################
###   DEFINING ENGINES   ###
############################
O.engines=[
ForceResetter(),
InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb()], allowBiggerThanPeriod=True,label="collider"),
InteractionLoop(
[Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
[Ip2_FrictMat_FrictMat_FrictPhys()],
[Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  CombinedKinematicEngine(ids=[clumpId],label='combEngine') + TranslationEngine(translationAxis=(0,0,-1),velocity=0) +\
  RotationEngine(rotationAxis=(0,0,1), angularVelocity=0, rotateAroundZero=True, zeroPoint=(0,0,0)),
  NewtonIntegrator(damping=numDamp,gravity=(0,0,gravityAcc)),
  PyRunner(iterPeriod=1000, command="vaneShear()" ,label='checker'),
  #PyRunner(iterPeriod=50,command='history()',label='recorder'),
  ]
O.dt = 1e-7 #SUBJECT TO SENSITIVITY ANALYSES TO CONFIRM STABILITY AND QUASI-STATIC CONDITION.

## GETTING TranslationEngine and RotationEngine FROM CombinedKinematicEngine
transEngine, rotEngine = combEngine.comb[0], combEngine.comb[1]
generated_in_y=[]
generated_in_x=[]
IDs=[]

def grainDeposition():
  unb=unbalancedForce()
  increments=O.iter
  if unb<0.01 and increments>50000:
    checker.command='vaneShear()'

def vaneShear():
  global sphereRadii, nodelist, sphereids, generated_in_y, generated_in_x, IDs
  generatedtem_in_x=[]
  generatedtem_in_y=[]
  IDstem=[]

  rotEngine.angularVelocity = vaneRotVel
  for i in E0:
    vaneRotZ=((i.state.rot().norm())*(180/math.pi))
  #vaneRotZ=(shearVane.state.rot().norm())*(180/math.pi) # GETTING THE ROTATION OF THE VANE (DIFFERENCE BETWEEN refOri AND ori AND CONVERSION TO DEGREES)
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
  sphereRadii6=[]
  nodelist6=[]
  sphereRadii9=[]
  nodelist9=[]

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if b.mask==6:
        sphereids6.append([b.id])  
        sphereRadii6.append([b.shape.radius])
        nodelist6.append([b.state.pos[0],b.state.pos[1],b.state.pos[2]])
      if b.mask==9:
        sphereids9.append([b.id])        
        sphereRadii9.append([b.shape.radius])
        nodelist9.append([b.state.pos[0],b.state.pos[1],b.state.pos[2]])

  sphereids6 = numpy.array(sphereids6).reshape(-1,1)  
  sphereids9 = numpy.array(sphereids9).reshape(-1,1)  
  sphereRadii6 = numpy.array(sphereRadii6).reshape(-1,1)  
  nodelist6= numpy.array(nodelist6).reshape(-1,3) 
  sphereRadii9 = numpy.array(sphereRadii9).reshape(-1,1)  
  nodelist9= numpy.array(nodelist9).reshape(-1,3) 

  for b in O.bodies:
    b.groupMask=3
    if isinstance(b.shape,Sphere):

      if ((b.state.pos[0])<0) and (b.state.vel[0]<0) and (b.id not in generated_in_y):
        pos_x.append([b.id])

      if ((b.state.pos[1])<0) and (b.state.vel[1]<0) and (b.id not in generated_in_x):
        pos_y.append([b.id])

  pos_x = numpy.array(pos_x).reshape(-1,1)
  pos_y = numpy.array(pos_y).reshape(-1,1)
  pos_xy=numpy.concatenate([numpy.array([pos_x]).reshape(-1,1),numpy.array([pos_y]).reshape(-1,1)])

  all_maskids=[]

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if sphereids6.size!=0 or sphereids9.size!=0:
        if b.id in sphereids6 or b.id in sphereids9:
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

          if b.id in sphereids6:
            maskids = checkOverlap(center,radius,binMin,binMax,Offset,sphereRadii9,nodelist9,sphereids9)

          if b.id in sphereids9:
            maskids = checkOverlap(center,radius,binMin,binMax,Offset,sphereRadii6,nodelist6,sphereids6)

          if maskids.size!=0:
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
      if (b.mask==6) and (b.id in IDs[:,1]):
        #irow=numpy.where((IDs[:,1] == b.id).all(axis=1))[0]
        irow=numpy.where(IDs[:,1] == b.id)[0][0]
        for body in O.bodies:
          if isinstance(body.shape,Sphere):
            if body.id==IDs[irow,0]:

              vel_x=(body.state.vel[0]*cosTeta)+(body.state.vel[1]*sinTeta)
              vel_y=(-(body.state.vel[0]*sinTeta))+(body.state.vel[1]*cosTeta)
              vel_z=body.state.vel[2]
              f_x=(O.forces.f(body.id)[0]*cosTeta)+(O.forces.f(body.id)[1]*sinTeta)
              f_y=(-(O.forces.f(body.id)[0]*sinTeta))+(O.forces.f(body.id)[1]*cosTeta)
              f_z=O.forces.f(body.id)[2] 
        b.state.vel=Vector3(vel_x,vel_y,vel_z)      
        O.forces.f=Vector3(f_x,f_y,f_z)   

      if (b.mask==9) and (b.id in IDs[:,1]):
        irow=numpy.where(IDs[:,1] == b.id)[0][0]
        for body in O.bodies:
          if isinstance(body.shape,Sphere):
            if body.id==IDs[irow,0]:

              vel_x=(body.state.vel[0]*cosEta)+(body.state.vel[1]*sinEta)
              vel_y=(-(body.state.vel[0]*sinEta))+(body.state.vel[1]*cosEta)
              vel_z=body.state.vel[2]
              f_x=(O.forces.f(body.id)[0]*cosEta)+(O.forces.f(body.id)[1]*sinEta)
              f_y=(-(O.forces.f(body.id)[0]*sinEta))+(O.forces.f(body.id)[1]*cosEta)
              f_z=O.forces.f(body.id)[2] 
        b.state.vel=Vector3(vel_x,vel_y,vel_z)      
        O.forces.f=Vector3(f_x,f_y,f_z)   

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if b.id in pos_x:
        b.state.pos_x=(b.state.pos[0]*cosTeta)+(b.state.pos[1]*sinTeta)
        b.state.pos_y=(-(b.state.pos[0]*sinTeta))+(b.state.pos[1]*cosTeta)
        b.state.pos_z=b.state.pos[2]
        b.state.vel_x=(b.state.vel[0]*cosTeta)+(b.state.vel[1]*sinTeta)
        b.state.vel_y=(-(b.state.vel[0]*sinTeta))+(b.state.vel[1]*cosTeta)
        b.state.vel_z=b.state.vel[2]
        f_x=(O.forces.f(b.id)[0]*cosTeta)+(O.forces.f(b.id)[1]*sinTeta)
        f_y=(-(O.forces.f(b.id)[0]*sinTeta))+(O.forces.f(b.id)[1]*cosTeta)
        f_z=O.forces.f(b.id)[2] 
        generatedtem_in_y.append([b.id])  

      elif b.id in pos_y:
        b.state.pos_x=(b.state.pos[0]*cosEta)+(b.state.pos[1]*sinEta)
        b.state.pos_y=(-(b.state.pos[0]*sinEta))+(b.state.pos[1]*cosEta)
        b.state.pos_z=b.state.pos[2]
        b.state.vel_x=(b.state.vel[0]*cosEta)+(b.state.vel[1]*sinEta)
        b.state.vel_y=(-(b.state.vel[0]*sinEta))+(b.state.vel[1]*cosEta)
        b.state.vel_z=b.state.vel[2]
        f_x=(O.forces.f(b.id)[0]*cosEta)+(O.forces.f(b.id)[1]*sinEta)
        f_y=(-(O.forces.f(b.id)[0]*sinEta))+(O.forces.f(b.id)[1]*cosEta)
        f_z=O.forces.f(b.id)[2]
        generatedtem_in_x.append([b.id]) 

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

        O.bodies.append(sphere(center=(b.state.pos_x,b.state.pos_y,b.state.pos_z), radius=b.shape.radius))
        newParticles=O.bodies[-1]
        newParticles.state.vel=Vector3(b.state.vel_x,b.state.vel_y,b.state.vel_z)      
        O.forces.f=Vector3(f_x,f_y,f_z) 
        IDstem.append([b.id, O.bodies[-1].id])

        if (maskids.size!=0):
          if ((b.state.pos[0])<maxR):
            O.bodies[-1].groupMask=6
          if ((b.state.pos[1])<maxR):
            O.bodies[-1].groupMask=9

          for body in O.bodies:
            if isinstance(body.shape,Sphere):           
              if body.id in maskids:
                if ((body.state.pos[0])<maxR) and (body.mask==3):
                  body.groupMask=6
                if ((body.state.pos[1])<maxR) and (body.mask==3):
                  body.groupMask=9


  generatedtem_in_x=numpy.array(generatedtem_in_x)
  generatedtem_in_y=numpy.array(generatedtem_in_y)
  IDstem=numpy.array(IDstem)

  if numpy.array(generated_in_x).size==0:
    generated_in_x=generatedtem_in_x
  generated_in_x=numpy.concatenate([numpy.array([generatedtem_in_x]).reshape(-1),numpy.array([generated_in_x]).reshape(-1)])
  generated_in_x=numpy.unique(generated_in_x)

  if numpy.array(generated_in_y).size==0:
    generated_in_y=generatedtem_in_y
  generated_in_y=numpy.concatenate([numpy.array([generatedtem_in_y]).reshape(-1),numpy.array([generated_in_y]).reshape(-1)])
  generated_in_y=numpy.unique(generated_in_y)

  if numpy.array(IDs).size==0:
    IDs=IDstem
  if numpy.array(IDs).size!=0:
    IDs=numpy.concatenate([numpy.array([IDstem]).reshape(-1,2),numpy.array([IDs]).reshape(-1,2)])
    IDs=numpy.unique(numpy.array(IDs).reshape(-1,2), axis=0).reshape(-1,2)

  for b in O.bodies:
    if isinstance(b.shape,Sphere):
      if ((b.state.pos[0]+b.shape.radius)<0) and (b.id in generated_in_y):
        IDs=numpy.delete(IDs,numpy.where(IDs[:,0] == b.id)[0][0], axis=0)
        O.bodies.erase(b.id)        
      if ((b.state.pos[1]+b.shape.radius)<0) and (b.id in generated_in_x):
        IDs=numpy.delete(IDs,numpy.where(IDs[:,0] == b.id)[0][0], axis=0)  
        O.bodies.erase(b.id)              
  IDs=numpy.array(IDs).reshape(-1,2)  

  pos_x_vane=s.pos[0]
  pos_y_vane=s.pos[1]
  pos_z_vane=s.pos[2]
  #print ("pos_x_vane:", pos_x_vane,"pos_y_vane:", pos_y_vane,"pos_z_vane:", pos_z_vane)
  if pos_x_vane<0:
    s.ori=Quaternion((0.5774129906785795407,-0.5773189004710571615,0.5773189113085662738),2.09430103916681265)
    s.pos=Vector3((s.pos[0]*cosEta)+(s.pos[1]*sinEta),(-(s.pos[0]*sinEta))+(s.pos[1]*cosEta),s.pos[2])
       

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
  #print(maskids)

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


        #O.pause()
        # b.state.pos[0]=(b.state.pos[0]*cosTeta)+(b.state.pos[1]*sinTeta)
        # b.state.pos[1]=(-(b.state.pos[0]*sinTeta))+(b.state.pos[1]*cosTeta)
  #print ("vaneRotZ:", vaneRotZ) 
  #print ("pos_x:", pos_x) 

  # if vaneRotZ>=90:
  #   O.pause() # EXPERMENT IS STOPPED AFTER 90 degrees OF ROTATION

# def history():
#   global vaneFx,vaneFy,vaneFz,vaneTx,vaneTy,vaneTz,vaneDx,vaneDy,vaneDz,vaneRotZ
#   vaneFx=0
#   vaneFy=0
#   vaneFz=0
#   vaneTx=0
#   vaneTy=0
#   vaneTz=0
#   RXFY=0
#   RyFx=0
#   for b in VaneID:
#     vaneFx+=O.forces.f(b.id,sync=True)[0]
#     vaneFy+=O.forces.f(b.id,sync=True)[1]
#     vaneFz+=O.forces.f(b.id,sync=True)[2]
#     RxFy=(b.state.pos[0]-XcenterofVane)*O.forces.f(b.id,sync=True)[1]
#     RxFz=(b.state.pos[0]-XcenterofVane)*O.forces.f(b.id,sync=True)[2]    
#     RyFx=(b.state.pos[1]-YcenterofVane)*O.forces.f(b.id,sync=True)[0]
#     RyFz=(b.state.pos[1]-YcenterofVane)*O.forces.f(b.id,sync=True)[2]
#     RzFx=(b.state.pos[2])*O.forces.f(b.id,sync=True)[0]    
#     RzFy=(b.state.pos[2])*O.forces.f(b.id,sync=True)[1]          
#     vaneTx+=(RyFz-RzFy)
#     vaneTy+=(RxFz-RzFx)
#     vaneTz+=(RxFy-RyFx)
#   vaneDx=rotEngine.zeroPoint[0]
#   vaneDy=rotEngine.zeroPoint[1]
#   vaneDz=rotEngine.zeroPoint[2]
#   vaneRotZ=(shearVane.state.rot().norm())*(180/math.pi) 
#   yade.plot.addData({'vaneRotZ':vaneRotZ,'vaneTz':vaneTz,})
#   ## SAVING THE DATA TO A TEXT FILE AT THE INTERVALS DEFINED FOR history() FUNCTION
#   plot.saveDataTxt('/home/ngoudarzi/Desktop/Vane Shear Test/Light PSD Version/Homogeneous/YADE/Executables/Output/Regular_Vane_in_Place_PSD4_SI/Regular_Vane_in_Place_PSD4_SI.txt')
  
# ## PLOTTING ON THE SCREEN
# plot.plots={'vaneRotZ':('vaneTz')}
# plot.plot()
  


# # O.run(2000000000000,True)
