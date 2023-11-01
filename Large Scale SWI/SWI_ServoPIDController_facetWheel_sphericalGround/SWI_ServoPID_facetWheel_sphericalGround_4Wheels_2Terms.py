# -*- coding: utf-8 -*-
#*********************************************************************************************************
#*********************************************************************************************************
# "SBIR Rights Notice (DEC 2007)
# These SBIR data are furnished with SBIR rights under Contract No. 80NSSC21C0483. 
# For a period of 20 years, unless extended in accordance with FAR 27.409(h), after 
# acceptance of all items to be delivered under this contract, the Government will 
# use these data for Government purposes only, and they shall not be disclosed outside 
# the Government (including disclosure for procurement purposes) during such period 
# without permission of the Contractor, except that, subject to the foregoing use 
# and disclosure prohibitions, these data may be disclosed for use by support Contractors. 
# After the protection period, the Government has a paid-up license
# # to use, and to authorize others to use on its behalf, these data for Government purposes, 
# but is relieved of all disclosure prohibitions and assumes no liability for unauthorized 
# use of these data by third parties. This notice shall be affixed to any reproductions of 
# these data, in whole or in part.
# (End of Notice)"
#*********************************************************************************************************
##############################
## IMPORT NECESSARY MODULES ##
##############################
from yade import pack, timing, qt
from yade.params import *
from math import pi
from yade import polyhedra_utils
from yade import ymport
from yade import plot

########################################################
## CREATE FRICTMAT FOR BOTH THE GROUND AND WHEEL ##
########################################################
regolithFrictDegree=30      # INTERPARTICLE FRICTION DEGREE
regolithYoung=0.70e8        # CONTACT STIFFNESS
regolithPoisson=0.42        # POISSON RATIO
regolithDensity=1774        # DENSITY
Regolith=O.materials.append(FrictMat(young=regolithYoung,poisson=regolithPoisson,frictionAngle=radians(regolithFrictDegree),density=regolithDensity))

###########################################################
######### IMPORTING A FACET WHEEL OF 0.15m RADIUS #########
###########################################################
facetWheel1=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/scaledWheelRegularLight.mesh',wire=False,color=Vector3(1,1,1),shift=Vector3(0,0,0),scale=1)

#facetWheel1=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelFinal.mesh',shift=Vector3(0,0.37382716/4,0),scale=1.0)
#facetWheel1 = ymport.stl('/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelRefined.stl',scale=1,shift=Vector3(0,0,-0.001),wire=True,color=Vector3(1.0,1.0,1.0))
num_facets = len(facetWheel1)
print("num_facets:",num_facets) 
## A CONSTANT NORMAL FORCE OFV 80 N IS ASSUMED
g = 9.81 # m/s
wheel_volume    = 0.00610556374 #m^3. FROM CAD
wheel_density   = 7850.00000000 #kg/m^3. s
wheel_radius    = 0.15000000000 #m. FROM CAD 
wheel_width     = 0.10000000000 #m. FROM CAD
wheel_self_mass = wheel_volume*wheel_density
vehicle_mass    = 430           # KG
vehicle_mass_single_wheel = vehicle_mass/4 
total_mass_single_wheel = wheel_self_mass+vehicle_mass_single_wheel
wheel_updated_density = vehicle_mass_single_wheel*wheel_density/wheel_self_mass # THE EFFECT OF ADDITIONAL NORMAL FORCE FOR MOMENT OF INERTIA
print("wheel_updated_density:",wheel_updated_density) 

#VOLUME-MOMENTS FROM CAD
Vx= 3.50391917e-05
Vy= 5.99024439e-05
Vz= 3.50391917e-05
Ix= Vx*wheel_updated_density
Iy= Vy*wheel_updated_density
Iz= Vz*wheel_updated_density

for i in facetWheel1:
  i.state.mass = total_mass_single_wheel/num_facets
  i.state.inertia = (1,1,1) # THIS IS TEMPORARY
facetWheelID1 = O.bodies.appendClumped(facetWheel1)
#O.bodies.updateClumpProperties()
Wheel1=O.bodies[-1]
wheel_mass = Wheel1.state.mass
# Ix = ((wheel_mass*(wheel_radius)**2)/4)+ ((wheel_mass*(wheel_width)**2)/12)  #m^4. MANULLY COMPUTED
# Iy = (wheel_mass*(wheel_radius)*(wheel_radius))/2                            #m^4. MANULLY COMPUTED
# Iz = Ix                                                                      #m^4. MANULLY COMPUTED
# ## UPDATING MOMENTS OF INERTIA FOR THE CLUMPED WHEEL
Wheel1.state.inertia=Vector3(Ix,Iy,Iz)
facets = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST ALL FACETS 
print("inertia:",Wheel1.state.inertia,"mass:",Wheel1.state.mass) 

facetWheel2=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/scaledWheelRegularLight.mesh',wire=False,color=Vector3(1,1,1),shift=Vector3(0.70,0,0),scale=1)

#facetWheel2=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelFinal.mesh',shift=Vector3(0,0.37382716/4,0),scale=1.0)
#facetWheel2 = ymport.stl('/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelRefined.stl',scale=1,shift=Vector3(0,0,-0.001),wire=True,color=Vector3(1.0,1.0,1.0))
num_facets = len(facetWheel2)
print("num_facets:",num_facets) 
## A CONSTANT NORMAL FORCE OFV 80 N IS ASSUMED
g = 9.81 # m/s
wheel_volume    = 0.00610556374 #m^3. FROM CAD
wheel_density   = 7850.00000000 #kg/m^3. s
wheel_radius    = 0.15000000000 #m. FROM CAD 
wheel_width     = 0.10000000000 #m. FROM CAD
wheel_self_mass = wheel_volume*wheel_density
vehicle_mass    = 430           # KG
vehicle_mass_single_wheel = vehicle_mass/4 
total_mass_single_wheel = wheel_self_mass+vehicle_mass_single_wheel
wheel_updated_density = vehicle_mass_single_wheel*wheel_density/wheel_self_mass # THE EFFECT OF ADDITIONAL NORMAL FORCE FOR MOMENT OF INERTIA
print("wheel_updated_density:",wheel_updated_density) 

#VOLUME-MOMENTS FROM CAD
Vx= 3.50391917e-05
Vy= 5.99024439e-05
Vz= 3.50391917e-05
Ix= Vx*wheel_updated_density
Iy= Vy*wheel_updated_density
Iz= Vz*wheel_updated_density

for i in facetWheel2:
  i.state.mass = total_mass_single_wheel/num_facets
  i.state.inertia = (1,1,1) # THIS IS TEMPORARY
facetWheelID2 = O.bodies.appendClumped(facetWheel2)
#O.bodies.updateClumpProperties()
Wheel2=O.bodies[-1]
wheel_mass = Wheel2.state.mass
# Ix = ((wheel_mass*(wheel_radius)**2)/4)+ ((wheel_mass*(wheel_width)**2)/12)  #m^4. MANULLY COMPUTED
# Iy = (wheel_mass*(wheel_radius)*(wheel_radius))/2                            #m^4. MANULLY COMPUTED
# Iz = Ix                                                                      #m^4. MANULLY COMPUTED
# ## UPDATING MOMENTS OF INERTIA FOR THE CLUMPED WHEEL
Wheel2.state.inertia=Vector3(Ix,Iy,Iz)
facets = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST ALL FACETS 
print("inertia:",Wheel2.state.inertia,"mass:",Wheel2.state.mass) 


facetWheel3=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/scaledWheelRegularLight.mesh',wire=False,color=Vector3(1,1,1),shift=Vector3(0,0.7,0),scale=1)

#facetWheel3=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelFinal.mesh',shift=Vector3(0,0.37382716/4,0),scale=1.0)
#facetWheel3 = ymport.stl('/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelRefined.stl',scale=1,shift=Vector3(0,0,-0.001),wire=True,color=Vector3(1.0,1.0,1.0))
num_facets = len(facetWheel3)
print("num_facets:",num_facets) 
## A CONSTANT NORMAL FORCE OFV 80 N IS ASSUMED
g = 9.81 # m/s
wheel_volume    = 0.00610556374 #m^3. FROM CAD
wheel_density   = 7850.00000000 #kg/m^3. s
wheel_radius    = 0.15000000000 #m. FROM CAD 
wheel_width     = 0.10000000000 #m. FROM CAD
wheel_self_mass = wheel_volume*wheel_density
vehicle_mass    = 430           # KG
vehicle_mass_single_wheel = vehicle_mass/4 
total_mass_single_wheel = wheel_self_mass+vehicle_mass_single_wheel
wheel_updated_density = vehicle_mass_single_wheel*wheel_density/wheel_self_mass # THE EFFECT OF ADDITIONAL NORMAL FORCE FOR MOMENT OF INERTIA
print("wheel_updated_density:",wheel_updated_density) 

#VOLUME-MOMENTS FROM CAD
Vx= 3.50391917e-05
Vy= 5.99024439e-05
Vz= 3.50391917e-05
Ix= Vx*wheel_updated_density
Iy= Vy*wheel_updated_density
Iz= Vz*wheel_updated_density

for i in facetWheel3:
  i.state.mass = total_mass_single_wheel/num_facets
  i.state.inertia = (1,1,1) # THIS IS TEMPORARY
facetWheelID3 = O.bodies.appendClumped(facetWheel3)
#O.bodies.updateClumpProperties()
Wheel3=O.bodies[-1]
wheel_mass = Wheel3.state.mass
# Ix = ((wheel_mass*(wheel_radius)**2)/4)+ ((wheel_mass*(wheel_width)**2)/12)  #m^4. MANULLY COMPUTED
# Iy = (wheel_mass*(wheel_radius)*(wheel_radius))/2                            #m^4. MANULLY COMPUTED
# Iz = Ix                                                                      #m^4. MANULLY COMPUTED
# ## UPDATING MOMENTS OF INERTIA FOR THE CLUMPED WHEEL
Wheel3.state.inertia=Vector3(Ix,Iy,Iz)
facets = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST ALL FACETS 
print("inertia:",Wheel3.state.inertia,"mass:",Wheel3.state.mass) 

facetWheel4=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/scaledWheelRegularLight.mesh',wire=False,color=Vector3(1,1,1),shift=Vector3(0.7,0.7,0),scale=1)

#facetWheel4=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelFinal.mesh',shift=Vector3(0,0.37382716/4,0),scale=1.0)
#facetWheel4 = ymport.stl('/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelRefined.stl',scale=1,shift=Vector3(0,0,-0.001),wire=True,color=Vector3(1.0,1.0,1.0))
num_facets = len(facetWheel4)
print("num_facets:",num_facets) 
## A CONSTANT NORMAL FORCE OFV 80 N IS ASSUMED
g = 9.81 # m/s
wheel_volume    = 0.00610556374 #m^3. FROM CAD
wheel_density   = 7850.00000000 #kg/m^3. s
wheel_radius    = 0.15000000000 #m. FROM CAD 
wheel_width     = 0.10000000000 #m. FROM CAD
wheel_self_mass = wheel_volume*wheel_density
vehicle_mass    = 430           # KG
vehicle_mass_single_wheel = vehicle_mass/4 
total_mass_single_wheel = wheel_self_mass+vehicle_mass_single_wheel
wheel_updated_density = vehicle_mass_single_wheel*wheel_density/wheel_self_mass # THE EFFECT OF ADDITIONAL NORMAL FORCE FOR MOMENT OF INERTIA
print("wheel_updated_density:",wheel_updated_density) 

#VOLUME-MOMENTS FROM CAD
Vx= 3.50391917e-05
Vy= 5.99024439e-05
Vz= 3.50391917e-05
Ix= Vx*wheel_updated_density
Iy= Vy*wheel_updated_density
Iz= Vz*wheel_updated_density

for i in facetWheel4:
  i.state.mass = total_mass_single_wheel/num_facets
  i.state.inertia = (1,1,1) # THIS IS TEMPORARY
facetWheelID4 = O.bodies.appendClumped(facetWheel4)
#O.bodies.updateClumpProperties()
Wheel4=O.bodies[-1]
wheel_mass = Wheel4.state.mass
# Ix = ((wheel_mass*(wheel_radius)**2)/4)+ ((wheel_mass*(wheel_width)**2)/12)  #m^4. MANULLY COMPUTED
# Iy = (wheel_mass*(wheel_radius)*(wheel_radius))/2                            #m^4. MANULLY COMPUTED
# Iz = Ix                                                                      #m^4. MANULLY COMPUTED
# ## UPDATING MOMENTS OF INERTIA FOR THE CLUMPED WHEEL
Wheel4.state.inertia=Vector3(Ix,Iy,Iz)
facets = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST ALL FACETS 
print("inertia:",Wheel4.state.inertia,"mass:",Wheel4.state.mass)

########################################################
# MODELING GROUND AS A PRECOMPACTED GRANULAR ASSEMBLY ##
########################################################
O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/HW_Deposited_Bed.txt',shift=Vector3(0,0,0),material=Regolith))
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

O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/HW_Deposited_Bed.txt',shift=Vector3(maxX_bed,0,0),material=Regolith))
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


O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/HW_Deposited_Bed.txt',shift=Vector3(0,0.7,0),material=Regolith))
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

O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/HW_Deposited_Bed.txt',shift=Vector3(0.77382086,0.7,0),material=Regolith))
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



#####################################
## CREATE WALLS AROUND THE PACKING ##
#####################################

mn1,mx1=Vector3(minX_bed,minY_bed,minZ_bed),Vector3(maxX_bed,0.18691358,10*maxZ_bed)
walls=aabbWalls([mn1,mx1],thickness=0)
wallIds=O.bodies.append(walls)

mn2,mx2=Vector3(minX_bed,0.7,minZ_bed),Vector3(maxX_bed,0.18691358+0.7,10*maxZ_bed)
walls=aabbWalls([mn2,mx2],thickness=0)
wallIds=O.bodies.append(walls)


###############################
## DEFINE ENGINES PARAMETERS ##
###############################
T_angVel_max = 300 # MAY NOT BE REQUIRED
F_max = 300 # MAY NOT BE REQUIRED
T_start = 0.1
F_start = 30

######################
## DEFINE TIME STEP ##
######################
D_t = 1.0e-6

##############################
## DEFINE TARGET VALUES ##
##############################
target_lin_vel = 0.1 # m/s
target_ang_vel = 1.33 # rad/s
target_slip_ratio = 0.5 

########################################################
## DEFINE PID GAINS (SUBJECT TO SENSITIVITY ANALYSES) ##
########################################################
# LINEAR VELOCITY
kP_lin_vel_wheel1 = 0.1
kI_lin_vel_wheel1 = 0.01
kD_lin_vel_wheel1 = 0.0

kP_lin_vel_wheel2 = 0.1
kI_lin_vel_wheel2 = 0.01
kD_lin_vel_wheel2 = 0.0

kP_lin_vel_wheel3 = 0.1
kI_lin_vel_wheel3 = 0.01
kD_lin_vel_wheel3 = 0.0

kP_lin_vel_wheel4 = 0.1
kI_lin_vel_wheel4 = 0.01
kD_lin_vel_wheel4 = 0.0

# ANGULAR VELOCITY
kP_ang_vel_wheel1 = 1.33
kI_ang_vel_wheel1 = 0.133
kD_ang_vel_wheel1 = 0.0

kP_ang_vel_wheel2 = 1.33
kI_ang_vel_wheel2 = 0.133
kD_ang_vel_wheel2 = 0.0

kP_ang_vel_wheel3 = 1.33
kI_ang_vel_wheel3 = 0.133
kD_ang_vel_wheel3 = 0.0

kP_ang_vel_wheel4 = 1.33
kI_ang_vel_wheel4 = 0.133
kD_ang_vel_wheel4 = 0.0
###########################
## DEFINE INITIAL ERRORS ##
###########################
# LINEAR VELOCITY
prev_error_lin_vel_wheel1 = 0
iTerm_lin_vel_wheel1 = 0

prev_error_lin_vel_wheel2 = 0
iTerm_lin_vel_wheel2 = 0

prev_error_lin_vel_wheel3 = 0
iTerm_lin_vel_wheel3 = 0

prev_error_lin_vel_wheel4 = 0
iTerm_lin_vel_wheel4 = 0

# ANGULAR VELOCITY
prev_error_ang_vel_wheel1 = 0
iTerm_ang_vel_wheel1 = 0

prev_error_ang_vel_wheel2 = 0
iTerm_ang_vel_wheel2 = 0

prev_error_ang_vel_wheel3 = 0
iTerm_ang_vel_wheel3 = 0

prev_error_ang_vel_wheel4 = 0
iTerm_ang_vel_wheel4 = 0
#########################################
## DEFINE SIMULATION NEWTON INTEGRATOR ##
#########################################
newton=NewtonIntegrator(damping=0.8,gravity=(0,0,-g))

#############
## ENGINES ##
#############
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/VTK_SWI_ServoPID_facetWheel_sphericalGround_4Wheels_2Terms/SWI_ServoPID_facetWheel_sphericalGround_4Wheels_2Terms' ,recorders=['all'],iterPeriod=20000),
  ForceResetter(),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine1_ang_vel",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[Wheel2.id],label="torqueEngine2_ang_vel",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[Wheel3.id],label="torqueEngine3_ang_vel",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[Wheel4.id],label="torqueEngine4_ang_vel",moment=Vector3(0,T_start,0)),
  ForceEngine(dead=True,ids=[Wheel1.id],label="forceEngine1_lin_vel",force=Vector3(F_start,0,0)),
  ForceEngine(dead=True,ids=[Wheel2.id],label="forceEngine2_lin_vel",force=Vector3(F_start,0,0)),
  ForceEngine(dead=True,ids=[Wheel3.id],label="forceEngine3_lin_vel",force=Vector3(F_start,0,0)),
  ForceEngine(dead=True,ids=[Wheel4.id],label="forceEngine4_lin_vel",force=Vector3(F_start,0,0)),
  InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Box_Aabb(),Bo1_Facet_Aabb()]),
  InteractionLoop(
    [Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
    [Ip2_FrictMat_FrictMat_FrictPhys(label='ContactModel')],
    [Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  newton,
  PyRunner(command='wheelSettlement()',iterPeriod=100,label='checker'),
]
O.dt=D_t


#ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
def wheelSettlement():
    #Wheel1.blockedDOFs = 'xyXYZ' ## NOTE: ONLY MOVEMENT ALONG Z AXIS IS ALLOWED. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    Wheel1.dynamics = False
    Wheel2.dynamics = False
    Wheel3.dynamics = False
    Wheel4.dynamics = False
    for b in O.bodies:
        if isinstance(b.shape,Facet):
            b.blockedDOFs = 'xyzXYZ'
    for b in O.bodies:
        if isinstance(b.shape,Facet):
          if b.state.vel[0]!=0:
             b.state.vel[0]=-b.state.vel[0]
          if b.state.vel[1]!=0:
             b.state.vel[1]=-b.state.vel[1]
          if b.state.angVel[0]!=0:
             b.state.angVel[0]=-b.state.angVel[0]
          if b.state.angVel[1]!=0:
             b.state.angVel[1]=-b.state.angVel[1]
          if b.state.angVel[2]!=0:
             b.state.angVel[2]=-b.state.angVel[2]

    print("wheel1_ang_vel[0]:",Wheel1.state.angVel[0])
    print("wheel1_ang_vel[2]:",Wheel1.state.angVel[2])
    print("wheel1_Pos[2]:",Wheel1.state.pos[2])
    
    print("wheel2_ang_vel[0]:",Wheel2.state.angVel[0])
    print("wheel2_ang_vel[2]:",Wheel2.state.angVel[2])
    print("wheel2_Pos[2]:",Wheel2.state.pos[2])    

    print("wheel3_ang_vel[0]:",Wheel3.state.angVel[0])
    print("wheel3_ang_vel[2]:",Wheel3.state.angVel[2])
    print("wheel3_Pos[2]:",Wheel3.state.pos[2])   

    print("wheel4_ang_vel[0]:",Wheel4.state.angVel[0])
    print("wheel4_ang_vel[2]:",Wheel4.state.angVel[2])
    print("wheel4_Pos[2]:",Wheel4.state.pos[2]) 


    unb=unbalancedForce()
    print("unb_force:",unb)
    if O.iter>10000 and unb<0.02:
        print ("TORQUE AND FORCE ARE BEING APPLIED")
        O.engines=O.engines+[PyRunner(command='addPlotData()',iterPeriod=1000)]
        checker.command='SrvoPIDController()'   

# THIS FUNCTION EMPLOYES THE PRINCIPALES OF PROPORTIONAL-INTEGRAL-DERIVATIVE (PID) CONTROL LOOP (https://en.wikipedia.org/wiki/PID_controller)
# TO ADJUST THE APPLIED TORQUE AND FORCE TO A WHEEL TO SECURE TARGET LINEAR VELOCITY, ANGULAR VELOCITY, AND SLIP RATIO AND MAINTAINING THEIR 
# EQUILIBRIUM FOR THE STEADY-STATE SOLUTION.  
i=obtained_lin_vel_wheel1=obtained_lin_vel_wheel2=obtained_lin_vel_wheel3=obtained_lin_vel_wheel4=obtained_ang_vel_wheel1=obtained_ang_vel_wheel2=obtained_ang_vel_wheel3=obtained_ang_vel_wheel4=obtained_slip_ratio_wheel1=obtained_slip_ratio_wheel2=obtained_slip_ratio_wheel3=obtained_slip_ratio_wheel4=error_slip_ratio_wheel1=error_slip_ratio_wheel2=error_slip_ratio_wheel3=error_slip_ratio_wheel4=error_ang_vel_wheel1=error_ang_vel_wheel2=error_ang_vel_wheel3=error_ang_vel_wheel4=error_lin_vel_wheel1=error_lin_vel_wheel2=error_lin_vel_wheel3=error_lin_vel_wheel4=torque_control_output_ang_vel_wheel1=torque_control_output_ang_vel_wheel2=torque_control_output_ang_vel_wheel3=torque_control_output_ang_vel_wheel4=force_control_output_lin_vel_wheel1=force_control_output_lin_vel_wheel2=force_control_output_lin_vel_wheel3=force_control_output_lin_vel_wheel4=obtained_x_pos_wheel1=obtained_x_pos_wheel2=obtained_x_pos_wheel3=obtained_x_pos_wheel4=obtained_y_pos_wheel1=obtained_y_pos_wheel2=obtained_y_pos_wheel3=obtained_y_pos_wheel4=obtained_z_pos_wheel1=obtained_z_pos_wheel2=obtained_z_pos_wheel3=obtained_z_pos_wheel4=0
def SrvoPIDController():
    global prev_error_ang_vel_wheel1, iTerm_ang_vel_wheel1, prev_error_lin_vel_wheel1, iTerm_lin_vel_wheel1
    global prev_error_ang_vel_wheel2, iTerm_ang_vel_wheel2, prev_error_lin_vel_wheel2, iTerm_lin_vel_wheel2
    global prev_error_ang_vel_wheel3, iTerm_ang_vel_wheel3, prev_error_lin_vel_wheel3, iTerm_lin_vel_wheel3
    global prev_error_ang_vel_wheel4, iTerm_ang_vel_wheel4, prev_error_lin_vel_wheel4, iTerm_lin_vel_wheel4



    global obtained_lin_vel_wheel1, obtained_ang_vel_wheel1, obtained_slip_ratio_wheel1
    global obtained_lin_vel_wheel2, obtained_ang_vel_wheel2, obtained_slip_ratio_wheel2
    global obtained_lin_vel_wheel3, obtained_ang_vel_wheel3, obtained_slip_ratio_wheel3
    global obtained_lin_vel_wheel4, obtained_ang_vel_wheel4, obtained_slip_ratio_wheel4

    global error_slip_ratio_wheel1, error_ang_vel_wheel1, error_lin_vel_wheel1
    global error_slip_ratio_wheel2, error_ang_vel_wheel2, error_lin_vel_wheel2
    global error_slip_ratio_wheel3, error_ang_vel_wheel3, error_lin_vel_wheel3
    global error_slip_ratio_wheel4, error_ang_vel_wheel4, error_lin_vel_wheel4

    global torque_control_output_ang_vel_wheel1, force_control_output_lin_vel_wheel1
    global torque_control_output_ang_vel_wheel2, force_control_output_lin_vel_wheel2
    global torque_control_output_ang_vel_wheel3, force_control_output_lin_vel_wheel3
    global torque_control_output_ang_vel_wheel4, force_control_output_lin_vel_wheel4


    global obtained_x_pos_wheel1, obtained_y_pos_wheel1, obtained_z_pos_wheel1
    global obtained_x_pos_wheel2, obtained_y_pos_wheel2, obtained_z_pos_wheel2
    global obtained_x_pos_wheel3, obtained_y_pos_wheel3, obtained_z_pos_wheel3
    global obtained_x_pos_wheel4, obtained_y_pos_wheel4, obtained_z_pos_wheel4


    global i
    #Wheel1.blockedDOFs = 'yXZ' 
    Wheel1.dynamics = False
    Wheel2.dynamics = False
    Wheel3.dynamics = False
    Wheel4.dynamics = False
    for b in O.bodies:
        if isinstance(b.shape,Facet):
            b.blockedDOFs = 'xyzXYZ'
    for b in O.bodies:
        if isinstance(b.shape,Facet):
          if b.state.vel[1]!=0:
             b.state.vel[1]=-b.state.vel[1]
          if b.state.angVel[0]!=0:
             b.state.angVel[0]=-b.state.angVel[0]
          if b.state.angVel[2]!=0:
             b.state.angVel[2]=-b.state.angVel[2]
    # TURNING ON TORQUE AND FORCE ENGINES
    torqueEngine1_ang_vel.dead=False
    torqueEngine2_ang_vel.dead=False
    torqueEngine3_ang_vel.dead=False
    torqueEngine4_ang_vel.dead=False
    obtained_x_pos_wheel1 = Wheel1.state.pos[0]
    obtained_y_pos_wheel1 = Wheel1.state.pos[1]
    obtained_z_pos_wheel1 = Wheel1.state.pos[2]
    obtained_lin_vel_wheel1 = Wheel1.state.vel[0]
    obtained_ang_vel_wheel1 = Wheel1.state.angVel[1]
    obtained_slip_ratio_wheel1 = abs (((wheel_radius*obtained_ang_vel_wheel1)-obtained_lin_vel_wheel1)/(wheel_radius*obtained_ang_vel_wheel1))
    print("wheel_ang_vel[0]_wheel1:",Wheel1.state.angVel[0])
    print("wheel_ang_vel[2]_wheel1:",Wheel1.state.angVel[2])
    print("obtained_lin_vel_wheel1:",obtained_lin_vel_wheel1,"obtained_ang_vel_wheel1:",obtained_ang_vel_wheel1,"obtained_slip_ratio_wheel1:",obtained_slip_ratio_wheel1)
    print("obtained_x_pos_wheel1:",obtained_x_pos_wheel1)

    obtained_x_pos_wheel2 = Wheel2.state.pos[0]
    obtained_y_pos_wheel2 = Wheel2.state.pos[1]
    obtained_z_pos_wheel2 = Wheel2.state.pos[2]
    obtained_lin_vel_wheel2 = Wheel2.state.vel[0]
    obtained_ang_vel_wheel2 = Wheel2.state.angVel[1]
    obtained_slip_ratio_wheel2 = abs (((wheel_radius*obtained_ang_vel_wheel2)-obtained_lin_vel_wheel2)/(wheel_radius*obtained_ang_vel_wheel2))
    print("wheel_ang_vel[0]_wheel2:",Wheel2.state.angVel[0])
    print("wheel_ang_vel[2]_wheel2:",Wheel2.state.angVel[2])
    print("obtained_lin_vel_wheel2:",obtained_lin_vel_wheel2,"obtained_ang_vel_wheel2:",obtained_ang_vel_wheel2,"obtained_slip_ratio_wheel2:",obtained_slip_ratio_wheel2)
    print("obtained_x_pos_wheel2:",obtained_x_pos_wheel2)

    obtained_x_pos_wheel3 = Wheel3.state.pos[0]
    obtained_y_pos_wheel3 = Wheel3.state.pos[1]
    obtained_z_pos_wheel3 = Wheel3.state.pos[2]
    obtained_lin_vel_wheel3 = Wheel3.state.vel[0]
    obtained_ang_vel_wheel3 = Wheel3.state.angVel[1]
    obtained_slip_ratio_wheel3 = abs (((wheel_radius*obtained_ang_vel_wheel3)-obtained_lin_vel_wheel3)/(wheel_radius*obtained_ang_vel_wheel3))
    print("wheel_ang_vel[0]_wheel3:",Wheel3.state.angVel[0])
    print("wheel_ang_vel[2]_wheel3:",Wheel3.state.angVel[2])
    print("obtained_lin_vel_wheel3:",obtained_lin_vel_wheel3,"obtained_ang_vel_wheel3:",obtained_ang_vel_wheel3,"obtained_slip_ratio_wheel3:",obtained_slip_ratio_wheel3)
    print("obtained_x_pos_wheel3:",obtained_x_pos_wheel3)

    obtained_x_pos_wheel4 = Wheel4.state.pos[0]
    obtained_y_pos_wheel4 = Wheel4.state.pos[1]
    obtained_z_pos_wheel4 = Wheel4.state.pos[2]
    obtained_lin_vel_wheel4 = Wheel4.state.vel[0]
    obtained_ang_vel_wheel4 = Wheel4.state.angVel[1]
    obtained_slip_ratio_wheel4 = abs (((wheel_radius*obtained_ang_vel_wheel4)-obtained_lin_vel_wheel4)/(wheel_radius*obtained_ang_vel_wheel4))
    print("wheel_ang_vel[0]_wheel4:",Wheel4.state.angVel[0])
    print("wheel_ang_vel[2]_wheel4:",Wheel4.state.angVel[2])
    print("obtained_lin_vel_wheel4:",obtained_lin_vel_wheel4,"obtained_ang_vel_wheel4:",obtained_ang_vel_wheel4,"obtained_slip_ratio_wheel4:",obtained_slip_ratio_wheel4)
    print("obtained_x_pos_wheel4:",obtained_x_pos_wheel4)

    
    #################################################################################################
    ######### COMPUTE THE ERROR VALUE FOR SLIP RATIO, ANGULAR VELOCITY, AND LINEAR VELOCITY #########
    #################################################################################################
    # SLIP RATIO ERROR
    error_slip_ratio_wheel1 = (target_slip_ratio - obtained_slip_ratio_wheel1)
    error_slip_ratio_wheel2 = (target_slip_ratio - obtained_slip_ratio_wheel2)
    error_slip_ratio_wheel3 = (target_slip_ratio - obtained_slip_ratio_wheel3)
    error_slip_ratio_wheel4 = (target_slip_ratio - obtained_slip_ratio_wheel4)

    # ANGULAR VELOCITY ERROR
    error_ang_vel_wheel1 = (target_ang_vel - obtained_ang_vel_wheel1)
    error_ang_vel_wheel2 = (target_ang_vel - obtained_ang_vel_wheel2)
    error_ang_vel_wheel3 = (target_ang_vel - obtained_ang_vel_wheel3)
    error_ang_vel_wheel4 = (target_ang_vel - obtained_ang_vel_wheel4)

    # LINEAR VELOCITY ERROR
    error_lin_vel_wheel1 = (target_lin_vel - obtained_lin_vel_wheel1)
    error_lin_vel_wheel2 = (target_lin_vel - obtained_lin_vel_wheel2)
    error_lin_vel_wheel3 = (target_lin_vel - obtained_lin_vel_wheel3)
    error_lin_vel_wheel4 = (target_lin_vel - obtained_lin_vel_wheel4)
    print("error_slip_ratio_wheel1:",error_slip_ratio_wheel1,"error_ang_vel_wheel1:",error_ang_vel_wheel1, "error_lin_vel_wheel1:",error_lin_vel_wheel1)
    print("error_slip_ratio_wheel2:",error_slip_ratio_wheel2,"error_ang_vel_wheel2:",error_ang_vel_wheel2, "error_lin_vel_wheel2:",error_lin_vel_wheel2)
    print("error_slip_ratio_wheel3:",error_slip_ratio_wheel3,"error_ang_vel_wheel3:",error_ang_vel_wheel3, "error_lin_vel_wheel3:",error_lin_vel_wheel3)
    print("error_slip_ratio_wheel4:",error_slip_ratio_wheel4,"error_ang_vel_wheel4:",error_ang_vel_wheel4, "error_lin_vel_wheel4:",error_lin_vel_wheel4)

    ############################################################
    ######### COMPUTE THE PID CONTROL OUTPUT VARIABLES #########
    #########    AND UPDATE TORQUE AND FORCE VALUES    #########
    ############################################################
    # ANGULAR VELOCITY PID--WHEEL1
    pTerm_ang_vel_wheel1 =  (kP_ang_vel_wheel1*(error_ang_vel_wheel1))
    iTerm_ang_vel_wheel1 += (kI_ang_vel_wheel1*(error_ang_vel_wheel1))
    dTerm_ang_vel_wheel1 =  (kD_ang_vel_wheel1*(error_ang_vel_wheel1-prev_error_ang_vel_wheel1))
    torque_control_output_ang_vel_wheel1 = pTerm_ang_vel_wheel1 + iTerm_ang_vel_wheel1 + dTerm_ang_vel_wheel1
    if torque_control_output_ang_vel_wheel1>T_angVel_max:
        torque_control_output_ang_vel_wheel1*=T_angVel_max/abs(torque_control_output_ang_vel_wheel1)
    torqueEngine1_ang_vel.moment=Vector3(0,torque_control_output_ang_vel_wheel1,0)
    print("torque_control_output_ang_vel_wheel1:",torque_control_output_ang_vel_wheel1)
    prev_error_ang_vel_wheel1 = error_ang_vel_wheel1
    if abs(obtained_lin_vel_wheel1)>target_lin_vel and i==0:
        i=1
        forceEngine1_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel1 =  (kP_lin_vel_wheel1*(error_lin_vel_wheel1))
        iTerm_lin_vel_wheel1 += (kI_lin_vel_wheel1*(error_lin_vel_wheel1))
        dTerm_lin_vel_wheel1 =  (kD_lin_vel_wheel1*(error_lin_vel_wheel1-prev_error_lin_vel_wheel1))
        force_control_output_lin_vel_wheel1 = pTerm_lin_vel_wheel1 + iTerm_lin_vel_wheel1 + dTerm_lin_vel_wheel1
        if force_control_output_lin_vel_wheel1>F_max:
            force_control_output_lin_vel_wheel1*=F_max/abs(force_control_output_lin_vel_wheel1)
        forceEngine1_lin_vel.force_control_output_lin_vel_wheel1=Vector3(force_control_output_lin_vel_wheel1,0,0)
        print("force_control_output_lin_vel_wheel1:",force_control_output_lin_vel_wheel1)
        prev_error_lin_vel_wheel1 = error_lin_vel_wheel1
    elif i==1:
        forceEngine1_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel1 =  (kP_lin_vel_wheel1*(error_lin_vel_wheel1))
        iTerm_lin_vel_wheel1 += (kI_lin_vel_wheel1*(error_lin_vel_wheel1))
        dTerm_lin_vel_wheel1 =  (kD_lin_vel_wheel1*(error_lin_vel_wheel1-prev_error_lin_vel_wheel1))
        force_control_output_lin_vel_wheel1 = pTerm_lin_vel_wheel1 + iTerm_lin_vel_wheel1 + dTerm_lin_vel_wheel1  
        forceEngine1_lin_vel.force=Vector3(force_control_output_lin_vel_wheel1,0,0)
        print("force_control_output_lin_vel_wheel1:",force_control_output_lin_vel_wheel1)
        prev_error_lin_vel_wheel1 = error_lin_vel_wheel1

    # ANGULAR VELOCITY PID--WHEEL2
    pTerm_ang_vel_wheel2 =  (kP_ang_vel_wheel2*(error_ang_vel_wheel2))
    iTerm_ang_vel_wheel2 += (kI_ang_vel_wheel2*(error_ang_vel_wheel2))
    dTerm_ang_vel_wheel2 =  (kD_ang_vel_wheel2*(error_ang_vel_wheel2-prev_error_ang_vel_wheel2))
    torque_control_output_ang_vel_wheel2 = pTerm_ang_vel_wheel2 + iTerm_ang_vel_wheel2 + dTerm_ang_vel_wheel2
    if torque_control_output_ang_vel_wheel2>T_angVel_max:
        torque_control_output_ang_vel_wheel2*=T_angVel_max/abs(torque_control_output_ang_vel_wheel2)
    torqueEngine2_ang_vel.moment=Vector3(0,torque_control_output_ang_vel_wheel2,0)
    print("torque_control_output_ang_vel_wheel2:",torque_control_output_ang_vel_wheel2)
    prev_error_ang_vel_wheel2 = error_ang_vel_wheel2
    if abs(obtained_lin_vel_wheel2)>target_lin_vel and i==0:
        i=1
        forceEngine2_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel2 =  (kP_lin_vel_wheel2*(error_lin_vel_wheel2))
        iTerm_lin_vel_wheel2 += (kI_lin_vel_wheel2*(error_lin_vel_wheel2))
        dTerm_lin_vel_wheel2 =  (kD_lin_vel_wheel2*(error_lin_vel_wheel2-prev_error_lin_vel_wheel2))
        force_control_output_lin_vel_wheel2 = pTerm_lin_vel_wheel2 + iTerm_lin_vel_wheel2 + dTerm_lin_vel_wheel2
        if force_control_output_lin_vel_wheel2>F_max:
            force_control_output_lin_vel_wheel2*=F_max/abs(force_control_output_lin_vel_wheel2)
        forceEngine2_lin_vel.force_control_output_lin_vel_wheel2=Vector3(force_control_output_lin_vel_wheel2,0,0)
        print("force_control_output_lin_vel_wheel2:",force_control_output_lin_vel_wheel2)
        prev_error_lin_vel_wheel2 = error_lin_vel_wheel2
    elif i==1:
        forceEngine2_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel2 =  (kP_lin_vel_wheel2*(error_lin_vel_wheel2))
        iTerm_lin_vel_wheel2 += (kI_lin_vel_wheel2*(error_lin_vel_wheel2))
        dTerm_lin_vel_wheel2 =  (kD_lin_vel_wheel2*(error_lin_vel_wheel2-prev_error_lin_vel_wheel2))
        force_control_output_lin_vel_wheel2 = pTerm_lin_vel_wheel2 + iTerm_lin_vel_wheel2 + dTerm_lin_vel_wheel2  
        forceEngine2_lin_vel.force=Vector3(force_control_output_lin_vel_wheel2,0,0)
        print("force_control_output_lin_vel_wheel2:",force_control_output_lin_vel_wheel2)
        prev_error_lin_vel_wheel2 = error_lin_vel_wheel2

    # ANGULAR VELOCITY PID--WHEEL3
    pTerm_ang_vel_wheel3 =  (kP_ang_vel_wheel3*(error_ang_vel_wheel3))
    iTerm_ang_vel_wheel3 += (kI_ang_vel_wheel3*(error_ang_vel_wheel3))
    dTerm_ang_vel_wheel3 =  (kD_ang_vel_wheel3*(error_ang_vel_wheel3-prev_error_ang_vel_wheel3))
    torque_control_output_ang_vel_wheel3 = pTerm_ang_vel_wheel3 + iTerm_ang_vel_wheel3 + dTerm_ang_vel_wheel3
    if torque_control_output_ang_vel_wheel3>T_angVel_max:
        torque_control_output_ang_vel_wheel3*=T_angVel_max/abs(torque_control_output_ang_vel_wheel3)
    torqueEngine3_ang_vel.moment=Vector3(0,torque_control_output_ang_vel_wheel3,0)
    print("torque_control_output_ang_vel_wheel3:",torque_control_output_ang_vel_wheel3)
    prev_error_ang_vel_wheel3 = error_ang_vel_wheel3
    if abs(obtained_lin_vel_wheel3)>target_lin_vel and i==0:
        i=1
        forceEngine3_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel3 =  (kP_lin_vel_wheel3*(error_lin_vel_wheel3))
        iTerm_lin_vel_wheel3 += (kI_lin_vel_wheel3*(error_lin_vel_wheel3))
        dTerm_lin_vel_wheel3 =  (kD_lin_vel_wheel3*(error_lin_vel_wheel3-prev_error_lin_vel_wheel3))
        force_control_output_lin_vel_wheel3 = pTerm_lin_vel_wheel3 + iTerm_lin_vel_wheel3 + dTerm_lin_vel_wheel3
        if force_control_output_lin_vel_wheel3>F_max:
            force_control_output_lin_vel_wheel3*=F_max/abs(force_control_output_lin_vel_wheel3)
        forceEngine3_lin_vel.force_control_output_lin_vel_wheel3=Vector3(force_control_output_lin_vel_wheel3,0,0)
        print("force_control_output_lin_vel_wheel3:",force_control_output_lin_vel_wheel3)
        prev_error_lin_vel_wheel3 = error_lin_vel_wheel3
    elif i==1:
        forceEngine3_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel3 =  (kP_lin_vel_wheel3*(error_lin_vel_wheel3))
        iTerm_lin_vel_wheel3 += (kI_lin_vel_wheel3*(error_lin_vel_wheel3))
        dTerm_lin_vel_wheel3 =  (kD_lin_vel_wheel3*(error_lin_vel_wheel3-prev_error_lin_vel_wheel3))
        force_control_output_lin_vel_wheel3 = pTerm_lin_vel_wheel3 + iTerm_lin_vel_wheel3 + dTerm_lin_vel_wheel3  
        forceEngine3_lin_vel.force=Vector3(force_control_output_lin_vel_wheel3,0,0)
        print("force_control_output_lin_vel_wheel3:",force_control_output_lin_vel_wheel3)
        prev_error_lin_vel_wheel3 = error_lin_vel_wheel3

    # ANGULAR VELOCITY PID--WHEEL4
    pTerm_ang_vel_wheel4 =  (kP_ang_vel_wheel4*(error_ang_vel_wheel4))
    iTerm_ang_vel_wheel4 += (kI_ang_vel_wheel4*(error_ang_vel_wheel4))
    dTerm_ang_vel_wheel4 =  (kD_ang_vel_wheel4*(error_ang_vel_wheel4-prev_error_ang_vel_wheel4))
    torque_control_output_ang_vel_wheel4 = pTerm_ang_vel_wheel4 + iTerm_ang_vel_wheel4 + dTerm_ang_vel_wheel4
    if torque_control_output_ang_vel_wheel4>T_angVel_max:
        torque_control_output_ang_vel_wheel4*=T_angVel_max/abs(torque_control_output_ang_vel_wheel4)
    torqueEngine4_ang_vel.moment=Vector3(0,torque_control_output_ang_vel_wheel4,0)
    print("torque_control_output_ang_vel_wheel4:",torque_control_output_ang_vel_wheel4)
    prev_error_ang_vel_wheel4 = error_ang_vel_wheel4
    if abs(obtained_lin_vel_wheel4)>target_lin_vel and i==0:
        i=1
        forceEngine4_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel4 =  (kP_lin_vel_wheel4*(error_lin_vel_wheel4))
        iTerm_lin_vel_wheel4 += (kI_lin_vel_wheel4*(error_lin_vel_wheel4))
        dTerm_lin_vel_wheel4 =  (kD_lin_vel_wheel4*(error_lin_vel_wheel4-prev_error_lin_vel_wheel4))
        force_control_output_lin_vel_wheel4 = pTerm_lin_vel_wheel4 + iTerm_lin_vel_wheel4 + dTerm_lin_vel_wheel4
        if force_control_output_lin_vel_wheel4>F_max:
            force_control_output_lin_vel_wheel4*=F_max/abs(force_control_output_lin_vel_wheel4)
        forceEngine4_lin_vel.force_control_output_lin_vel_wheel4=Vector3(force_control_output_lin_vel_wheel4,0,0)
        print("force_control_output_lin_vel_wheel4:",force_control_output_lin_vel_wheel4)
        prev_error_lin_vel_wheel4 = error_lin_vel_wheel4
    elif i==1:
        forceEngine4_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel_wheel4 =  (kP_lin_vel_wheel4*(error_lin_vel_wheel4))
        iTerm_lin_vel_wheel4 += (kI_lin_vel_wheel4*(error_lin_vel_wheel4))
        dTerm_lin_vel_wheel4 =  (kD_lin_vel_wheel4*(error_lin_vel_wheel4-prev_error_lin_vel_wheel4))
        force_control_output_lin_vel_wheel4 = pTerm_lin_vel_wheel4 + iTerm_lin_vel_wheel4 + dTerm_lin_vel_wheel4  
        forceEngine4_lin_vel.force=Vector3(force_control_output_lin_vel_wheel4,0,0)
        print("force_control_output_lin_vel_wheel4:",force_control_output_lin_vel_wheel4)
        prev_error_lin_vel_wheel4 = error_lin_vel_wheel4



def addPlotData():
    wheel_target_lin_vel = target_lin_vel 
    wheel_target_ang_vel = target_ang_vel
    wheel_target_slip_ratio = target_slip_ratio
    wheel_obtained_lin_vel_wheel1 = obtained_lin_vel_wheel1
    wheel_obtained_lin_vel_wheel2 = obtained_lin_vel_wheel2
    wheel_obtained_lin_vel_wheel3 = obtained_lin_vel_wheel3
    wheel_obtained_lin_vel_wheel4 = obtained_lin_vel_wheel4

    wheel_obtained_ang_vel_wheel1 = obtained_ang_vel_wheel1
    wheel_obtained_ang_vel_wheel2 = obtained_ang_vel_wheel2
    wheel_obtained_ang_vel_wheel3 = obtained_ang_vel_wheel3
    wheel_obtained_ang_vel_wheel4 = obtained_ang_vel_wheel4

    wheel_obtained_slip_ratio_wheel1 = obtained_slip_ratio_wheel1
    wheel_obtained_slip_ratio_wheel2 = obtained_slip_ratio_wheel2
    wheel_obtained_slip_ratio_wheel3 = obtained_slip_ratio_wheel3
    wheel_obtained_slip_ratio_wheel4 = obtained_slip_ratio_wheel4

    wheel_x_disp_wheel1 = obtained_x_pos_wheel1
    wheel_x_disp_wheel2 = obtained_x_pos_wheel2
    wheel_x_disp_wheel3 = obtained_x_pos_wheel3
    wheel_x_disp_wheel4 = obtained_x_pos_wheel4

    wheel_y_disp_wheel1 = obtained_y_pos_wheel1
    wheel_y_disp_wheel2 = obtained_y_pos_wheel2
    wheel_y_disp_wheel3 = obtained_y_pos_wheel3
    wheel_y_disp_wheel4 = obtained_y_pos_wheel4

    wheel_z_disp_wheel1 = obtained_z_pos_wheel1 
    wheel_z_disp_wheel2 = obtained_z_pos_wheel2 
    wheel_z_disp_wheel3 = obtained_z_pos_wheel3 
    wheel_z_disp_wheel4 = obtained_z_pos_wheel4 

    wheel_net_torque_wheel1 = torque_control_output_ang_vel_wheel1
    wheel_net_torque_wheel2 = torque_control_output_ang_vel_wheel2
    wheel_net_torque_wheel3 = torque_control_output_ang_vel_wheel3
    wheel_net_torque_wheel4 = torque_control_output_ang_vel_wheel4

    wheel_drawbar_pull_wheel1 = force_control_output_lin_vel_wheel1 
    wheel_drawbar_pull_wheel2 = force_control_output_lin_vel_wheel2 
    wheel_drawbar_pull_wheel3 = force_control_output_lin_vel_wheel3 
    wheel_drawbar_pull_wheel4 = force_control_output_lin_vel_wheel4 

    lin_vel_error_wheel1 = error_lin_vel_wheel1
    lin_vel_error_wheel2 = error_lin_vel_wheel2
    lin_vel_error_wheel3 = error_lin_vel_wheel3
    lin_vel_error_wheel4 = error_lin_vel_wheel4

    ang_vel_error_wheel1 = error_ang_vel_wheel1
    ang_vel_error_wheel2 = error_ang_vel_wheel2
    ang_vel_error_wheel3 = error_ang_vel_wheel3
    ang_vel_error_wheel4 = error_ang_vel_wheel4

    slip_ratio_error_wheel1 = error_slip_ratio_wheel1
    slip_ratio_error_wheel2 = error_slip_ratio_wheel2
    slip_ratio_error_wheel3 = error_slip_ratio_wheel3
    slip_ratio_error_wheel4 = error_slip_ratio_wheel4

    yade.plot.addData({'i':O.iter,'wheel_target_lin_vel':wheel_target_lin_vel,'wheel_target_ang_vel':wheel_target_ang_vel,'wheel_target_slip_ratio':wheel_target_slip_ratio,\
        'wheel_x_disp_wheel1':wheel_x_disp_wheel1,'wheel_y_disp_wheel1':wheel_y_disp_wheel1,'wheel_z_disp_wheel1':wheel_z_disp_wheel1,\
        'wheel_x_disp_wheel2':wheel_x_disp_wheel2,'wheel_y_disp_wheel1':wheel_y_disp_wheel2,'wheel_z_disp_wheel1':wheel_z_disp_wheel2,\
        'wheel_x_disp_wheel3':wheel_x_disp_wheel3,'wheel_y_disp_wheel3':wheel_y_disp_wheel3,'wheel_z_disp_wheel3':wheel_z_disp_wheel3,\
        'wheel_x_disp_wheel4':wheel_x_disp_wheel4,'wheel_y_disp_wheel4':wheel_y_disp_wheel4,'wheel_z_disp_wheel4':wheel_z_disp_wheel4,\
        'wheel_obtained_lin_vel_wheel1':wheel_obtained_lin_vel_wheel1,'wheel_obtained_ang_vel_wheel1':wheel_obtained_ang_vel_wheel1,'wheel_obtained_slip_ratio_wheel1':wheel_obtained_slip_ratio_wheel1,\
        'wheel_obtained_lin_vel_wheel2':wheel_obtained_lin_vel_wheel2,'wheel_obtained_ang_vel_wheel2':wheel_obtained_ang_vel_wheel2,'wheel_obtained_slip_ratio_wheel2':wheel_obtained_slip_ratio_wheel2,\
        'wheel_obtained_lin_vel_wheel3':wheel_obtained_lin_vel_wheel3,'wheel_obtained_ang_vel_wheel3':wheel_obtained_ang_vel_wheel3,'wheel_obtained_slip_ratio_wheel3':wheel_obtained_slip_ratio_wheel3,\
        'wheel_obtained_lin_vel_wheel4':wheel_obtained_lin_vel_wheel4,'wheel_obtained_ang_vel_wheel4':wheel_obtained_ang_vel_wheel4,'wheel_obtained_slip_ratio_wheel4':wheel_obtained_slip_ratio_wheel4,\
        'wheel_net_torque_wheel1':wheel_net_torque_wheel1,'wheel_drawbar_pull_wheel1':wheel_drawbar_pull_wheel1,\
        'wheel_net_torque_wheel2':wheel_net_torque_wheel2,'wheel_drawbar_pull_wheel2':wheel_drawbar_pull_wheel2,\
        'wheel_net_torque_wheel3':wheel_net_torque_wheel3,'wheel_drawbar_pull_wheel3':wheel_drawbar_pull_wheel3,\
        'wheel_net_torque_wheel4':wheel_net_torque_wheel4,'wheel_drawbar_pull_wheel4':wheel_drawbar_pull_wheel4,\
        'lin_vel_error_wheel1':lin_vel_error_wheel1,'ang_vel_error_wheel1':ang_vel_error_wheel1, 'slip_ratio_error_wheel1':slip_ratio_error_wheel1,\
        'lin_vel_error_wheel2':lin_vel_error_wheel2,'ang_vel_error_wheel2':ang_vel_error_wheel2, 'slip_ratio_error_wheel2':slip_ratio_error_wheel2,\
        'lin_vel_error_wheel3':lin_vel_error_wheel3,'ang_vel_error_wheel3':ang_vel_error_wheel3, 'slip_ratio_error_wheel3':slip_ratio_error_wheel3,\
        'lin_vel_error_wheel4':lin_vel_error_wheel4,'ang_vel_error_wheel4':ang_vel_error_wheel4, 'slip_ratio_error_wheel4':slip_ratio_error_wheel4})
    plot.saveDataTxt("/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/SWI_ServoPID_facetWheel_sphericalGround_4Wheels_2Terms.txt")
plot.plots = {'wheel_x_disp_wheel1': ('wheel_obtained_lin_vel_wheel1', 'wheel_target_lin_vel'), 'wheel_x_disp_wheel2': ('wheel_obtained_lin_vel_wheel2', 'wheel_target_lin_vel'), 'wheel_x_disp_wheel3': ('wheel_obtained_lin_vel_wheel3', 'wheel_target_lin_vel'), 'wheel_x_disp_wheel4': ('wheel_obtained_lin_vel_wheel4', 'wheel_target_lin_vel'),'wheel_x_disp_wheel1 ': ('wheel_obtained_ang_vel_wheel1', 'wheel_target_ang_vel'),'wheel_x_disp_wheel2 ': ('wheel_obtained_ang_vel_wheel2', 'wheel_target_ang_vel'), 'wheel_x_disp_wheel3 ': ('wheel_obtained_ang_vel_wheel3', 'wheel_target_ang_vel'), 'wheel_x_disp_wheel4 ': ('wheel_obtained_ang_vel_wheel4', 'wheel_target_ang_vel'), 'wheel_x_disp_wheel1  ': ('wheel_obtained_slip_ratio_wheel1', 'wheel_target_slip_ratio'), 'wheel_x_disp_wheel2  ': ('wheel_obtained_slip_ratio_wheel2', 'wheel_target_slip_ratio'), 'wheel_x_disp_wheel3  ': ('wheel_obtained_slip_ratio_wheel3', 'wheel_target_slip_ratio'), 'wheel_x_disp_wheel4  ': ('wheel_obtained_slip_ratio_wheel4', 'wheel_target_slip_ratio'), 'wheel_x_disp_wheel1   ': ('wheel_net_torque_wheel1', 'wheel_drawbar_pull_wheel1'), 'wheel_x_disp_wheel2   ': ('wheel_net_torque_wheel2', 'wheel_drawbar_pull_wheel2'), 'wheel_x_disp_wheel3   ': ('wheel_net_torque_wheel3', 'wheel_drawbar_pull_wheel3'), 'wheel_x_disp_wheel4   ': ('wheel_net_torque_wheel4', 'wheel_drawbar_pull_wheel4'), 'wheel_x_disp_wheel1    ': ('lin_vel_error_wheel1', 'ang_vel_error_wheel1', 'slip_ratio_error_wheel1'), 'wheel_x_disp_wheel2    ': ('lin_vel_error_wheel2', 'ang_vel_error_wheel2', 'slip_ratio_error_wheel2'), 'wheel_x_disp_wheel3    ': ('lin_vel_error_wheel3', 'ang_vel_error_wheel3', 'slip_ratio_error_wheel3'), 'wheel_x_disp_wheel4    ': ('lin_vel_error_wheel4', 'ang_vel_error_wheel4', 'slip_ratio_error_wheel4')}
plot.plot()


