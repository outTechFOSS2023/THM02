# -*- coding: utf-8 -*-
#*********************************************************************************************************
#*********************************************************************************************************
#Copyright 2023 Blueshift, LLC
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, #including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to #do so, subject to the following conditions:
         #The Software is subject to all use, distribution, modification, sales, and other restrictions applicable to the software-as-a-service product specified in the Agreement.
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND #NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR #IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
facetWheel=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/Regular Mesh.stl',wire=False,color=Vector3(1,1,1),shift=Vector3(0,0.37382716/4,0),scale=1)

#facetWheel=ymport.gmsh(meshfile='/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelFinal.mesh',shift=Vector3(0,0.37382716/4,0),scale=1.0)
#facetWheel = ymport.stl('/home/ngoudarzi/Desktop/servoPIDcontroller/finalServoPIDController_facetWheel_sphericalGround/scaledWheelRefined.stl',scale=1,shift=Vector3(0,0,-0.001),wire=True,color=Vector3(1.0,1.0,1.0))
num_facets = len(facetWheel)
print("num_facets:",num_facets) 
## A CONSTANT NORMAL FORCE OFV 80 N IS ASSUMED
g = 9.81 # m/s
wheel_volume    = 0.0061096032  #m^3. FROM CAD
wheel_density   = 7850.00000000 #kg/m^3. 
wheel_radius    = 0.15000000000 #m. FROM CAD 
wheel_width     = 0.10000000000 #m. FROM CAD
wheel_self_mass = wheel_volume*wheel_density
vehicle_mass    = 430           # KG
vehicle_mass_single_wheel = vehicle_mass/4 
total_mass_single_wheel = wheel_self_mass+vehicle_mass_single_wheel
wheel_updated_density = vehicle_mass_single_wheel*wheel_density/wheel_self_mass # THE EFFECT OF ADDITIONAL NORMAL FORCE FOR MOMENT OF INERTIA
print("wheel_updated_density:",wheel_updated_density) 

#VOLUME-MOMENTS FROM CAD
Vx= 3.50812973e-05
Vy= 5.99799225e-05
Vz= 3.50812973e-05
Ix= Vx*wheel_updated_density
Iy= Vy*wheel_updated_density
Iz= Vz*wheel_updated_density

for i in facetWheel:
  i.state.mass = total_mass_single_wheel/num_facets
  i.state.inertia = (1,1,1) # THIS IS TEMPORARY
facetWheelID = O.bodies.appendClumped(facetWheel)
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
########################################################
# MODELING GROUND AS A PRECOMPACTED GRANULAR ASSEMBLY ##
########################################################
O.bodies.append(ymport.text('/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/Deposited_Bed.txt',shift=Vector3(0,0,0),material=Regolith))
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

mn,mx=Vector3(minX_bed,minY_bed,minZ_bed),Vector3(maxX_bed,maxY_bed,10*maxZ_bed)
walls=aabbWalls([mn,mx],thickness=0)
wallIds=O.bodies.append(walls)

###############################
## DEFINE ENGINES PARAMETERS ##
###############################
T_angVel_max = 300 # MAY NOT BE REQUIRED
T_slipRatio_max = 100 # MAY NOT BE REQUIRED
F_max = 300 # MAY NOT BE REQUIRED
T_start = 0.1
F_start = 0.1

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
# SLIP RATIO
kP_slip_ratio = 20
kI_slip_ratio = 0.1
kD_slip_ratio = 0.01

# LINEAR VELOCITY
kP_lin_vel = 0.1
kI_lin_vel = 0.05
kD_lin_vel = 0.01

# ANGULAR VELOCITY
kP_ang_vel = 100
kI_ang_vel = 5
kD_ang_vel = 0.5

###########################
## DEFINE INITIAL ERRORS ##
###########################
# SLIP RATIO
prev_error_slip_ratio = 0
iTerm_slip_ratio = 0

# LINEAR VELOCITY
prev_error_lin_vel = 0
iTerm_lin_vel = 0

# ANGULAR VELOCITY
prev_error_ang_vel = 0
iTerm_ang_vel = 0

#########################################
## DEFINE SIMULATION NEWTON INTEGRATOR ##
#########################################
newton=NewtonIntegrator(damping=0.8,gravity=(0,0,-g))

#############
## ENGINES ##
#############
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/VTK_SWI_ServoPID__facetWheel_sphericalGround_3Terms/SWI_ServoPID__facetWheel_sphericalGround_3Terms' ,recorders=['all'],iterPeriod=20000),
  ForceResetter(),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine_slip_ratio",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine_ang_vel",moment=Vector3(0,T_start,0)),
  ForceEngine(dead=True,ids=[Wheel1.id],label="forceEngine_lin_vel",force=Vector3(F_start,0,0)),
  InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Box_Aabb(),Bo1_Facet_Aabb()]),
  InteractionLoop(
    [Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
    [Ip2_FrictMat_FrictMat_FrictPhys(label='ContactModel')],
    [Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  newton,
  PyRunner(command='wheelSettlement()',iterPeriod=10,label='checker'),
]
O.dt=D_t


#ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
def wheelSettlement():
    #Wheel1.blockedDOFs = 'xyXYZ' ## NOTE: ONLY MOVEMENT ALONG Z AXIS IS ALLOWED. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    Wheel1.dynamics = False
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

    print("wheel_ang_vel[0]:",Wheel1.state.angVel[0])
    print("wheel_ang_vel[2]:",Wheel1.state.angVel[2])
    print("wheelPos[2]:",Wheel1.state.pos[2])
    unb=unbalancedForce()
    print("unb_force:",unb)
    if O.iter>10000 and unb<0.02:
        print ("TORQUE AND FORCE ARE BEING APPLIED")
        O.engines=O.engines+[PyRunner(command='addPlotData()',iterPeriod=1000)]
        checker.command='SrvoPIDController()'   

# THIS FUNCTION EMPLOYES THE PRINCIPALES OF PROPORTIONAL-INTEGRAL-DERIVATIVE (PID) CONTROL LOOP (https://en.wikipedia.org/wiki/PID_controller)
# TO ADJUST THE APPLIED TORQUE AND FORCE TO A WHEEL TO SECURE TARGET LINEAR VELOCITY, ANGULAR VELOCITY, AND SLIP RATIO AND MAINTAINING THEIR 
# EQUILIBRIUM FOR THE STEADY-STATE SOLUTION.  
i=obtained_lin_vel=obtained_ang_vel=obtained_slip_ratio=error_slip_ratio=error_ang_vel=error_lin_vel=torque_control_output_slip_ratio=torque_control_output_ang_vel=force_control_output_lin_vel=obtained_x_pos=obtained_y_pos=obtained_z_pos=0
def SrvoPIDController():
    global prev_error_slip_ratio, iTerm_slip_ratio, prev_error_ang_vel, iTerm_ang_vel, prev_error_lin_vel, iTerm_lin_vel
    global obtained_lin_vel, obtained_ang_vel, obtained_slip_ratio
    global error_slip_ratio, error_ang_vel, error_lin_vel
    global torque_control_output_slip_ratio, torque_control_output_ang_vel, force_control_output_lin_vel
    global obtained_x_pos, obtained_y_pos, obtained_z_pos
    global i
    #Wheel1.blockedDOFs = 'yXZ' 
    Wheel1.dynamics = False
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
    torqueEngine_slip_ratio.dead=False
    torqueEngine_ang_vel.dead=False
    #forceEngine_lin_vel.dead=False
    obtained_x_pos = Wheel1.state.pos[0]
    obtained_y_pos = Wheel1.state.pos[1]
    obtained_z_pos = Wheel1.state.pos[2]
    obtained_lin_vel = Wheel1.state.vel[0]
    obtained_ang_vel = Wheel1.state.angVel[1]
    obtained_slip_ratio = abs (((wheel_radius*obtained_ang_vel)-obtained_lin_vel)/(wheel_radius*obtained_ang_vel))
    print("wheel_ang_vel[0]:",Wheel1.state.angVel[0])
    print("wheel_ang_vel[2]:",Wheel1.state.angVel[2])
    print("obtained_lin_vel:",obtained_lin_vel,"obtained_ang_vel:",obtained_ang_vel,"obtained_slip_ratio:",obtained_slip_ratio)
    print("obtained_x_pos:",obtained_x_pos)
    
    #################################################################################################
    ######### COMPUTE THE ERROR VALUE FOR SLIP RATIO, ANGULAR VELOCITY, AND LINEAR VELOCITY #########
    #################################################################################################
    # SLIP RATIO ERROR
    error_slip_ratio = (target_slip_ratio - obtained_slip_ratio)
    
    # ANGULAR VELOCITY ERROR
    error_ang_vel = (target_ang_vel - obtained_ang_vel)

    # LINEAR VELOCITY ERROR
    error_lin_vel = (target_lin_vel - obtained_lin_vel)
    print("error_slip_ratio:",error_slip_ratio,"error_ang_vel:",error_ang_vel, "error_lin_vel:",error_lin_vel)

    ############################################################
    ######### COMPUTE THE PID CONTROL OUTPUT VARIABLES #########
    #########    AND UPDATE TORQUE AND FORCE VALUES    #########
    ############################################################
    # SLIP RATIO PID
    pTerm_slip_ratio =  (kP_slip_ratio*(error_slip_ratio))
    iTerm_slip_ratio += (kI_slip_ratio*(error_slip_ratio))
    dTerm_slip_ratio =  (kD_slip_ratio*(error_slip_ratio-prev_error_slip_ratio))
    torque_control_output_slip_ratio = pTerm_slip_ratio + iTerm_slip_ratio + dTerm_slip_ratio
    if torque_control_output_slip_ratio>T_slipRatio_max:
        torque_control_output_slip_ratio*=T_slipRatio_max/abs(torque_control_output_slip_ratio)
    torqueEngine_slip_ratio.moment=Vector3(0,torque_control_output_slip_ratio,0)
    print("torque_control_output_slip_ratio:",torque_control_output_slip_ratio)
    prev_error_slip_ratio = error_slip_ratio

    # ANGULAR VELOCITY PID
    pTerm_ang_vel =  (kP_ang_vel*(error_ang_vel))
    iTerm_ang_vel += (kI_ang_vel*(error_ang_vel))
    dTerm_ang_vel =  (kD_ang_vel*(error_ang_vel-prev_error_ang_vel))
    torque_control_output_ang_vel = pTerm_ang_vel + iTerm_ang_vel + dTerm_ang_vel
    if torque_control_output_ang_vel>T_angVel_max:
        torque_control_output_ang_vel*=T_angVel_max/abs(torque_control_output_ang_vel)
    torqueEngine_ang_vel.moment=Vector3(0,torque_control_output_ang_vel,0)
    print("torque_control_output_ang_vel:",torque_control_output_ang_vel)
    prev_error_ang_vel = error_ang_vel
    if error_lin_vel<0 and abs(error_ang_vel)<0.005 and i==0:
        i=1
        forceEngine_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel =  (kP_lin_vel*(error_lin_vel))
        iTerm_lin_vel += (kI_lin_vel*(error_lin_vel))
        dTerm_lin_vel =  (kD_lin_vel*(error_lin_vel-prev_error_lin_vel))
        force_control_output_lin_vel = pTerm_lin_vel + iTerm_lin_vel + dTerm_ang_vel
        if force_control_output_lin_vel>F_max:
            force_control_output_lin_vel*=F_max/abs(force_control_output_lin_vel)
        forceEngine_lin_vel.force=Vector3(force_control_output_lin_vel,0,0)
        print("force_control_output_lin_vel:",force_control_output_lin_vel)
        prev_error_lin_vel = error_lin_vel
    elif i==1:
        forceEngine_lin_vel.dead=False
        # LINEAR VELOCITY PID
        pTerm_lin_vel =  (kP_lin_vel*(error_lin_vel))
        iTerm_lin_vel += (kI_lin_vel*(error_lin_vel))
        dTerm_lin_vel =  (kD_lin_vel*(error_lin_vel-prev_error_lin_vel))
        force_control_output_lin_vel = pTerm_lin_vel + iTerm_lin_vel + dTerm_ang_vel  
        forceEngine_lin_vel.force=Vector3(force_control_output_lin_vel,0,0)
        print("force_control_output_lin_vel:",force_control_output_lin_vel)
        prev_error_lin_vel = error_lin_vel
def addPlotData():
    wheel_target_lin_vel = target_lin_vel 
    wheel_target_ang_vel = target_ang_vel
    wheel_target_slip_ratio = target_slip_ratio
    wheel_obtained_lin_vel = obtained_lin_vel 
    wheel_obtained_ang_vel = obtained_ang_vel
    wheel_obtained_slip_ratio = obtained_slip_ratio
    wheel_x_disp = obtained_x_pos
    wheel_y_disp = obtained_y_pos
    wheel_z_disp = obtained_y_pos 
    wheel_net_torque = torque_control_output_slip_ratio+torque_control_output_ang_vel
    wheel_drawbar_pull = force_control_output_lin_vel 
    lin_vel_error = error_lin_vel
    ang_vel_error = error_ang_vel
    slip_ratio_error = error_slip_ratio
    yade.plot.addData({'i':O.iter,'wheel_target_lin_vel':wheel_target_lin_vel,'wheel_target_ang_vel':wheel_target_ang_vel,'wheel_target_slip_ratio':wheel_target_slip_ratio,\
        'wheel_x_disp':wheel_x_disp,'wheel_y_disp':wheel_y_disp,'wheel_z_disp':wheel_z_disp,\
        'wheel_obtained_lin_vel':wheel_obtained_lin_vel,'wheel_obtained_ang_vel':wheel_obtained_ang_vel,'wheel_obtained_slip_ratio':wheel_obtained_slip_ratio,\
        'wheel_net_torque':wheel_net_torque,'wheel_drawbar_pull':wheel_drawbar_pull,\
        'lin_vel_error':lin_vel_error,'ang_vel_error':ang_vel_error, 'slip_ratio_error':slip_ratio_error})
    plot.saveDataTxt("/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_sphericalGround/Output_SWI_ServoPID__facetWheel_sphericalGround_3Terms/SWI_ServoPID__facetWheel_sphericalGround_3Terms")
plot.plots = {'wheel_x_disp': ('wheel_obtained_lin_vel', 'wheel_target_lin_vel'), 'wheel_x_disp ': ('wheel_obtained_ang_vel', 'wheel_target_ang_vel'), 'wheel_x_disp  ': ('wheel_obtained_slip_ratio', 'wheel_target_slip_ratio'), 'wheel_x_disp   ': ('wheel_net_torque', 'wheel_drawbar_pull'), 'wheel_x_disp    ': ('lin_vel_error', 'ang_vel_error', 'slip_ratio_error')}
plot.plot()


#     if abs(upadted_current_input_Fx)>fMax:
#         upadted_current_input_Fx*=fMax/abs(upadted_current_input_Fx)
#     if abs(upadted_current_input_Ty)>tMax:
#         upadted_current_input_Ty*=tMax/abs(upadted_current_input_Ty)