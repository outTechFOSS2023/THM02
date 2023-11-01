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
## CREATE POLYHEDRA MAT FOR BOTH THE GROUND AND WHEEL ##
########################################################
# NOTE: WHEN POLYHEDRA-SPHERE INTERACTION IS INVESTIGATION, THE SPHERE CAN USE FrictMAT().
# BUT FOR POLYHEDRA-FACET, BOTH SHOULD USE PolyhdraMAT(). 
ground = PolyhedraMat()
ground.density = 2600 #kg/m^3 
ground.young = 1e7 #Pa
ground.poisson = 20000/1e7
ground.frictionAngle = 0.5 #rad

wheel = FrictMat()
wheel.density = 7850 #kg/m^3 
wheel.young = 1e7 #Pa
wheel.poisson = 0.3
wheel.frictionAngle = 0.8 #rad. NOTE: FRICTIONLESS WHEEL JUST ROTATES. IT DOES NOT MOVE FORWARD

###########################################################
## MODELING WHEEL AS A SINGLE SPHERE WITH RADIUS OF 0.1m ##
###########################################################
O.bodies.append(sphere(center=(0, 0, 0.1), radius=.1, fixed=False,material=wheel))
sphereWheel=O.bodies[-1]

##################################################
## MODELING GROUND AS A SINGLE PLYHEDRA ELEMENT ##
##################################################
O.bodies.append(polyhedra_utils.polyhedra(ground,v=((-20,-20,-0.1),(-20,-20,0),(-20,20,-0.1),(-20,20,0),(20,-20,-0.1),(20,-20,0),(20,20,-0.1),(20,20,0)),fixed=True, color=(0.65,0.65,0.65)))

###############################
## DEFINE ENGINES PARAMETERS ##
###############################
T_max = 1000 # MAY NOT BE REQUIRED
F_max = 1000 # MAY NOT BE REQUIRED
T_start = 0.1
F_start = 0.01

######################
## DEFINE TIME STEP ##
######################
D_t = 1.0e-7

##############################
## DEFINE TARGET VALUES ##
##############################
target_lin_vel = 0.1 # m/s
target_ang_vel = 2.0 # rad/s
target_slip_ratio = 0.5 

########################################################
## DEFINE PID GAINS (SUBJECT TO SENSITIVITY ANALYSES) ##
########################################################
# SLIP RATIO
kP_slip_ratio = 20
kI_slip_ratio = 0.1
kD_slip_ratio = 0.01

# LINEAR VELOCITY
kP_lin_vel = 20
kI_lin_vel = 0.1
kD_lin_vel = 0.01

# ANGULAR VELOCITY
kP_ang_vel = 20
kI_ang_vel = 0.1
kD_ang_vel = 0.01

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
newton=NewtonIntegrator(damping=0.4,gravity=(0,0,-9.81))

#############
## ENGINES ##
#############
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_sphereWheel_polyHedraGround/VTK/SWI_ServoPIDController_sphereWheel_polyHedraGround' ,recorders=['all'],iterPeriod=100000),
  ForceResetter(),
  TorqueEngine(dead=True,ids=[sphereWheel.id],label="torqueEngine_slip_ratio",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[sphereWheel.id],label="torqueEngine_ang_vel",moment=Vector3(0,T_start,0)),
  ForceEngine(dead=True,ids=[sphereWheel.id],label="forceEngine_lin_vel",force=Vector3(F_start,0,0)),
  InsertionSortCollider([Bo1_Polyhedra_Aabb(),Bo1_Sphere_Aabb()]),
  InteractionLoop(
    [Ig2_Sphere_Polyhedra_ScGeom()], 
    [Ip2_FrictMat_PolyhedraMat_FrictPhys()],
    [Law2_ScGeom_FrictPhys_CundallStrack()],
  ),
  newton,
  PyRunner(command='wheelSettlement()',iterPeriod=1000,label='checker'),
]
O.dt=D_t

#ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
def wheelSettlement():
    sphereWheel.blockedDOFs = 'xyXYZ' ## NOTE: ONLY MOVEMENT ALONG Z AXIS IS ALLOWED. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    print("wheelPos[2]:",sphereWheel.state.pos[2])
    unb=unbalancedForce()
    print("unb_force:",unb)
    if O.iter>5000 and unb<0.001:
        print ("TORQUE AND FORCE ARE BEING APPLIED")
        O.engines=O.engines+[PyRunner(command='addPlotData()',iterPeriod=10000)]
        checker.command='SrvoPIDController()'	

# THIS FUNCTION EMPLOYES THE PRINCIPALES OF PROPORTIONAL-INTEGRAL-DERIVATIVE (PID) CONTROL LOOP (https://en.wikipedia.org/wiki/PID_controller)
# TO ADJUST THE APPLIED TORQUE AND FORCE TO A WHEEL TO SECURE TARGET LINEAR VELOCITY, ANGULAR VELOCITY, AND SLIP RATIO AND MAINTAINING THEIR 
# EQUILIBRIUM FOR THE STEADY-STATE SOLUTION.  
obtained_lin_vel=obtained_ang_vel=obtained_slip_ratio=error_slip_ratio=error_ang_vel=error_lin_vel=torque_control_output_slip_ratio=torque_control_output_ang_vel=force_control_output_lin_vel=obtained_x_pos=obtained_y_pos=obtained_z_pos=0
def SrvoPIDController():
    global prev_error_slip_ratio, iTerm_slip_ratio, prev_error_ang_vel, iTerm_ang_vel, prev_error_lin_vel, iTerm_lin_vel
    global obtained_lin_vel, obtained_ang_vel, obtained_slip_ratio
    global error_slip_ratio, error_ang_vel, error_lin_vel
    global torque_control_output_slip_ratio, torque_control_output_ang_vel, force_control_output_lin_vel
    global obtained_x_pos, obtained_y_pos, obtained_z_pos       
    sphereWheel.blockedDOFs = 'yXZ' ## WHEELS MOVES ONLY IN +-Z AND ROTATES ABPUT +Y AXES. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    # TURNING ON TORQUE AND FORCE ENGINES
    torqueEngine_slip_ratio.dead=False
    torqueEngine_ang_vel.dead=False
    forceEngine_lin_vel.dead=False
    wheel_radius = sphereWheel.shape.radius
    obtained_x_pos = sphereWheel.state.pos[0]
    obtained_y_pos = sphereWheel.state.pos[1]
    obtained_z_pos = sphereWheel.state.pos[2]
    obtained_lin_vel = sphereWheel.state.vel[0]
    obtained_ang_vel = sphereWheel.state.angVel[1]
    obtained_slip_ratio = abs (((wheel_radius*obtained_ang_vel)-obtained_lin_vel)/(wheel_radius*obtained_ang_vel))
    print("obtained_lin_vel:",obtained_lin_vel,"obtained_ang_vel:",obtained_ang_vel,"obtained_slip_ratio:",obtained_slip_ratio)
    
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
    torqueEngine_slip_ratio.moment=Vector3(0,torque_control_output_slip_ratio,0)
    print("torque_control_output_slip_ratio:",torque_control_output_slip_ratio)
    prev_error_slip_ratio = error_slip_ratio

    # ANGULAR VELOCITY PID
    pTerm_ang_vel =  (kP_ang_vel*(error_ang_vel))
    iTerm_ang_vel += (kI_ang_vel*(error_ang_vel))
    dTerm_ang_vel =  (kD_ang_vel*(error_ang_vel-prev_error_ang_vel))
    torque_control_output_ang_vel = pTerm_ang_vel + iTerm_ang_vel + dTerm_ang_vel  
    torqueEngine_ang_vel.moment=Vector3(0,torque_control_output_ang_vel,0)
    print("torque_control_output_ang_vel:",torque_control_output_ang_vel)
    prev_error_ang_vel = error_ang_vel  

    # LINEAR VELOCITY PID
    pTerm_lin_vel =  (kP_lin_vel*(error_lin_vel))
    iTerm_lin_vel += (kI_lin_vel*(error_lin_vel))
    dTerm_lin_vel =  (kD_lin_vel*(error_lin_vel-prev_error_lin_vel))
    force_control_output_lin_vel = pTerm_lin_vel + iTerm_lin_vel + dTerm_ang_vel  
    forceEngine_lin_vel.force=Vector3(force_control_output_lin_vel,0,0)
    print("force_control_output_lin_vel:",force_control_output_lin_vel)
    prev_error_lin_vel = error_lin_vel 
 
def addPlotData():
    wheel_radius = sphereWheel.shape.radius
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
    plot.saveDataTxt("/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_sphereWheel_polyHedraGround/Output/SWI_ServoPIDController_sphereWheel_polyHedraGround")
plot.plots = {'wheel_x_disp': ('wheel_obtained_lin_vel', 'wheel_target_lin_vel'), 'wheel_x_disp ': ('wheel_obtained_ang_vel', 'wheel_target_ang_vel'), 'wheel_x_disp  ': ('wheel_obtained_slip_ratio', 'wheel_target_slip_ratio'), 'wheel_x_disp   ': ('wheel_net_torque', 'wheel_drawbar_pull'), 'wheel_x_disp    ': ('lin_vel_error', 'ang_vel_error', 'slip_ratio_error')}
plot.plot()

#     if abs(upadted_current_input_Fx)>fMax:
#         upadted_current_input_Fx*=fMax/abs(upadted_current_input_Fx)
#     if abs(upadted_current_input_Ty)>tMax:
#         upadted_current_input_Ty*=tMax/abs(upadted_current_input_Ty)


























# # IMPORT NECESSARY MODULES
# from yade import pack, timing, qt
# from yade.params import *
# from math import pi
# from yade import polyhedra_utils
# from yade import ymport

# FricDegree = 30 
# youngRegolith=0.7e8 
# youngContainer=210e9 
# poissonRegolith=0.3 
# poissionContainer=0.25 
# densRegolith=2650 
# densContainer=7850 


# # CREATE POLYHEDRA MAT FOR BOTH THE GROUND AND WHEEL 
# ground = PolyhedraMat()
# ground.density = 2600 #kg/m^3 
# ground.young = 1E7 #Pa
# ground.poisson = 20000/1E7
# ground.frictionAngle = 0.5 #rad

# wheel = FrictMat()
# wheel.density = 7850 #kg/m^3 
# wheel.young = 1E7 #Pa
# wheel.poisson = 0.3
# wheel.frictionAngle = 0.8 #rad


# O.bodies.append(sphere(center=(0, 0, 0.1), radius=.1, fixed=False,material=wheel))
# sphereWheel=O.bodies[-1]


# O.bodies.append(polyhedra_utils.polyhedra(ground,v=((-20,-20,-0.1),(-20,-20,0),(-20,20,-0.1),(-20,20,0),(20,-20,-0.1),(20,-20,0),(20,20,-0.1),(20,20,0)),fixed=True, color=(0.65,0.65,0.65)))

# ## DEFINE ENGINE PARAMETERS
# T_max = 1000
# F_max = 1000
# T_start = 0.1
# F_start = 0.01

# ## DEFINE TIME STEP
# D_t = 1.0e-7

# ## DEFINE TARGET VALUES
# target_lin_vel = 0.1 # m/s
# target_ang_vel = 2.0 # rad/s
# target_slip_ratio = 0.5 

# ## DEFINE PID GAINS (SUBJECT TO SENSITIVITY ANALYSES)
# # SLIP RATIO
# kP_slip_ratio = 2
# kI_slip_ratio = 0.1
# kD_slip_ratio = 0.01

# # LINEAR VELOCITY
# # kP_lin_vel = 500
# # kI_lin_vel = 0
# # kD_lin_vel = 0.01

# # ANGULAR VELOCITY
# # kP_ang_vel = 1000
# # kI_ang_vel = 2
# # kD_ang_vel = 0.02

# ## DEFINE INITIAL ERRORS
# # SLIP RATIO
# prev_error_slip_ratio = 0
# iTerm_slip_ratio = 0

# # LINEAR VELOCITY
# # prev_error_lin_vel = 0
# # iTerm_lin_vel = 0

# # ANGULAR VELOCITY
# # prev_error_ang_vel = 0
# # iTerm_ang_vel = 0

# newton=NewtonIntegrator(damping=0.4,gravity=(0,0,-9.81))

# O.engines=[
#   ForceResetter(),
#   TorqueEngine(dead=True,ids=[sphereWheel.id],label="torqueEngine",moment=Vector3(0,T_start,0)),
#   ForceEngine(dead=True,ids=[sphereWheel.id],label="forceEngine",force=Vector3(F_start,0,0)),
#   InsertionSortCollider([Bo1_Polyhedra_Aabb(),Bo1_Sphere_Aabb()]),
#   InteractionLoop(
#     [Ig2_Sphere_Polyhedra_ScGeom()], 
#     [Ip2_FrictMat_PolyhedraMat_FrictPhys()],
#     [Law2_ScGeom_FrictPhys_CundallStrack()],
#   ),
#   newton,
#   PyRunner(command='freeFall()',iterPeriod=1000,label='checker'),
# ]
# O.dt=D_t

# #ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
# def freeFall():
#     sphereWheel.blockedDOFs = 'xyXYZ' ## NOTE: ONLY DEPOSITION ALONG Z AXIS IS ALLOWED
#     print("wheelPos[2]:",sphereWheel.state.pos[2])
#     unb=unbalancedForce()
#     print("unb_force:",unb)
#     if O.iter>5000 and unb<0.001:
#         print ("torque is being applied")
#         checker.command='compute_slip_ratio()'  

# #DEFINE A FUNCTION TO CALCULATE THE SLIP RATIO
# def compute_slip_ratio():
#     global prev_error_slip_ratio, iTerm_slip_ratio
#     sphereWheel.blockedDOFs = 'yXZ' ## WHEELS MOVES IN +-Z AND ROTATES ABPUT +Y AXES
#     torqueEngine.dead=False
#     forceEngine.dead=True
#     wheel_radius = sphereWheel.shape.radius
#     obtained_lin_vel = sphereWheel.state.vel[0]
#     obtained_ang_vel = sphereWheel.state.angVel[1]
#     obtained_slip_ratio = abs (((wheel_radius*obtained_ang_vel)-obtained_lin_vel)/(wheel_radius*obtained_ang_vel))
#     print("obtained_lin_vel:",obtained_lin_vel,"obtained_ang_vel:",obtained_ang_vel,"obtained_slip_ratio:",obtained_slip_ratio)
    
#     #COMPUTE THE ERROR VALUE (WE ASSUME THAT THE TORQUE ENGINE IS ABLE TO SECURE THE SLIP RATIO ALONE-- LET'S SEE)
#     error_slip_ratio = (target_slip_ratio - obtained_slip_ratio)
#     print("error_slip_ratio:",error_slip_ratio)

#     # COMPUTE THE PID CONTROL OUTPUT VARIABLES
#     # SLIP RATIO
#     pTerm_slip_ratio =  (kP_slip_ratio*(error_slip_ratio))
#     iTerm_slip_ratio += (kI_slip_ratio*(error_slip_ratio))
#     dTerm_slip_ratio =  (kD_slip_ratio*(error_slip_ratio-prev_error_slip_ratio))
#     torque_control_output = pTerm_slip_ratio + iTerm_slip_ratio + dTerm_slip_ratio
#     torqueEngine.moment=Vector3(0,torque_control_output,0)
#     prev_error_slip_ratio = error_slip_ratio
#     print("torque_control_output:",torque_control_output)


# # IMPORT NECESSARY MODULES
# from yade import pack, timing, qt
# from yade.params import *
# from math import pi
# from yade import polyhedra_utils
# from yade import ymport

# FricDegree = 30 
# youngRegolith=0.7e8 
# youngContainer=210e9 
# poissonRegolith=0.3 
# poissionContainer=0.25 
# densRegolith=2650 
# densContainer=7850 


# # CREATE POLYHEDRA MAT FOR BOTH THE GROUND AND WHEEL 
# ground = PolyhedraMat()
# ground.density = 2600 #kg/m^3 
# ground.young = 1E7 #Pa
# ground.poisson = 20000/1E7
# ground.frictionAngle = 0.5 #rad

# wheel = FrictMat()
# wheel.density = 7850 #kg/m^3 
# wheel.young = 1E7 #Pa
# wheel.poisson = 0.3
# wheel.frictionAngle = 0.5 #rad


# O.bodies.append(sphere(center=(0, 0, 0.1), radius=.1, fixed=False,material=wheel))
# sphereWheel=O.bodies[-1]


# O.bodies.append(polyhedra_utils.polyhedra(ground,v=((-2,-2,-0.1),(-2,-2,0),(-2,2,-0.1),(-2,2,0),(2,-2,-0.1),(2,-2,0),(2,2,-0.1),(2,2,0)),fixed=True, color=(0.65,0.65,0.65)))

# ## DEFINE ENGINE PARAMETERS
# T_max = 1000
# F_max = 1000
# T_start = 0.1
# F_start = 0.01

# ## DEFINE TIME STEP
# D_t = 1.0e-7

# ## DEFINE TARGET VALUES
# target_lin_vel = 0.1 # m/s
# target_ang_vel = 2.0 # rad/s
# target_slip_ratio = 0.5 

# ## DEFINE PID GAINS (SUBJECT TO SENSITIVITY ANALYSES)
# # SLIP RATIO
# # kP_slip = 1.0
# # kI_slip = 0.1
# # kD_slip = 0.01

# # LINEAR VELOCITY
# kP_lin_vel = 500
# kI_lin_vel = 0
# kD_lin_vel = 0.01

# # ANGULAR VELOCITY
# kP_ang_vel = 1000
# kI_ang_vel = 2
# kD_ang_vel = 0.02

# ## DEFINE INITIAL ERRORS
# # SLIP RATIO
# # prev_error_slip = 0
# # integral_slip = 0

# # LINEAR VELOCITY
# prev_error_lin_vel = 0
# iTerm_lin_vel = 0

# # ANGULAR VELOCITY
# prev_error_ang_vel = 0
# iTerm_ang_vel = 0

# newton=NewtonIntegrator(damping=0.4,gravity=(0,0,-9.81))

# O.engines=[
#   ForceResetter(),
#   TorqueEngine(dead=True,ids=[sphereWheel.id],label="torqueEngine",moment=Vector3(0,T_start,0)),
#   ForceEngine(dead=True,ids=[sphereWheel.id],label="forceEngine",force=Vector3(F_start,0,0)),
#   InsertionSortCollider([Bo1_Polyhedra_Aabb(),Bo1_Sphere_Aabb()]),
#   InteractionLoop(
#     [Ig2_Sphere_Polyhedra_ScGeom()], 
#     [Ip2_FrictMat_PolyhedraMat_FrictPhys()],
#     [Law2_ScGeom_FrictPhys_CundallStrack()],
#   ),
#   newton,
#   PyRunner(command='freeFall()',iterPeriod=1,label='checker'),
# ]
# O.dt=D_t

# #ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
# def freeFall():
#     sphereWheel.blockedDOFs = 'xyXYZ' ## NOTE: ONLY DEPOSITION ALONG Z AXIS IS ALLOWED
#     print("wheelPos[2]:",sphereWheel.state.pos[2])
#     unb=unbalancedForce()
#     print("unb_force:",unb)
#     if O.iter>5000 and unb<0.001:
#         print ("torque is being applied")
#         checker.command='compute_slip_ratio()'  

# #DEFINE A FUNCTION TO CALCULATE THE SLIP RATIO
# def compute_slip_ratio():
#     global prev_error_lin_vel, iTerm_lin_vel, prev_error_ang_vel, iTerm_ang_vel
#     sphereWheel.blockedDOFs = 'yXZ' ## WHEELS MOVES IN +-Z AND ROTATES ABPUT +Y AXES
#     torqueEngine.dead=False
#     forceEngine.dead=False
#     wheel_radius = sphereWheel.shape.radius
#     obtained_lin_vel = sphereWheel.state.vel[0]
#     obtained_ang_vel = sphereWheel.state.angVel[1]
#     obtained_slip_ratio = abs (((wheel_radius*obtained_ang_vel)-obtained_lin_vel)/(wheel_radius*obtained_ang_vel))
#     print("obtained_lin_vel:",obtained_lin_vel,"obtained_ang_vel:",obtained_ang_vel,"obtained_slip_ratio:",obtained_slip_ratio)
    
#     #COMPUTE THE ERROR VALUES (WE ASSUME THAT IF obtained_lin_vel and obtained_ang_vel CAN BE CALIBRATED, THE obtained_slip_ratio IS CALIBRATED AUTOMATICALLY)
#     error_lin_vel = (target_lin_vel - obtained_lin_vel)
#     error_ang_vel = (target_ang_vel - obtained_ang_vel)
#     print("error_lin_vel:",error_lin_vel, "error_ang_vel:",error_ang_vel)

#     # COMPUTE THE PID CONTROL OUTPUT VARIABLES
#     # LINEAR VELOCITY
#     pTerm_lin_vel =  (kP_lin_vel*(error_lin_vel))
#     iTerm_lin_vel += (kI_lin_vel*(error_lin_vel))*D_t
#     dTerm_lin_vel =  (kD_lin_vel*(error_lin_vel-prev_error_lin_vel))/D_t
#     force_control_output = pTerm_lin_vel + iTerm_lin_vel + dTerm_lin_vel
#     forceEngine.force=Vector3(force_control_output,0,0)
#     prev_error_lin_vel = error_lin_vel


#     # ANGULAR VELOCITY
#     pTerm_ang_vel =  (kP_ang_vel*(error_ang_vel))
#     iTerm_ang_vel += (kI_ang_vel*(error_ang_vel))*D_t
#     dTerm_ang_vel =  (kD_ang_vel*(error_ang_vel-prev_error_ang_vel))/D_t
#     torque_control_output = pTerm_ang_vel + iTerm_ang_vel + dTerm_ang_vel
#     torqueEngine.moment=Vector3(0,torque_control_output,0)
#     prev_error_ang_vel = error_ang_vel

#     print("force_control_output:",force_control_output, "torque_control_output:",torque_control_output)  




# # IMPORT NECESSARY MODULES
# from yade import pack, timing, qt
# from yade.params import *
# from math import pi
# from yade import polyhedra_utils
# from yade import ymport

# FricDegree = 30 
# youngRegolith=0.7e8 
# youngContainer=210e9 
# poissonRegolith=0.3 
# poissionContainer=0.25 
# densRegolith=2650 
# densContainer=7850 


# # CREATE POLYHEDRA MAT FOR BOTH THE GROUND AND WHEEL 
# ground = PolyhedraMat()
# ground.density = 2600 #kg/m^3 
# ground.young = 1E7 #Pa
# ground.poisson = 20000/1E7
# ground.frictionAngle = 0.5 #rad

# wheel = FrictMat()
# wheel.density = 7850 #kg/m^3 
# wheel.young = 1E7 #Pa
# wheel.poisson = 0.3
# wheel.frictionAngle = 0.5 #rad




# O.bodies.append(sphere(center=(0, 0, 0.1), radius=.1, fixed=False,material=wheel))
# sphereWheel=O.bodies[-1]


# O.bodies.append(polyhedra_utils.polyhedra(ground,v=((-2,-2,-0.1),(-2,-2,0),(-2,2,-0.1),(-2,2,0),(2,-2,-0.1),(2,-2,0),(2,2,-0.1),(2,2,0)),fixed=True, color=(0.65,0.65,0.65)))

# # DEFINE SERVO-CONTROLLERS PARAMETERS
# dt=1.0e-7
# max_force = 1000
# max_torque = 500
# torque_start = 0.01
# force_start=0.01
# # DEFINE THE TARGET VALUES FOR THE SLIP RATIO, LINEAR VELOCITY AND ANGULAR VELOCITY
# target_linear_velocity = 0.1
# target_angular_velocity = 2
# target_slip_ratio = ((sphereWheel.shape.radius*target_angular_velocity)-target_linear_velocity)/(sphereWheel.shape.radius*target_angular_velocity)
# print("target_slip_ratio:",target_slip_ratio,"sphereWheel.shape.radius:",sphereWheel.shape.radius)



# # DEFINE THE PID CONTROLLER GAIN
# Kp = 5
# Ki = 0.5
# Kd = 0.01

# # INITIALIZE THE PID CONTROLLER VARIABLES
# error = 0
# iTerm = 0
# previous_error = 0

# newton=NewtonIntegrator(damping=0.4,gravity=(0,0,-9.81))

# O.engines=[
#   ForceResetter(),
#   TorqueEngine(dead=True,ids=[sphereWheel.id],label="torqueEngine",moment=Vector3(0,torque_start,0)),
#   ForceEngine(dead=True,ids=[sphereWheel.id],label="forceEngine",force=Vector3(force_start,0,0)),
#   InsertionSortCollider([Bo1_Polyhedra_Aabb(),Bo1_Sphere_Aabb()]),
#   InteractionLoop(
#     [Ig2_Sphere_Polyhedra_ScGeom()], 
#     [Ip2_FrictMat_PolyhedraMat_FrictPhys()],
#     [Law2_ScGeom_FrictPhys_CundallStrack()],
#   ),
#   newton,
#   PyRunner(command='freeFall()',iterPeriod=1,label='checker'),
# ]
# O.dt=dt

# #ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
# def freeFall():
#     sphereWheel.blockedDOFs = 'xyXYZ' ## NOTE: ONLY DEPOSITION ALONG Z AXIS IS ALLOWED
#     print("wheelPos[2]:",sphereWheel.state.pos[2])
#     unb=unbalancedForce()
#     print("unb_force:",unb)
#     if O.iter>5000 and unb<0.001:
#         print ("torque is being applied")
#         checker.command='compute_slip_ratio()'  

# #DEFINE A FUNCTION TO CALCULATE THE SLIP RATIO
# def compute_slip_ratio():
#     global error,iTerm,previous_error
#     sphereWheel.blockedDOFs = 'yXZ' ## WHEELS MOVES IN +-Z AND ROTATES ABPUT +Y AXES
#     torqueEngine.dead=False
#     forceEngine.dead=False
#     linear_velocity = sphereWheel.state.vel[0]
#     angular_velocity =sphereWheel.state.angVel[1]
#     slip_ratio = abs(((sphereWheel.shape.radius*sphereWheel.state.angVel[1])-sphereWheel.state.vel[0])/(sphereWheel.shape.radius*sphereWheel.state.angVel[1]))
#     print("linear_velocity:",linear_velocity,"angular_velocity:",angular_velocity,"slip_ratio:",slip_ratio)
#     #COMPUTE THE ERROR VALUES
#     error_slip_ratio = (target_slip_ratio - slip_ratio)
#     error_linear_velocity = target_linear_velocity - linear_velocity
#     error_angular_velocity = target_angular_velocity - angular_velocity
#     # COMPUTE THE PID CONTROL OUTPUT VARIABLES
#     current_error= error_slip_ratio + error_linear_velocity + error_angular_velocity
#     print("current_error:",current_error)
#     pTerm = Kp*(current_error)
#     iTerm += (Ki*error)*dt
#     dTerm = (Kd*(error-previous_error))/dt
#     control_output = pTerm + iTerm + dTerm 
#     print("control_output:",control_output)
#     forceEngine.force[0]=control_output
#     torqueEngine.moment[1]=control_output
#     print("wheelPos[0]:",sphereWheel.state.pos[0])
#     error=error_slip_ratio
#     previous_error = error




































# # IMPORT NECESSARY MODULES
# from yade import pack, timing, qt
# from yade.params import *
# from math import pi
# from yade import polyhedra_utils
# from yade import ymport

# FricDegree = 30 
# youngRegolith=0.7e8 
# youngContainer=210e9 
# poissonRegolith=0.3 
# poissionContainer=0.25 
# densRegolith=2650 
# densContainer=7850 


# # CREATE POLYHEDRA MAT FOR BOTH THE GROUND AND WHEEL 
# ground = PolyhedraMat()
# ground.density = 2600 #kg/m^3 
# ground.young = 1E7 #Pa
# ground.poisson = 20000/1E7
# ground.frictionAngle = 0.5 #rad

# wheel = FrictMat()
# wheel.density = 7850 #kg/m^3 
# wheel.young = 1E7 #Pa
# wheel.poisson = 0.3
# wheel.frictionAngle = 0.5 #rad




# O.bodies.append(sphere(center=(0, 0, 0.1), radius=.1, fixed=False,material=wheel))
# sphereWheel=O.bodies[-1]


# O.bodies.append(polyhedra_utils.polyhedra(ground,v=((-2,-2,-0.1),(-2,-2,0),(-2,2,-0.1),(-2,2,0),(2,-2,-0.1),(2,-2,0),(2,2,-0.1),(2,2,0)),fixed=True, color=(0.65,0.65,0.65)))

# # DEFINE SERVO-CONTROLLERS PARAMETERS
# dt = 1e-7
# fMax = 1000
# tMax = 1000
# torque_start = 0.1
# torque_step = 0.01
# force_start=0.1
# torque_step = 0.01
# target_slip_ratio = 0.5
# target_ground_Vel=0.1
# target_ang_Vel=(sphereWheel.shape.radius*target_ground_Vel)/(1-target_slip_ratio)

# kP = 0.5  # Proportional Gain Coeeficient
# kI = 0.1  # Integral Gain Coeeficient
# kD = 0.05 # Derivative Gain Coefficient 

# #INITIALIZE VARIABLES FOR PID SERVO-CONTROLLER:
# previous_error_ground_vel=0.0
# previous_error_ang_vel=0.0
# iTerm_ground_vel=0.0
# iTerm_ang_vel=0.0

# newton=NewtonIntegrator(damping=0.4,gravity=(0,0,-9.81))

# O.engines=[
#   ForceResetter(),
#   TorqueEngine(dead=True,ids=[sphereWheel.id],label="torqueEngine",moment=Vector3(0,torque_start,0)),
#   ForceEngine(dead=True,ids=[sphereWheel.id],label="forceEngine",force=Vector3(force_start,0,0)),
#   InsertionSortCollider([Bo1_Polyhedra_Aabb(),Bo1_Sphere_Aabb()]),
#   InteractionLoop(
#     [Ig2_Sphere_Polyhedra_ScGeom()], 
#     [Ip2_FrictMat_PolyhedraMat_FrictPhys()],
#     [Law2_ScGeom_FrictPhys_CundallStrack()],
#   ),
#   newton,
#   PyRunner(command='freeFall()',iterPeriod=100000,label='checker'),
# ]
# O.dt=dt

# #ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
# def freeFall():
#     sphereWheel.blockedDOFs = 'xyXYZ' ## NOTE: ONLY DEPOSITION ALONG Z AXIS IS ALLOWED
#     print("wheelPos[2]:",sphereWheel.state.pos[2])
#     unb=unbalancedForce()
#     print("unb_force:",unb)
#     if O.iter>5000 and unb<0.1:
#         print ("torque is being applied")
#         checker.command='compute_slip_ratio()'  

# #DEFINE A FUNCTION TO CALCULATE THE SLIP RATIO
# def compute_slip_ratio():
#     global previous_error_ground_vel,previous_error_ang_vel,iTerm_ground_vel,iTerm_ang_vel
#     sphereWheel.blockedDOFs = 'yXZ' ## WHEELS MOVES IN +-Z AND ROTATES ABPUT +Y AXES
#     torqueEngine.dead=False
#     forceEngine.dead=False
#     # COMPUTE THE CURRENT GROUND AND ANGULAR VELOCITIES 
#     current_ground_vel=sphereWheel.state.vel[0]
#     current_ang_vel = sphereWheel.state.angVel[1]
#     print("current_ground_vel:",current_ground_vel,"current_ang_vel:",current_ang_vel)
#     # COMPUTE THE CURRENT RESULTANT Fx AND TY
#     current_resultant_Fx=abs(O.forces.f(sphereWheel.id)[0])
#     current_resultant_Ty=abs(O.forces.t(sphereWheel.id)[1])
#     print("current_resultant_Fx:",current_resultant_Fx,"current_resultant_Ty:",current_resultant_Ty)
#     # COMPUTE THE CURRENT SLIP RATIO
#     current_slip_ratio=((current_ang_vel*sphereWheel.shape.radius)-current_ground_vel)/(current_ang_vel*sphereWheel.shape.radius)
#     print("current_slip_ratio:",current_slip_ratio)
#     # SET THE CURRENT INPUT Fx AND Ty
#     current_input_Fx = forceEngine.force[0]
#     current_input_Ty = torqueEngine.moment[1]
#     print("current_input_Fx:",current_input_Fx,"current_input_Ty:",current_input_Ty)
#     # CHECK IF THE WHEEL MOVES IN +X DIRECTION
#     print("wheelPos[0]:",sphereWheel.state.pos[0])
#     # COMPUTE THE ERROR, PTROPORTIONAL, INTEGRAL, AND DERIVATIVE TERMS
#     current_error_ground_vel=abs(current_ground_vel-target_ground_Vel)
#     current_error_ang_vel=abs(current_ang_vel-target_ang_Vel)
#     # pTerm
#     pTerm_ground_vel=kP*current_error_ground_vel
#     pTerm_ang_vel=kP*current_error_ang_vel
#     # iTerm
#     iTerm_ground_vel+=(kI*current_error_ground_vel)*dt
#     iTerm_ang_vel+=(kI*current_error_ang_vel)*dt
#     # dTerm
#     dTerm_ground_vel=kD*(current_error_ground_vel-previous_error_ground_vel)*dt
#     dTerm_ang_vel=kD*(current_error_ang_vel-previous_error_ang_vel)*dt
#     # SAVING THE CURRENT VALUES OF ERRORS
#     previous_error_ground_vel=current_error_ground_vel
#     previous_error_ang_vel=current_error_ang_vel
#     # UPDATING THE INPUT FORCE AND TORQUE
#     upadted_current_input_Fx = (pTerm_ground_vel + iTerm_ground_vel + dTerm_ground_vel)
#     upadted_current_input_Ty = (pTerm_ang_vel + iTerm_ang_vel + dTerm_ang_vel)
#     # UPDATING THE FORCE AND TORQUE ENGINES MOVEMENT
#     forceEngine.force[0]=upadted_current_input_Fx
#     torqueEngine.moment[1]=upadted_current_input_Ty
#     # LIMITING THE FORCE AND TORQUE MAXIMUM
#     if abs(upadted_current_input_Fx)>fMax:
#         upadted_current_input_Fx*=fMax/abs(upadted_current_input_Fx)
#     if abs(upadted_current_input_Ty)>tMax:
#         upadted_current_input_Ty*=tMax/abs(upadted_current_input_Ty)











