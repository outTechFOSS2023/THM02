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
O.bodies.append(polyhedra_utils.polyhedra(ground,v=((-20,-20,-0.1),(-20,-20,0),(-20,20,-0.1),(-20,20,0),(20,-20,-0.1),(20,-20,0),(20,20,-0.1),(20,20,0)),fixed=True, color=(0,1.0,0)))

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
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_sphereWheel_polyHedraGround/VTK_twoTerms/SWI_ServoPIDController_sphereWheel_polyHedraGround_twoTerms' ,recorders=['all'],iterPeriod=100000),
  ForceResetter(),
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
obtained_lin_vel=obtained_ang_vel=obtained_slip_ratio=error_slip_ratio=error_ang_vel=error_lin_vel=torque_control_output_ang_vel=force_control_output_lin_vel=obtained_x_pos=obtained_y_pos=obtained_z_pos=0
def SrvoPIDController():
    global prev_error_ang_vel, iTerm_ang_vel, prev_error_lin_vel, iTerm_lin_vel
    global obtained_lin_vel, obtained_ang_vel, obtained_slip_ratio
    global error_ang_vel, error_lin_vel
    global torque_control_output_ang_vel, force_control_output_lin_vel
    global obtained_x_pos, obtained_y_pos, obtained_z_pos       
    sphereWheel.blockedDOFs = 'yXZ' ## WHEELS MOVES ONLY IN +-Z AND ROTATES ABPUT +Y AXES. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    # TURNING ON TORQUE AND FORCE ENGINES
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
    # ANGULAR VELOCITY ERROR
    error_ang_vel = (target_ang_vel - obtained_ang_vel)

    # LINEAR VELOCITY ERROR
    error_lin_vel = (target_lin_vel - obtained_lin_vel)
    print("error_slip_ratio:",error_slip_ratio,"error_ang_vel:",error_ang_vel, "error_lin_vel:",error_lin_vel)

    ############################################################
    ######### COMPUTE THE PID CONTROL OUTPUT VARIABLES #########
    #########    AND UPDATE TORQUE AND FORCE VALUES    #########
    ############################################################
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
    wheel_net_torque = torque_control_output_ang_vel
    wheel_drawbar_pull = force_control_output_lin_vel 
    lin_vel_error = error_lin_vel
    ang_vel_error = error_ang_vel
    slip_ratio_error = error_slip_ratio
    yade.plot.addData({'i':O.iter,'wheel_target_lin_vel':wheel_target_lin_vel,'wheel_target_ang_vel':wheel_target_ang_vel,'wheel_target_slip_ratio':wheel_target_slip_ratio,\
        'wheel_x_disp':wheel_x_disp,'wheel_y_disp':wheel_y_disp,'wheel_z_disp':wheel_z_disp,\
        'wheel_obtained_lin_vel':wheel_obtained_lin_vel,'wheel_obtained_ang_vel':wheel_obtained_ang_vel,'wheel_obtained_slip_ratio':wheel_obtained_slip_ratio,\
        'wheel_net_torque':wheel_net_torque,'wheel_drawbar_pull':wheel_drawbar_pull,\
        'lin_vel_error':lin_vel_error,'ang_vel_error':ang_vel_error})
    plot.saveDataTxt("/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_sphereWheel_polyHedraGround/Output_twoTerms/SWI_ServoPIDController_sphereWheel_polyHedraGround_twoTerms")
plot.plots = {'wheel_x_disp': ('wheel_obtained_lin_vel', 'wheel_target_lin_vel'), 'wheel_x_disp ': ('wheel_obtained_ang_vel', 'wheel_target_ang_vel'), 'wheel_x_disp  ': ('wheel_obtained_slip_ratio', 'wheel_target_slip_ratio'), 'wheel_x_disp   ': ('wheel_net_torque', 'wheel_drawbar_pull'), 'wheel_x_disp    ': ('lin_vel_error', 'ang_vel_error')}
plot.plot()

