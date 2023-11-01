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
# CREATE POLYHEDRA MAT FOR BOTH THE GROUND AND WHEEL 
ground = PolyhedraMat()
ground.density = 2600 #kg/m^3 
ground.young = 1e7 #Pa
ground.poisson = 20000/1e7
ground.frictionAngle = 0.5 #rad

wheel = PolyhedraMat()
wheel.density = 1000 #kg/m^3 
wheel.young = 1e7 #Pa
wheel.poisson = 0.3
wheel.frictionAngle = 0.8 #rad. NOTE: FRICTIONLESS WHEEL JUST ROTATES. IT DOES NOT MOVE FORWARD

###########################################################
######### IMPORTING A FACET WHEEL OF 0.15m RADIUS #########
###########################################################
facetWheel = ymport.stl('/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_polyHedraGround/scaledWheel.stl',scale=0.001,shift=Vector3(0,0,0.10),wire=False,color=Vector3(0.9,0.0,0.4),material=wheel)
num_facets = len(facetWheel)
print("num_facets:",num_facets) 
wheel_mass=8.0 #kg. FOR NOW THE VEHICLE NORMAL FORCE IS IGNORED
for i in facetWheel:
  i.state.mass = wheel_mass/num_facets
  i.state.inertia = (1,1,1)
facetWheelID = O.bodies.appendClumped(facetWheel)
Wheel1=O.bodies[-1]
facets = [b for b in O.bodies if isinstance(b.shape,Facet)] # list of facets in simulation

##################################################
## MODELING GROUND AS A SINGLE POLYHEDRA ELEMENT ##
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
D_t = 1.0e-8

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
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_polyHedraGround/VTK/SWI_ServoPIDController_facetWheel_polyHedraGround' ,recorders=['all'],iterPeriod=100000),
  ForceResetter(),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine_slip_ratio",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine_ang_vel",moment=Vector3(0,T_start,0)),
  ForceEngine(dead=True,ids=[Wheel1.id],label="forceEngine_lin_vel",force=Vector3(F_start,0,0)),
  InsertionSortCollider([Bo1_Polyhedra_Aabb(),Bo1_Facet_Aabb()],verletDist=0.1),
  InteractionLoop(
    [Ig2_Facet_Polyhedra_PolyhedraGeom()],
    [Ip2_PolyhedraMat_PolyhedraMat_PolyhedraPhys()],
    [Law2_PolyhedraGeom_PolyhedraPhys_Volumetric()]
  ),
  newton,
  PyRunner(command='wheelSettlement()',iterPeriod=100,label='checker'),
]
O.dt=0.025*polyhedra_utils.PWaveTimeStep()


#ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
def wheelSettlement():
    Wheel1.blockedDOFs = 'xyXYZ' ## NOTE: ONLY MOVEMENT ALONG Z AXIS IS ALLOWED. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    print("wheelPos[2]:",Wheel1.state.pos[2])
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
    Wheel1.blockedDOFs = 'yXZ' ## WHEELS MOVES ONLY IN +-Z AND ROTATES ABPUT +Y AXES. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    # TURNING ON TORQUE AND FORCE ENGINES
    torqueEngine_slip_ratio.dead=False
    torqueEngine_ang_vel.dead=False
    forceEngine_lin_vel.dead=False
    wheel_radius = 0.15
    obtained_x_pos = Wheel1.state.pos[0]
    obtained_y_pos = Wheel1.state.pos[1]
    obtained_z_pos = Wheel1.state.pos[2]
    obtained_lin_vel = Wheel1.state.vel[0]
    obtained_ang_vel = Wheel1.state.angVel[1]
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
    wheel_radius = 0.15
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
    plot.saveDataTxt("/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetWheel_polyHedraGround/Output/SWI_ServoPIDController_facetWheel_polyHedraGround")
plot.plots = {'wheel_x_disp': ('wheel_obtained_lin_vel', 'wheel_target_lin_vel'), 'wheel_x_disp ': ('wheel_obtained_ang_vel', 'wheel_target_ang_vel'), 'wheel_x_disp  ': ('wheel_obtained_slip_ratio', 'wheel_target_slip_ratio'), 'wheel_x_disp   ': ('wheel_net_torque', 'wheel_drawbar_pull'), 'wheel_x_disp    ': ('lin_vel_error', 'ang_vel_error', 'slip_ratio_error')}
plot.plot()


