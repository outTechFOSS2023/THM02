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
ground = FrictMat()
ground.density = 2650 #kg/m^3 
ground.young = 0.7e8 #Pa
ground.poisson = 0.3
ground.frictionAngle = 0.5 #rad

wheel = FrictMat()
wheel.density = 7850 #kg/m^3 
wheel.young = 210e9  #Pa
wheel.poisson = 0.3
wheel.frictionAngle = 0.3 #rad. NOTE: FRICTIONLESS WHEEL JUST ROTATES. IT DOES NOT MOVE FORWARD

#########################################################
## MODELING GROUND AS A PRECOMPACTED GRANULAR ASSEMBLY ##
#########################################################
O.bodies.append(ymport.text("/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetCylinderWheel_sphericalGround/HW_Deposited_Bed.txt",shift=Vector3(0,0,0),material=ground))
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

#######################################################################################
######### MODELING WHEEL AS A FACET CYLINDER WITH 0.15m RADIUS AND 0.1m WIDTH #########
#######################################################################################
wheel_radius = 0.1505330 #m
wheel_width  = 0.1000000 #m 
whhel_x_center,wheel_y_center,wheel_z_center= 0.2,sampleWidth_bed/2,(maxZ_bed+wheel_radius)
facetWheel = geom.facetCylinder((whhel_x_center,wheel_y_center,wheel_z_center),radius=wheel_radius,height=wheel_width,orientation=Quaternion((1,0,0),-pi/2),wallMask=7,segmentsNumber=20,wire=False,color=Vector3(255,255,255))
num_facets = len(facetWheel)
wheel_normal_force = 430*9.81/4 # N
wheel_mass = wheel_normal_force/9.81 # kg
for i in facetWheel:
  i.state.mass = wheel_mass/num_facets
  i.state.inertia = (1,1,1)
facetWheelID = O.bodies.appendClumped(facetWheel)
Wheel1=O.bodies[-1]
wheel_mass = Wheel1.state.mass
Ix = ((wheel_mass*(wheel_radius)**2)/4)+ ((wheel_mass*(wheel_width)**2)/12)  #m^4. MANULLY COMPUTED
Iy = (wheel_mass*(wheel_radius)*(wheel_radius))/2                            #m^4. MANULLY COMPUTED
Iz = Ix                                                                      #m^4. MANULLY COMPUTED  
## UPDATING MOMENTS OF INERTIA FOR THE CLUMPED WHEEL
Wheel1.state.inertia=Vector3(Ix,Iy,Iz)
facets = [b for b in O.bodies if isinstance(b.shape,Facet)] # LIST ALL FACETS 
print("inertia:",Wheel1.state.inertia,"mass:",Wheel1.state.mass) 

#####################################
## CREATE WALLS AROUND THE PACKING ##
#####################################

mn,mx=Vector3(minX_bed,minY_bed,minZ_bed),Vector3(maxX_bed,maxY_bed,10*maxZ_bed)
walls=aabbWalls([mn,mx],thickness=0,material=wheel)
wallIds=O.bodies.append(walls)

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
newton=NewtonIntegrator(damping=0.7,gravity=(0,0,-9.81))

#############
## ENGINES ##
#############
O.engines=[VTKRecorder(fileName='/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetCylinderWheel_sphericalGround/VTK/SWI_ServoPIDController_facetCylinderWheel_sphericalGround' ,recorders=['all'],iterPeriod=10000),
  ForceResetter(),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine_slip_ratio",moment=Vector3(0,T_start,0)),
  TorqueEngine(dead=True,ids=[Wheel1.id],label="torqueEngine_ang_vel",moment=Vector3(0,T_start,0)),
  ForceEngine(dead=True,ids=[Wheel1.id],label="forceEngine_lin_vel",force=Vector3(F_start,0,0)),
  InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Box_Aabb(),Bo1_Facet_Aabb()]),
  InteractionLoop(
    [Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
    [Ip2_FrictMat_FrictMat_FrictPhys()],
    [Law2_ScGeom_FrictPhys_CundallStrack()]
  ),
  newton,
  PyRunner(command='wheelSettlement()',iterPeriod=100,label='checker'),
]
O.dt=D_t


#ALLOW THE WHEEL TO SETTLE ON THE GROUND SURFACE
def wheelSettlement():
    Wheel1.blockedDOFs = 'xyXYZ' ## NOTE: ONLY MOVEMENT ALONG Z AXIS IS ALLOWED. THEREFORE, ALL OTHER DOFs ARE CONSTRAINED
    Wheel1.state.angVel[0] = 0
    Wheel1.state.angVel[2] = 0
    print("wheel_ang_vel[0]:",Wheel1.state.angVel[0])
    print("wheel_ang_vel[2]:",Wheel1.state.angVel[2])
    print("wheelPos[2]:",Wheel1.state.pos[2])
    unb=unbalancedForce()
    print("unb_force:",unb)
    if O.iter>10000 and unb<0.01:
        print ("TORQUE AND FORCE ARE BEING APPLIED")
        O.engines=O.engines+[PyRunner(command='addPlotData()',iterPeriod=1000)]
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
    Wheel1.state.angVel[0] = 0
    Wheel1.state.angVel[2] = 0
    # TURNING ON TORQUE AND FORCE ENGINES
    torqueEngine_slip_ratio.dead=False
    torqueEngine_ang_vel.dead=False
    forceEngine_lin_vel.dead=False
    obtained_x_pos = Wheel1.state.pos[0]
    obtained_y_pos = Wheel1.state.pos[1]
    obtained_z_pos = Wheel1.state.pos[2]
    obtained_lin_vel = Wheel1.state.vel[0]
    obtained_ang_vel = Wheel1.state.angVel[1]
    obtained_slip_ratio = abs (((wheel_radius*obtained_ang_vel)-obtained_lin_vel)/(wheel_radius*obtained_ang_vel))
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
    plot.saveDataTxt("/home/ngoudarzi/Desktop/Large Scale SWI/SWI_ServoPIDController_facetCylinderWheel_sphericalGround/Output/SWI_ServoPIDController_facetCylinderWheel_sphericalGround")
plot.plots = {'wheel_x_disp': ('wheel_obtained_lin_vel', 'wheel_target_lin_vel'), 'wheel_x_disp ': ('wheel_obtained_ang_vel', 'wheel_target_ang_vel'), 'wheel_x_disp  ': ('wheel_obtained_slip_ratio', 'wheel_target_slip_ratio'), 'wheel_x_disp   ': ('wheel_net_torque', 'wheel_drawbar_pull'), 'wheel_x_disp    ': ('lin_vel_error', 'ang_vel_error', 'slip_ratio_error')}
plot.plot()


