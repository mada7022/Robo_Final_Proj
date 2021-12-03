"""lab3 controller."""
# Copyright Prof. Bradley Hayes <bradley.hayes@colorado.edu> 2021
# University of Colorado Boulder CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math
import numpy as np




# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67/1 # [rad/s]
MAX_SPEED_MS = 0.22/1 # [m/s]
AXLE_LENGTH = 0.16 # [m]

WHEEL_RADIUS = 0.033

MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
target_pos = ('inf', 'inf')
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0



#target/goal positions
targetcount = 0
world_targets = [[5.21442, 0, 7.72136], [5.5, 0, 5.27226], [3.63448, 0, 4.63976], [3.92854, 0, 3.23861]]
robot_targets = []


#axis conversion stuff...
OW_rotation_matrix = np.array([[1,0,0], [0,1,0], [0,0,-1]])
O_transformation = np.array([3.289, 0, 8.719])
O_transformation.transpose()


def robot_point(OtoW_rotation_matrix, O_translation, O_point):
    rotation = np.copy(OtoW_rotation_matrix)
    translation = np.copy(O_translation)
    rotation.transpose()
    
    return np.subtract(np.matmul(rotation, O_point),np.matmul(rotation, translation))
    
    
    

#convert real world coordinate plane to robot coordinates
for target in world_targets: 
    robot_targets.append(robot_point(OW_rotation_matrix, O_transformation, target))

#dest pose variables
dest_pose_x = robot_targets[0][0]
dest_pose_y = robot_targets[0][2]
dest_pose_theta = math.pi/2

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

debug = False

while robot.step(timestep) != -1:

    #STEP 2: Calculate sources of error
    dist_err = math.sqrt((dest_pose_x-pose_x)**2 + (dest_pose_y-pose_y)**2)
    bearing_err = math.atan2((dest_pose_y-pose_y),(dest_pose_x-pose_x)) - pose_theta
    heading_err = pose_theta - dest_pose_theta
    
    if(debug):
        print("===> dist_err: {:<30}heading_err: {:<40}bearing_err: {:<50}".format(dist_err, heading_err, bearing_err))
    
    
    #stopping mechanism
    if ((dist_err < 0.05 and dist_err > -0.05) and (heading_err < math.pi/6 and heading_err > -math.pi/6)):
        print("completedTargeNum: {} \t\t node:({:.2f},{:.2f})".format(targetcount, dest_pose_x, dest_pose_y))
        
        
        #if we've reached the intermediary targets
        if(targetcount < 4):
                        
            dest_pose_x = robot_targets[targetcount][0]
            dest_pose_y = robot_targets[targetcount][2]
            targetcount += 1
                 
        #the robot reached it's final target 
        else:
            print("Goal Reached!")
            robot_parts[MOTOR_LEFT].setVelocity(0)
            robot_parts[MOTOR_RIGHT].setVelocity(0)
            break


        
    #STEP 2.4: Feedback Controller
    p1 = 1*1
    p2 = 10.0*.8
    p3 = 0.2*5

    x_r = p1*dist_err
    theta_r = p2*bearing_err + p3*heading_err
    
    
    
    #STEP 1.2: Inverse Kinematics Equations
    phi_l = (x_r - ((theta_r * AXLE_LENGTH)/2)) / WHEEL_RADIUS
    phi_r = (x_r + ((theta_r * AXLE_LENGTH)/2)) / WHEEL_RADIUS        
 
    
    # STEP 2.5: Compute wheel velocities (vL, vR)
    vL = WHEEL_RADIUS * phi_l
    vR = WHEEL_RADIUS * phi_r
    
    
    
    #STEP 2.7: Proportional velocities
    if (vL == 0 and vR == 0):
        pass
    else:
        max_val = max(abs(vL), abs(vR))

        vL = vL/max_val * MAX_SPEED
        vR = vR/max_val * MAX_SPEED
        


    #STEP 2.6: Clamp wheel speeds
    if vL > MAX_SPEED:
        vL = MAX_SPEED
    if vR > MAX_SPEED:
        vR = MAX_SPEED
    if vL < -1 * MAX_SPEED:
        vL = -1 * MAX_SPEED
    if vR < -1 * MAX_SPEED:
        vR = -1 * MAX_SPEED




    # Odometry code. Don't change speeds (vL and vR) after this line
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/AXLE_LENGTH
    
    
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28

    if (debug):
        print("pos: {:.2f}, {:.2f}, {:.2f}\t\t vL:{:.2f}, vR:{:.2f}".format(pose_x, pose_y, pose_theta, vL, vR))
    
    # TODO: Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)

    