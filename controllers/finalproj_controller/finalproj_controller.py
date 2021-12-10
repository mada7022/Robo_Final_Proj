"""finalproj_controller controller."""

#imported from lab4
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor, Keyboard


mode = "manual"



# These are your pose values that you will update by solving the odometry equations
pose_x = 0.197
pose_y = 0.678
pose_theta = 0

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28


# create the Robot instance.
robot=Robot()
timestep = int(robot.getBasicTimeStep())

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)


# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Initialize the Display
display = robot.getDevice("display")


# Main Control Loop:
while robot.step(timestep) != -1:

    #####################################################
    #                 Sensing                           #
    #####################################################

    # Read ground sensors
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()



    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad



    #set display color to red (robot's current pose)
    display.setColor(0xFF0000)
    # display.drawPixel(converted_pose[0], converted_pose[1])
    display.drawPixel(pose_x, pose_y)



    #####################################################
    #                 Robot controller                  #
    #####################################################
    vL, vR = 0,0
    limited_max_speed = MAX_SPEED * 0.6
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT:
            vL = -limited_max_speed
            vR = limited_max_speed
        elif key == keyboard.RIGHT:
            vL = limited_max_speed
            vR = -limited_max_speed
        elif key == keyboard.UP:
            vL = limited_max_speed
            vR = limited_max_speed
        elif key == keyboard.DOWN:
            vL = -limited_max_speed
            vR = -limited_max_speed
        elif key == ord(' '):
            vL = 0
            vR = 0



    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
