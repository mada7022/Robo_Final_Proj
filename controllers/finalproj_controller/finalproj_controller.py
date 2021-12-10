"""finalproj_controller controller."""

#imported from lab4
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor, Keyboard


mode = "manual"

#map = 300x300
map = np.array([[0 for i in range(300)] for j in range(300)])

def world_to_map(pose_x, pose_y):
    ##### calculations here
    # https://drive.google.com/file/d/1hsYZIMMoSasWAD4TrqjwaIvc5tasvH2y/view?usp=drivesdk

    return (int(150+pose_x*200), int(150+pose_y*200))



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
    gs.enable(timestep)

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
    # print("pose_x: {}\tpose_y: {}".format(pose_x, pose_y))

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad



    #set display color to red (robot's current pose)
    display.setColor(0xFF0000)
    # display.drawPixel(converted_pose[0], converted_pose[1])
    a,b = world_to_map(pose_x, pose_y)
    display.drawPixel(a,b)


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
            vL = MAX_SPEED
            vR = MAX_SPEED

        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED

        elif key == ord(' '):
            vL = 0
            vR = 0



    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

###################
#
# Automap
#
###################
if mode == 'automap':
    ##############################################################################
    # start by doing a 360 to gain info of robo's surroundings.
    # Initialize a boolean array -- false for unexplored, true for explored. Update lidar code accordingly.
    # Choose a random point that's unexplored, false in the boolean array.
    # Path plan to it, follow the path until the point is reached
    # OR more likely, until the robot gets close to a wall.
    # Once thaty happens, do another 360 to gain info, rerun path planning to get a new path.
    # follow until the point is reached, OR etc. etc. until it is reached.
    # once a random point is reached, run explored percent
    # Pick a new random point, redo the same stuff.
    # Pick random points until boolean array is 95%-ish true.
    ##############################################################################

    explored_bools = []

    def robo_spin():
        # robo 360
        pass

    def get_random_point():
        # gets random unexplored point on map
        pass

    def path_planner(map, start, end):
        pass

    def explored_percent(exploredPixels):
        length = len(exploredPixels)
        count = 0
        for i in range(length):
            for j in range(length):
                if exploredPixels[i][j] == True:
                    count += 1

        return (count/length)

    # we'll change that variable
    mapnotdone = True
    while(robot.step(timestep) != -1 or mapnotdone):
        break
