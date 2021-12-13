"""finalproj_controller controller."""

#imported from lab4
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor, Keyboard


################################################################################
######### Set up variables and initialize things ###############################
################################################################################
def world_to_map(pose_x, pose_y):
    ##### calculations here
    # https://drive.google.com/file/d/1hsYZIMMoSasWAD4TrqjwaIvc5tasvH2y/view?usp=drivesdk

    x = int(150+pose_x*200)
    y = int(150+pose_y*200)

    x = 299 if x > 299 else x
    x = 0 if x < 0 else x

    y = 299 if y > 299 else y
    y = 0 if y < 0 else y

    return (x, y)

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28
MAX_SPEED_MS = 0.22
AXLE_LENGTH = 0.16 # [m]

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

#enable lidar usage (implemented from lab 4&5)
lidar = robot.getDevice("LDS-01")
lidar.enable(timestep)
lidar.enablePointCloud()

LIDAR_SENSOR_MAX_RANGE = 3 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(LIDAR_ANGLE_RANGE/2., -LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)

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
INCREMENT_VALUE = 5e-3

#map = 300x300
map = np.array([[0.0 for i in range(300)] for j in range(300)])
explored_bools = np.array([[False for i in range(300)] for j in range(300)])

#set mode
mode = "automap"
################################################################################



##########################################################################################################
#                                  Sensing and Display Functions                                         #
##########################################################################################################
def update_map(type, coordinate):
    if type == "pose":
        map[coordinate[0]][coordinate[1]] = 2

    if type == "lidar" and map[coordinate[0]][coordinate[1]] < 1:
        map[coordinate[0]][coordinate[1]] += INCREMENT_VALUE
        explored_bools[coordinate[0]][coordinate[1]] = True


    return map

def refreshScreen(first, second):
    g = map[first][second]
    if g == 2:
        display.setColor(0xFF0000)
    elif g == 1:
        display.setColor(int(0xFFFFFF))
    else:
        display.setColor(int((g*256**2+g*256+g)))

    display.drawPixel(first, second)

def doLidar(pose_x, pose_y, pose_theta, lidar):
    # return a list of obstacle coordinates!
    lidar_sensor_readings = lidar.getRangeImage()

    lidar_readings = []
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        rx = -math.sin(alpha)*rho
        ry = math.cos(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y


        lidar_readings.append((wx, wy))

    return lidar_readings

def update_display(pose_x,pose_y,pose_theta, lidar):
    # display robot's current position in red
    coordinate = world_to_map(pose_x, pose_y)
    update_map("pose", coordinate)
    refreshScreen(coordinate[0], coordinate[1])

    # display out lidar readings
    readings = doLidar(pose_x, pose_y, pose_theta, lidar)
    for reading in readings:
        coordinate = world_to_map(reading[0], reading[1])

        update_map("lidar", coordinate)
        refreshScreen(coordinate[0], coordinate[1])
##########################################################################################################


##########################################################################################################
#                                       Controller Functions                                             #
##########################################################################################################
def manual_mode(limited_max_speed, keyboard):
    vL, vR = (0,0)
    key = keyboard.getKey()
    while (keyboard.getKey() != -1): pass

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

    return (vL, vR)

def automap_mode(limited_max_speed, ground_sensors):
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
    def hard_robo_spin():  # should give exact 360
        orig = wb_compass_get_values()

        vL = limited_max_speed
        vR = -1 * limited_max_speed
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)

        robot.step(timestep)

        while wb_compass_get_values() != orig:
            robot.step(timestep)

        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

    def robo_spin():  # estimate 360
        vL = limited_max_speed
        vR = -1 * limited_max_speed
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)
        for i in range(100):  # toggle iterations to get a 360
            robot.step(timestep)

        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

    # inneficient
    def bad_get_random_point(explored_bools):
        while True:
            int1 = np.randint(0, 301)  # range inclusive, exclusive
            int2 = np.randint(0, 301)  # RANGE MUST BE ADJUSTED FOR NEW MAP DIMENSIONS
            if explored_bools[int1][int2] == False:
                break

        return (int1, int2)

    def path_planner(map, start, end):
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''

        path = []  # path is a list of touples from beg to end
        INT_MAX = 2147483647

        world_height = len(map)
        world_width = len(map[0])

        # A* Algorithm ()
        visited_nodes = []
        node_dict = {}  # key=coord, value=map_node
        pqueue = []

        starting_node = map_node(start, 0, get_heuristic(start, end), None)
        node_dict[start] = starting_node
        pqueue.append(starting_node)

        while len(pqueue) > 0:
            min_value = INT_MAX
            min_node = None
            for node in pqueue:
                if node.get_totalWeight() < min_value:
                    min_value = node.get_totalWeight()
                    min_node = node

            pqueue.remove(min_node)
            visited_nodes.append(min_node)

            if min_node.get_coord() == end:
                temp_node = min_node
                while (temp_node is not None):
                    path.append(temp_node.get_coord())
                    temp_node = temp_node.get_parent()

                break

            neighbors = get_neighbors(min_node.get_coord(), map=map)
            for neighbor_coord in neighbors:
                temp_heuristic = get_heuristic(neighbor_coord, end)
                if neighbor_coord not in node_dict:
                    node_dict[neighbor_coord] = map_node(neighbor_coord, 1, temp_heuristic, min_node)
                    pqueue.append(node_dict[neighbor_coord])
                else:
                    neighbor_node = node_dict[neighbor_coord]

                    # skip over the nodes that have been "visited"
                    if neighbor_node in visited_nodes:
                        continue

                    # check this part!
                    if neighbor_node.get_totalWeight() > 1 + temp_heuristic:
                        neighbor_node.set_totalWeight(1, temp_heuristic)
                        neighbor_node.set_parent(min_node)

                        if neighbor_node in pqueue:
                            pqueue.remove(neighbor_node)

        return path

    def explored_percent(explored_bools):
        length = len(explored_bools) ** 2
        count = 0
        for i in range(length):
            for j in range(length):
                if explored_bools[i][j] == True:
                    count += 1

        return (count / length)

    # def createSquare(cntr, temp_map, i, j, expanded_bools, explored_bools):
    #     if cntr >= 9:
    #         return
    #
    #     for t_i in [i - 1, i, i + 1]:
    #         for t_j in [j - 1, j, j + 1]:
    #             if 0 <= t_i < len(temp_map) and 0 <= t_j < len(temp_map[0]):
    #                 if temp_map[t_i][t_j] != 1:
    #                     temp_map[t_i][t_j] = 1
    #                     explored_bools[t_i][t_j] = True
    #                     createSquare(cntr + 1, temp_map, t_i, t_j, expanded_bools, explored_bools)
    #
    #     expanded_bools = explored_bools

    # Before while loop initializes...
    robo_spin()  # start by spinning to get info.
    bad_get_random_point(explored_bools)  # get unexplored point
    path_planner(map,(pose_x,pose_y), bad_get_random_point(explored_bools))  # plan path

    mapnotdone = True
    while (robot.step(timestep) != -1 and mapnotdone):
        obstacle = False
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
            if ps[i] > 100:
                obstacle = True

        if obstacle:
            # back away for wall, will help with rotating!
            vL = -1 * vL
            vR = -1 * vR
            for i in range(
                    10):  # if angle of obstacle is an issue, turn to face the sensor that sensed the obstacle and then back up.
                robot.step(timestep)  # back up away from the wall so obstacle doesn't trigger infinitely

            robo_spin()
            # note: use new expanded_bools array. If the pixel we're trying to expand's index is false
            # in the expanded array, expand it.
            # when createsquare updates 0's to 1's update explored_bools at that index to be true.
            # If expanded Bools is True, don't expand. At the the very end of the createSquare
            # function, set expanded_bools = explored_bools.

            # temp_map = np.copy(map)
            # for i in range(len(map)):
            #     for j in range(len(map[i])):
            #         if map[i][j] == 1 and expanded_bools[i][j] == False:
            #             createSquare(0, temp_map, i,j, expanded_bools, explored_bools)
            # map = temp_map

            path_planner(map, start, end)  # variables?
            # continue # forget the rest, do while beginning loop right after

        else:
            # keep following path
            pass


##########################################################################################################


# Main Control Loop:
while robot.step(timestep) != -1:

    #####################################################
    #                 Sensing and Display               #
    #####################################################
    # Read ground sensors
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # get current position
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    # get current orientation
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2])) - 1.5708)
    pose_theta = rad

    #display lidar reading and epuck curr pose
    update_display(pose_x,pose_y,pose_theta, lidar)



    #####################################################
    #                 Robot controller                  #
    #####################################################
    vL, vR = 0,0
    limited_max_speed = MAX_SPEED * 0.6
    if mode == 'manual':
        vL, vR = manual_mode(limited_max_speed, keyboard)

    elif mode == 'automap':
        vL, vR = automap_mode(limited_max_speed, gsr)

    else:
        vL, vR = (0,0)


    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)




if __name__ == "__main__":
    print("hello world!")