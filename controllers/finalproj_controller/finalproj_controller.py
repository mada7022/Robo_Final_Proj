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

<<<<<<< HEAD
=======
# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
# mode = 'autonomous'
# mode = 'automap'

def show_display(display, map,length,height):
    for i in range(length):
        for j in range(height):
            if (map[i, j] == 1):
                display.setColor(int(0xFFFFFF))
                display.drawPixel(360-j, i)

def get_heuristic(begin_coord, end_coord):
    return np.linalg.norm(np.array(begin_coord) - np.array(end_coord))

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

###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame

    start_w = (4.5,8) # (Pose_X, Pose_Z) in meters
    end_w = (10,7) # (Pose_X, Pose_Z) in meters
    # end_w = (7.5, 3.25)


    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = (int(start_w[0]*30), int(start_w[1]*30)) # (x, y) in 360x360 map
    end = (int(end_w[0]*30), int(end_w[1]*30)) # (x, y) in 360x360 map
    #start : 45,45
    #end:115, 115

    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    class map_node:
        def __init__(self, original_coord, distance_weight, heuristic_weight, parent_node):
            self.coord = original_coord
            self.distance = distance_weight
            self.total_weight = distance_weight + heuristic_weight
            self.parent = parent_node

        def get_parent(self):
            return self.parent

        def get_coord(self):
            return self.coord

        def get_distance(self):
            return self.distance

        def get_totalWeight(self):
            return self.total_weight

        def set_totalWeight(self, distance_weight, heuristic_weight):
            self.total_weight = distance_weight + heuristic_weight

        def set_parent(self, parent_node):
            self.parent = parent_node


    def get_neighbors(coords, map):
        x = coords[0]
        y = coords[1]

        neighbor_choords = []


        for t_x, t_y in [(x-1,y), (x+1,y), (x, y-1), (x, y+1)]:
            if 0 <= t_x < len(map) and 0 <= t_y < len(map[0]) and map[t_x][t_y] == 0:
                    neighbor_choords.append((t_x, t_y))

        return neighbor_choords


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
>>>>>>> 546fbcba7db218d5890ebe088254ad93a7eb28cb

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
