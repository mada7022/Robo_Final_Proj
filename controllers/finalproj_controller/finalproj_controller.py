"""finalproj_controller controller."""

from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

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

        return path
    ############################################################################

    # Part 2.1: Load map (map.npy) from disk and visualize it]
    map = (np.load("map.npy"))
    print("Map loaded")

    ############################################################################

    # Part 2.2: Compute an approximation of the “configuration space”
    def createSquare(cntr, temp_map, i, j):
        if cntr >= 9:
            return

        for t_i in [i - 1, i, i + 1]:
            for t_j in [j - 1, j, j + 1]:
                if 0 <= t_i < len(temp_map) and 0 <= t_j < len(temp_map[0]):
                    if temp_map[t_i][t_j] != 1:
                        temp_map[t_i][t_j] = 1
                        createSquare(cntr + 1, temp_map, t_i, t_j)


    temp_map = np.copy(map)
    for i in range(len(map)):
        for j in range(len(map[i])):
            if map[i][j] == 1:
                createSquare(0, temp_map, i,j)
    map = temp_map
    ############################################################################

    # Part 2.3 continuation: Call path_planner
    # map = np.fliplr(map)
    path = path_planner(map, start, end)
    ############################################################################


    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    waypoints = []

    counter = 0
    # waypoints.append((start[0]/30,start[1]/30))
    for step in path:
        counter +=1
        if counter %15 == 0:
            waypoints.append((step[0]/30, step[1]/30))

    waypoints.reverse()
    waypoints.append((end[0] / 30, end[1] / 30))

    np.save("path.npy", waypoints)


    plt.imshow(np.fliplr(map))
    # plt.imshow(map)
    for p in path:
        plt.plot(12*30 - int(p[1]), int(p[0]), '-bD')
    for p in waypoints:
        plt.plot(12*30 - int(p[1]*30), int(p[0]*30), '-gD')
    plt.show()
    ############################################################################

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
#create a 12x12 array full of 0s
# map = np.array([[0.0 for j in range(int(360))] for i in range(int(360))]) # Replace None by a numpy 2D floating point array
map = np.zeros(shape=(360,360))
INCREMENT_VALUE = 5e-3
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = [] # Replace with code to load your path

    waypoints = np.load("path.npy")

    # # DISPLAY MAP
    # plt.imshow(np.fliplr(map))
    # plt.imshow(map)
    # for p in waypoints:
    #     plt.plot(12*30-int(p[1] * 30), int(p[0] * 30), '-gD')
    # plt.show()

state = 1 # use this to iterate through your path

# appending values to our "waypoint" arr
waypoint = [[target[0], 0, target[1]] for target in waypoints]



while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        ################ ^ [End] Do not modify ^ ##################



        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
        dx = int(wx*30)
        dx = 359 if dx>=360 else dx
        dx = 0 if dx < 0 else dx

        dy = int(wy*30)
        dy = 359 if dy>=360 else dy
        dy = 0 if dy < 0 else dy

        temp_var = map[dx][dy]
        if (temp_var < 1):
            temp_var += INCREMENT_VALUE
        else:
            temp_var = 1

        map[dx][dy] = temp_var
        # print("map[{}, {}] = {}".format(int(wx*30), int(wy*30), temp_var))


        if rho < LIDAR_SENSOR_MAX_RANGE:
            ## Part 1.3: visualize map gray values.

            ## You will eventually REPLACE the following 2 lines with a more robust version of the map
            ## with a grayscale drawing containing more levels than just 0 and 1.
            g = map[dx][dy]
            if (g == 1):
                display.setColor(int(0xFFFFFF))
            else:
                display.setColor(int((g*256**2+g*256+g)))
            display.drawPixel(360-dy,dx)

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*30),int(pose_x*30))



    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT:
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            true_arr = (map>0.5)*1
            np.save("map.npy", true_arr)
            print("Map file saved")

        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")

            show_display(display, map,360,360)


        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode

        ############################################################################
        # Feedback controller
        ############################################################################

        #STEP 3: Compute wheelspeeds
        limited_max_speed = MAX_SPEED * 0.6
        limited_max_speed_ms = MAX_SPEED_MS * 0.6

        dest_pose_x, dest_pose_y, dest_pose_theta = 0,0, -math.pi/2

        if state < len(waypoint):
            dest_pose_x, dest_pose_y = waypoint[state][0], waypoint[state][2]


        # STEP 1: Calculate the error
        rho = get_heuristic((pose_x, pose_y), (dest_pose_x, dest_pose_y))
        alpha = -(math.atan2(waypoint[state][2] - pose_y, waypoint[state][0] - pose_x) + pose_theta)


        # STEP 2: Controller
        p1 = 7
        p2 = 15

        x_r = p1 * rho
        theta_r = p2 * alpha

        # STEP 3: Compute wheelspeeds
        vL = (x_r - ((theta_r * AXLE_LENGTH) / 2))
        vR = (x_r + ((theta_r * AXLE_LENGTH) / 2))

        # Normalize wheelspeed (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
        if (vL != 0 or vR != 0):
            max_val = max(abs(vL), abs(vR))

            vL = vL / max_val * limited_max_speed
            vR = vR / max_val * limited_max_speed

        if vL < -1 * limited_max_speed:
            vL = 0
        if vR < -1 * limited_max_speed:
            vR = 0


        # next state
        if rho < 0.5:
            state += 1

            if state < len(waypoint):
                print("reached state {}. Next state={} ({},{})".format(state-1, state, waypoint[state][0], waypoint[state][2]))

        #once we've reached our goal
        if (get_heuristic((pose_x, pose_y), (waypoint[-1][0], waypoint[-1][2])) < 0.5):
            print("REACHED FINALS GOAL!")
            robot_parts[MOTOR_LEFT].setVelocity(0)
            robot_parts[MOTOR_RIGHT].setVelocity(0)
            break


    limited_max_speed=MAX_SPEED * 0.6
    limited_max_speed_ms = MAX_SPEED_MS * 0.6
    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/limited_max_speed*limited_max_speed*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/limited_max_speed*limited_max_speed*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/limited_max_speed*limited_max_speed_ms*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)