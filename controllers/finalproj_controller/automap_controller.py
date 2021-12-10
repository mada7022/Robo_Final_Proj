from controller import Robot, Motor

ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

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
    expanded_bools = []

    def hard_robo_spin(): # should give exact 360
        orig = wb_compass_get_values()

        vL = limited_max_speed
        vR = -1 * limited_max_speed
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)

        robot.step(timestep)

        while wb_compass_get_values() != orig:
            robot.step(timestep)

        return
    
    def robo_spin(): # estimate 360
        vL = limited_max_speed
        vR = -1 * limited_max_speed
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)
        for i in range(100): #toggle iterations to get a 360
            robot.step(timestep)

    def bad_get_random_point(explored_bools):
        # import random
        while True:
            int1 = math.rand(0, 301) # range inclusive, exclusive
            int2 = math.rand(0, 301) # RANGE MUST BE ADJUSTED FOR NEW MAP DIMENSIONS
            if explored_bools[int1][int2] == False:
                break
        
        return (int1, int2)

    def path_planner(map, start, end):
        pass

    def explored_percent(explored_bools):
        length = len(explored_bools)**2
        count = 0
        for i in range(length):
            for j in range(length):
                if explored_bools[i][j] == True:
                    count += 1

        return (count/length)
    
    def createSquare(cntr, temp_map, i, j, expanded_bools, explored_bools):
        if cntr >= 9:
            return

        for t_i in [i - 1, i, i + 1]:
            for t_j in [j - 1, j, j + 1]:
                if 0 <= t_i < len(temp_map) and 0 <= t_j < len(temp_map[0]):
                    if temp_map[t_i][t_j] != 1:
                        temp_map[t_i][t_j] = 1
                        explored_bools[t_i][t_j] = True
                        createSquare(cntr + 1, temp_map, t_i, t_j, expanded_bools, explored_bools)

        expanded_bools = explored_bools


    # Before while loop initializes...
    robo_spin() # start by spinning to get info.
    bad_get_random_point() # get unexplored point
    path_planner() # plan path

    mapnotdone = True
    while(robot.step(timestep) != -1 and mapnotdone):
        obstacle = False
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
            if ps[i] > 100:
                obstacle = True

        if obstacle:
            vL = -1 * vL
            vR = -1 * vR
            for i in range(10): # if angle of obstacle is an issue, turn to face the sensor that sensed the obstacle and then back up.
                robot.step(timestep) # back up away from the wall so obstacle doesn't trigger infinitely

            robo_spin()
            # note: use new expanded_bools array. If the pixel we're trying to expand's index is false
            # in the expanded array, expand it. 
            # when createsquare updates 0's to 1's update explored_bools at that index to be true.
            # If expanded Bools is True, don't expand. At the the very end of the createSquare 
            # function, set expanded_bools = explored_bools.
            temp_map = np.copy(map)
            for i in range(len(map)):
                for j in range(len(map[i])):
                    if map[i][j] == 1 and expanded_bools[i][j] == False:
                        createSquare(0, temp_map, i,j, expanded_bools, explored_bools)
            map = temp_map

            path_planner(map, start, end) # variables?
            continue # forget the rest, do while beginning loop right after

