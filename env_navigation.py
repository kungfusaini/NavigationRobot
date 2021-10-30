#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, os
from sensor_msgs.msg import Illuminance, LaserScan
from tf.transformations import euler_from_quaternion
import time
from signal import signal, SIGINT
from sys import exit
import math
from parameters import *
from tree import *

# I proposed to import this library as documented in my writing
try:
    import playsound
except ImportError:
    print('Sound library not found')

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

# All of these values are chosen together, unless otherwise specified.

# Error message
e = """
Communications Failed
"""

# I contributed to create these variables needed for the online localisation.
# Positional variables
angular_pos = 0
robot_position = (0, 0)
orientation = 'N'

# Flag if the robot is in contact with an object
bumped = False

# Distance from the robot to the closest object
min_laser_distance = float('inf')

# Colour that the light sensor is reading
colour = ""

# Colour read in the parking space
colour_read = 'not defined'

# Light sensor reading
light_value = 0

# Laser sensor reading
laser_readings = []

# Obstacles on the map
obstacles = set()


############################################################
#                       ROS METHODS                        #
############################################################

# I moved and adjusted the callback methods from Term 1 work
# Retrieves data from the light sensor and updates the colour
def callback_light_sensor(data):
    global light_value
    light_value = data.illuminance
    localisation_colours()


# Retrieves the odom topic and updates the robot's yaw
def callback_odom(data):
    global angular_pos
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    angular_pos = yaw


# Finds the distance of an object via the laser
# If within a set range, set the bumped flag to true
# This simulates a bumper sensor
def callback_laser(data):
    global min_laser_distance
    global bumped
    global laser_readings
    laser_readings = data.ranges
    min_laser_distance = min(data.ranges)
    # 0.13 is the minimal distance to bump
    bumped = min(data.ranges[0:2] + data.ranges[-2:]) < 0.15


# Updates the values for Odometry, the laser scanner and the light sensor
def listener():
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/scan", LaserScan, callback_laser)
    rospy.Subscriber("/light_sensor_plugin/lightSensor", Illuminance, callback_light_sensor)


def get_key():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def handler(s, frame):
    set_velocity()
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    exit(0)


############################################################
#                   MOVEMENT METHODS                       #
############################################################


# Sets the twist message for the robot's x, y, z and phi speeds
def set_velocity(x=0.0, y=0.0, z=0.0, phi=0.0):
    global pub
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = phi

    pub.publish(twist)


# Implements rotation with P controller
def rotate(des_phi, linear_vel=0):
    kp = 1
    error_threshold = math.radians(0.5)

    while True:
        # First_error and second_error solve the angle shifting from -180 degrees and 180 degrees
        first_error = des_phi - angular_pos

        # If second_error is positive, 360 degrees is added...
        if first_error >= 0:
            second_error = -2 * math.pi + first_error
        # ... otherwise 360 degrees is subtracted
        else:
            second_error = 2 * math.pi + first_error

        # Use the smallest error (shortest path to the des_phi)
        if abs(first_error) < abs(second_error):
            phi_error = first_error
        else:
            phi_error = second_error

        # If within the error_threshold, resume navigation
        if abs(phi_error) < error_threshold:
            break

        angular_vel = kp * phi_error
        set_velocity(phi=angular_vel, x=linear_vel)

    set_velocity(x=linear_vel)


# I helped proactively in the making of this method.
# This is referenced in Miscellaneous to allow for acceleration
# and deceleration during movement.
# Moves the robot the given distance given in meters
def move(distance=0.0):
    # Number of steps simulating gradual acceleration
    steps = 20

    # Movement times when acceleration is 1/4 of the total time
    # Calculated from Velocity Time graph
    total_mov_time = abs((4 * distance) / (3 * WAFFLE_SPEED))
    constant_vel_time = total_mov_time / 2
    acceleration_time = total_mov_time / 4

    # Sets the direction according to where we are moving
    if distance < 0:
        direction = -1
    else:
        direction = 1

    # Acceleration
    for step in range(1, steps + 1):
        speed = (direction * WAFFLE_SPEED / steps) * step
        set_velocity(x=speed)
        time.sleep(acceleration_time / steps)

    # Constant velocity
    set_velocity(x=direction * WAFFLE_SPEED)
    time.sleep(constant_vel_time)

    # Deceleration
    for step in range(steps, 0, -1):
        speed = (direction * WAFFLE_SPEED / steps) * step
        set_velocity(x=speed)
        time.sleep(acceleration_time / steps)

    set_velocity(x=0)


# Defines methods for moving forward and backward
# one tile from the localisation line
def forward_step():
    move(distance=line_block_size)


def backward_step():
    move(distance=-line_block_size)


############################################################
#                  LOCALISATION METHODS                    #
############################################################

# I contributed greatly to this section account for the localisation methods
# All of these have been references in Section 4.2.1

# Sets the colour according to the light value
def localisation_colours():
    global colour, light_value
    if 124 <= light_value < 126:
        colour = "grey"
    elif 254 < light_value < 256:
        colour = "white"
    elif 200 < light_value < 202:
        colour = "floor"
    else:
        colour = "not defined"


# Gets the correct sensor model according to the colour detected by the light sensor
def get_colour_list(colour_in):
    if colour_in == 'white':
        return white_prob
    elif colour_in == 'grey':
        return grey_prob
    else:
        raise Exception('Colour not found')


# Updates pos_belief (robot position on the line)
def bayes_filter(pos_belief, colour_in, vel=0):
    colour_prob = get_colour_list(colour_in)
    n = 0

    # Updating based on the colour
    if vel == 0:
        for i in range(len(pos_belief)):
            pos_belief[i] = colour_prob[i] * pos_belief[i]
            n = n + pos_belief[i]

        for i in range(len(pos_belief)):
            pos_belief[i] = pos_belief[i] * (1 / n)

    # Updating based on the action taken and the read colour
    else:
        prev_belief = pos_belief[:]
        for i in range(len(pos_belief)):
            if i == 0 and vel == 1:
                pos_belief[i] = 0
            elif i == len(pos_belief) - 1 and vel == -1:
                pos_belief[i] = 0
            else:
                pos_belief[i] = colour_prob[i] * prev_belief[i - vel] / ACTION_MODEL_ERROR
            n = n + pos_belief[i]

        for i in range(len(pos_belief)):
            pos_belief[i] = pos_belief[i] * (1 / n)

    return pos_belief


# Task 1: localisation
def localise():
    # The initial equally distributed probability...
    pos_prob = [1.0 / len(white_prob)] * len(white_prob)

    # ...is then updated using the bayes filter
    pos_prob = bayes_filter(pos_prob, colour)

    direction = 1  # 1 is forward, -1 is backwards
    steps_to_move = 2
    remaining_steps = steps_to_move

    # Moves 2 step forward, 4 backward, 6 forward, 8 backward
    # until localised with a 75% threshold
    while max(pos_prob) <= 0.75:

        # Keeps robot orientated north
        rotate(math.pi / 2)

        if direction == 1:
            forward_step()
        else:
            backward_step()

        remaining_steps -= 1
        pos_prob = bayes_filter(pos_prob, colour, direction)

        # Switches direction once it has moved the required amount of steps
        if remaining_steps == 0:
            steps_to_move += 2
            remaining_steps = steps_to_move
            direction *= -1

    localised_pos = pos_prob.index(max(pos_prob))

    print("Localised to cell: {}".format(localised_pos))
    return localised_pos


############################################################
#                       A* METHODS                         #
############################################################

# I contributed to the making of ALL the A* methods (all the methods in this subsection).
# These are referenced in section 4.2.3 and 4.2.4.

# Adds surroundings in a circular fashion
# around an obstacle so that the robot doesn't collide
def get_surroundings(x, y):
    surroundings = [(0, 0), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
    output = set()

    for surrounding_node in surroundings:
        to_add = Node(x + surrounding_node[0], y + surrounding_node[1], 0)
        # Checks that we're not marking the goal as an obstacle
        if to_add != get_goal():
            output.add(to_add)

    return output


# Returns nodes corresponding to the localisation line barrier
def get_barrier_nodes():
    barrier_nodes = set()

    # y displacement of the barrier at the end of the line
    y_pos = int(math.floor((line_block_size * len(white_prob) - 0.1) / SIZE_SQUARE))

    # half the number of total nodes forming the barrier
    number_squares = int(math.ceil(1.0 / SIZE_SQUARE / 2.0))

    # Barrier is either side of node (0,0) so nodes have negative then positive
    # x values
    for x_pos in range(-number_squares, number_squares + 1):
        barrier_nodes.update(get_surroundings(x_pos, y_pos))
        barrier_nodes.update(get_surroundings(x_pos, y_pos + 2))
        barrier_nodes.update(get_surroundings(x_pos, y_pos - 2))

    return barrier_nodes


# Gets starting node obtained from the localisation algorithm
def get_start_node(loc_pos):
    y = int((line_block_size * loc_pos) // SIZE_SQUARE)
    x = 0

    return x, y


# Translates a path of nodes into a list of cardinal directions
def cardinal_translator(path_in):
    translation = []
    for i in range(len(path_in) - 1):
        # Gets the change in x and y to the new node
        x, y = path_in[i + 1].x - path_in[i].x, path_in[i + 1].y - path_in[i].y

        # Based on the change in x & y, generate the correct cardinal movement direction
        if y == 1:
            cardinal = 'N'
        elif y == -1:
            cardinal = 'S'
        elif x == -1:
            cardinal = 'W'
        else:
            cardinal = 'E'

        translation.append(cardinal)

    return translation


# Translates a list of cardinal directions to robot instruction
def robot_instructions(cardinal_list):
    global orientation, robot_position

    # For every instruction...
    for position in cardinal_list:
        # ... orientate the robot to the correct position...
        rotate(CARDINALS_MAP[position])
        orientation = position

        # ... and move forward one grid space
        move(distance=SIZE_SQUARE)

        # Based on the robots orientation, the correct new position is generated
        if orientation == 'N':
            robot_position = (robot_position[0], robot_position[1] + 1)
        elif orientation == 'E':
            robot_position = (robot_position[0] + 1, robot_position[1])
        elif orientation == 'S':
            robot_position = (robot_position[0], robot_position[1] - 1)
        elif orientation == 'W':
            robot_position = (robot_position[0] - 1, robot_position[1])

        # Updates the number of obstacles according to the new environment scan
        number_objects = len(obstacles)
        obstacles.update(scan_env(robot_position[0], robot_position[1]))

        # If new objects are detected, the instructions are stopped
        if len(obstacles) != number_objects:
            return


# Creates the 4 adjacent nodes to the node passed as an argument
def generate_neighbours(current_node):
    cardinal = 'N'

    # Checks which orientation would the robot be in that node
    if current_node.parent is not None:
        parent = current_node.parent
        x, y = current_node.x - parent.x, current_node.y - parent.y

        # Based on the change in x & y, generate the correct cardinal movement direction
        if y == 1:
            cardinal = 'N'
        elif y == -1:
            cardinal = 'S'
        elif x == -1:
            cardinal = 'W'
        else:
            cardinal = 'E'

    # Expands the node penalising rotation before moving
    if cardinal == 'N':
        north = Node(current_node.x, current_node.y + 1, current_node.g + 1, current_node)
        south = Node(current_node.x, current_node.y - 1, current_node.g + 2, current_node)
        west = Node(current_node.x - 1, current_node.y, current_node.g + 2, current_node)
        east = Node(current_node.x + 1, current_node.y, current_node.g + 2, current_node)
    elif cardinal == 'S':
        north = Node(current_node.x, current_node.y + 1, current_node.g + 2, current_node)
        south = Node(current_node.x, current_node.y - 1, current_node.g + 1, current_node)
        west = Node(current_node.x - 1, current_node.y, current_node.g + 2, current_node)
        east = Node(current_node.x + 1, current_node.y, current_node.g + 2, current_node)
    elif cardinal == 'W':
        north = Node(current_node.x, current_node.y + 1, current_node.g + 2, current_node)
        south = Node(current_node.x, current_node.y - 1, current_node.g + 2, current_node)
        west = Node(current_node.x - 1, current_node.y, current_node.g + 1, current_node)
        east = Node(current_node.x + 1, current_node.y, current_node.g + 2, current_node)
    elif cardinal == 'E':
        north = Node(current_node.x, current_node.y + 1, current_node.g + 2, current_node)
        south = Node(current_node.x, current_node.y - 1, current_node.g + 2, current_node)
        west = Node(current_node.x - 1, current_node.y, current_node.g + 2, current_node)
        east = Node(current_node.x + 1, current_node.y, current_node.g + 1, current_node)

    # Do not duplicate the parent node
    return list({north, west, south, east}.difference({current_node.parent}))


# Selects the next node in the open list
def select_next(open_list):
    return min(open_list)


# Updates the open list with newly generated nodes. If these nodes are
# already in the open list but they now have a smaller f value, we update them
def add_open_list(nodes_list, closed_list, open_list):
    for node in nodes_list:
        if node not in closed_list:
            if node in open_list:
                for i in range(len(open_list)):
                    if node == open_list[i] and node < open_list[i]:
                        open_list[i] = node
                        break
            else:
                open_list.append(node)


# Task 2: Uses A* to return a path of nodes from the initial position to the goal
def a_star(initial_in, goal_in, obstacles_in):
    current_node = initial_in
    closed_list = [initial_in] + list(obstacles_in)
    open_list = []
    add_open_list(generate_neighbours(current_node), closed_list, open_list)

    # Applies A* to expand the open list till the goal is found or the list is empty
    while len(open_list) > 0:
        current_node = select_next(open_list)

        if goal_in in open_list:
            break

        open_list.remove(current_node)
        closed_list.append(current_node)
        add_open_list(generate_neighbours(current_node), closed_list, open_list)

    # Checks if goal is in open list and then assigns to last_node
    last_node = None
    for each in open_list:
        if each == goal_in:
            last_node = each
            break

    # Computes the path from the goal to the start node
    path_return = []
    while last_node is not None:
        path_return.append(last_node)
        last_node = last_node.parent

    path_return.reverse()
    return path_return


# Scans for new obstacles in the environment and assigns them a grid position
def scan_env(current_x, current_y):
    global orientation
    new_obstacles = set()
    for i in range(len(laser_readings)):
        # If an object is detected within the range...
        if laser_readings[i] < 2.5:
            h = laser_readings[i]

            # ... calculate grid position relative to robot
            x = -h * math.sin(math.radians(i))
            x_cells = math.floor(x / SIZE_SQUARE)

            y = h * math.cos(math.radians(i))
            y_cells = math.floor(y / SIZE_SQUARE)

            # Based on the robot's orientation, calculate the objects grid position relative to the map
            x_node, y_node = 0, 0
            if orientation == 'N':
                x_node = x_cells + current_x
                y_node = y_cells + current_y

            elif orientation == 'E':
                x_node = current_x + y_cells
                y_node = current_y - x_cells

            elif orientation == 'W':
                x_node = current_x - y_cells
                y_node = current_y + x_cells

            elif orientation == 'S':
                x_node = current_x - x_cells
                y_node = current_y - y_cells

            new_obstacles.update(get_surroundings(x_node, y_node))

    return new_obstacles


# A* using real-time obstacle detection
def online_a_star(goal_in):
    global robot_position, obstacles

    set_goal(goal_in[0], goal_in[1])

    # Gets the new obstacles
    obstacles.update(scan_env(robot_position[0], robot_position[1]))
    obstacles.update(get_barrier_nodes())

    # Keeps moving until at goal
    while robot_position != goal_in:
        initial_in = Node(robot_position[0], robot_position[1], 0)
        path = a_star(initial_in, goal_in, obstacles)
        cardinals = cardinal_translator(path)
        robot_instructions(cardinals)


############################################################
#                   PARKING METHODS                        #
############################################################

# Referenced in miscallaneous, I proposed this library I was familiar with.
# Plays a sound when the robot is bumped
def play_oof():
    try:
        playsound.playsound('../oof.mp3')
    except playsound.PlaysoundException:
        print('Sound not found')


# Detects the corresponding color in the parking spot
def parking_colours():
    global colour_read
    if 170 < light_value < 171:
        colour_read = 'yellow'
    elif 144 < light_value < 145:
        colour_read = 'purple'
    else:
        colour_read = 'not defined'
    print('Colour read: {} '.format(colour_read))


# I greatly comtributed to this parking algorithm, making sure
# the robot would not collide with the walls.
# Task 3: parking algorithm
def parking():
    global orientation

    # Rotates towards the parking spot
    rotate(math.radians(45))

    # Moves the robot until the bumper sensor is triggered
    # Every 25 steps the orientation is corrected
    iteration = 0
    while not bumped:
        left = min(laser_readings[0:90])
        right = min(laser_readings[-90:])

        # Re-centres according to laser sensor
        # Prevents bumping into walls of parking space
        if iteration % 10 == 0:
            if abs(left-right) < 0.05:
                rotate(math.radians(45))
            elif left > right:
                rotate(math.radians(45 + 10*(left/right)))
            elif right > left:
                rotate(math.radians(45 - 10*(right/left)))

        set_velocity(x=WAFFLE_SPEED)
        time.sleep(0.1)

        iteration += 1

    play_oof()
    set_velocity()
    parking_colours()

    # Reverse
    rotate(math.radians(45))
    move(distance=-1.05)

    # Orientates south
    rotate(-1.571)
    orientation = 'S'


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    signal(SIGINT, handler)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Updates sensor values
    listener()
    time.sleep(2)

    # Task 1: localisation
    line_position = localise()
    robot_position = get_start_node(line_position)

    # Task 2: A*
    online_a_star(goal_parking)

    # Task 3: parking
    parking()

    # Task 4: A* to base
    online_a_star(return_waypoint_coord_1)
    online_a_star((0, 0))

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
