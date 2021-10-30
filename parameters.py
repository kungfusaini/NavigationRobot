import math

# I have contributed to the choice of these values
# This mostly was a group work

############################################
#              ROBOT PARAMETERS            #
############################################

# Max linear speed
WAFFLE_MAX_LIN_VEL = 0.24

# Max linear speed used by the waffle
WAFFLE_SPEED = WAFFLE_MAX_LIN_VEL / 3

# Min linear speed
WAFFLE_MIN_LIN_VEL = 0.05

# Max angular speed
WAFFLE_MAX_ANG_VEL = 1.82

# Max angular speed used by the waffle
WAFFLE_ANG_VEL = WAFFLE_MAX_ANG_VEL / 3

############################################
#               MAP PARAMETERS             #
############################################

# Size of each node
SIZE_SQUARE = 0.1

# Probability that action succeeds
ACTION_MODEL_ERROR = 0.985

CARDINALS_MAP = {'N': 1.571, 'W': 3.14, 'S': -1.571, 'E': 0}

# Localisation Line Parameters ########
grey_prob = [0.05, 0.95, 0.95, 0.05, 0.05, 0.95, 0.95, 0.95, 0.05, 0.05, 0.95, 0.95, 0.95, 0.05, 0.95, 0.95, 0.05,
             0.05, 0.95, 0.95, 0.95, 0.05, 0.95, 0.95]
white_prob = [0.95, 0.05, 0.05, 0.95, 0.95, 0.05, 0.05, 0.05, 0.95, 0.95, 0.05, 0.05, 0.05, 0.95, 0.05, 0.05, 0.95,
              0.95, 0.05, 0.05, 0.05, 0.95, 0.05, 0.05]

line_block_size = 0.1

############################################
#          GOALS AND WAYPOINTS             #
############################################

init_map_pos = (2.06825, -5.15689)
goal_map_pos = (1.0, -1.6855)
return_waypoint_0 = (1.0952, -2.02099)
return_waypoint_1 = (3.2608, -2.02099)
goal_parking = (math.floor((goal_map_pos[0] - init_map_pos[0]) / SIZE_SQUARE),
        math.floor((goal_map_pos[1] - init_map_pos[1]) / SIZE_SQUARE))
return_waypoint_coord_0 = (math.floor((return_waypoint_0[0] - init_map_pos[0]) / SIZE_SQUARE),
                           math.floor((return_waypoint_0[1] - init_map_pos[1]) / SIZE_SQUARE))
return_waypoint_coord_1 = (math.floor((return_waypoint_1[0] - init_map_pos[0]) / SIZE_SQUARE),
                           math.floor((return_waypoint_1[1] - init_map_pos[1]) / SIZE_SQUARE))

