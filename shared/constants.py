import math

# +------------------+
# | robot dimensions |
# +------------------+

# in centimetres
ROBOT_WIDTH = 25
ROBOT_HEIGHT = 28
ROBOT_ACTUAL_WIDTH = 18.8 # defined in lecture notes
ROBOT_ACTUAL_HEIGHT = 23 # defined in lecture notes
ROBOT_VERT_OFFSET = (ROBOT_HEIGHT - ROBOT_ACTUAL_HEIGHT) / 2

# +--------------------+
# | collision checking |
# +--------------------+
"""
  # TODO: Measureable
  WPS: Waypoints
  Measure using Bottom Left of Robot
"""

# Measure (Using Bottom Left of Robot)

# usually take last path point as the final displacement

# OUR MEASUREMENTS INDOOR V1 (OURS V1)
# WPS_FL_IN = [(-2,10, 0.3142), (-8,15.5, 0.6283), (-15,21, 0.9425), (-22,23, 1.2566), (-28,27, 1.5707963267948966)]
# WPS_FR_IN = [(4,25, -0.3142), (11,31.5, -0.6283), (20,37, -0.9425), (30.5,43, -1.2566), (40.5,44.5, -1.5707963267948966)]
# WPS_BR_IN = [(5.3, -10, 0.3142), (15.1, -19.7, 0.6283), (27.6, -25.8, 0.9425), (34.4, -28.2, 1.2566), (41.1, -30.5, 1.5707963267948966)]
# WPS_BL_IN = [(-1, -3.8, -0.3142), (-4.7, -7.0, -0.6283), (-8.5, -8.3, -0.9425), (-10, -9, -1.2566), (-17.5, -11, -1.5707963267948966)]

# OUR MEASUREMENTS INDOOR V2 (FRIDAY MORNING 18 oct at lab indoor)
# WPS_FL_IN = [(-3,5, 0.3927), (-9,23.5, 0.7854), (-16,26, 1.1781), (-19,26.5, 1.5707963267948966)]
# WPS_FR_IN = [(0.5,16, -0.3142), (4.5,29, -0.6283), (9,34.5, -0.9425), (37,49.5, -1.2566), (44,50, -1.5707963267948966)]
# WPS_BR_IN = [(2,-7, 0.3142), (9,-16, 0.6283), (17.5,-25, 0.9425), (30,-32.5, 1.2566), (43,-35, 1.5707963267948966)]
# WPS_BL_IN = [(-1.5,-2, -0.3927), (-4,-4, -0.7854), (-7,-4.5, -1.1781), (-10.5,-5, -1.5707963267948966)]

# OUR MEASUREMENTS INDOOR V3 (Saturday morning 19 october at arc)
WPS_FL_IN = [(-0.5, 4, 0.3927), (-1.5, 8, 0.7854), (-3.5, 10.5, 1.1781), (-6.5, 11, 1.5707963267948966)]
WPS_FR_IN = [(5.5, 12.5, -0.3142), (13, 16, -0.6283), (20.5, 38.5, -0.9425), (34, 51, -1.2566), (54, 55, -1.5707963267948966)]
WPS_BR_IN = [(9, -20.5, 0.3142), (16, -34, 0.6283), (23.5, -46, 0.9425), (47.5, -55, 1.2566), (55, -56, 1.5707963267948966)]
WPS_BL_IN = [(-0.5,-3.5, -0.3927), (-3.5, -6, -0.7854), (-8, -6.25, -1.1781), (-12, -6.5, -1.5707963267948966)]

WPS_FL = WPS_FL_IN
WPS_FR = WPS_FR_IN
WPS_BR = WPS_BR_IN
WPS_BL = WPS_BL_IN

BUFFER = 5.01

FL_A, FL_B = (19.8, 25.3)
FR_A, FR_B = (44.6, 51.3 )
BR_A, BR_B = (48.9, 43.1 )
BL_A, BL_B = (29, 22.5)

# Our algorithms move in 5cm increments
_DIST_STR = 5
DIST_BW = _DIST_STR
DIST_FW = _DIST_STR

_circum = lambda a, b: math.pi * ( 3*(a+b) - math.sqrt( (3*a + b) * (a + 3*b) ) )
# x displacement, y displacement, arc length
DIST_FL = WPS_FL[-1][0], WPS_FL[-1][1], _circum(FL_A, FL_B)/4
DIST_FR = WPS_FR[-1][0], WPS_FR[-1][1], _circum(FR_A, FR_B)/4
DIST_BL = WPS_BL[-1][0], WPS_BL[-1][1], _circum(BL_A, BL_B)/4
DIST_BR = WPS_BR[-1][0], WPS_BR[-1][1], _circum(BR_A, BR_B)/4

PENALTY_STOP = 30
MAX_THETA_ERR = math.pi / 12 # PI = 180 degrees, 180/12 = 15 degrees
MAX_X_ERR = 5, 5  # Left, Right
MAX_Y_ERR = 5, 5 # Up, Down

# +---------------------+
# | obstacle dimensions |
# +---------------------+

OBSTACLE_WIDTH = 10
IMG_THICKNESS = 2
EDGE_ERR = 0.1 #TODO: Edit this

# +--------------------+
# | Priority obstacles |
# +--------------------+
FL_OUTER = 41
FR_OUTER = 54
BL_OUTER = 47
BR_OUTER = 69

BACKWARDS_A_B_MULTIPLIER = 1.5

FL_X_BOUND = [OBSTACLE_WIDTH/2 + FL_A - ROBOT_WIDTH/2 + ROBOT_HEIGHT - ROBOT_VERT_OFFSET, 
              OBSTACLE_WIDTH/2 + ROBOT_WIDTH]

FL_Y_BOUND = [OBSTACLE_WIDTH/2 + FL_OUTER + (FL_B - FL_A) + ROBOT_VERT_OFFSET, 
              OBSTACLE_WIDTH/2]

FR_X_BOUND = [OBSTACLE_WIDTH/2, 
              OBSTACLE_WIDTH/2 + FR_A + ROBOT_WIDTH/2 + ROBOT_HEIGHT - ROBOT_VERT_OFFSET]

FR_Y_BOUND = [OBSTACLE_WIDTH/2 + FR_OUTER + (FR_B - FL_A) + ROBOT_VERT_OFFSET, 
              OBSTACLE_WIDTH/2]

BL_X_BOUND = [OBSTACLE_WIDTH/2 + (BL_A*BACKWARDS_A_B_MULTIPLIER) - ROBOT_WIDTH/2 + ROBOT_VERT_OFFSET, 
              OBSTACLE_WIDTH/2 + BL_OUTER - ((BL_A*BACKWARDS_A_B_MULTIPLIER) - ROBOT_WIDTH/2)] 

BL_Y_BOUND = [OBSTACLE_WIDTH/2 + ROBOT_HEIGHT, 
              OBSTACLE_WIDTH/2 + (BL_B*BACKWARDS_A_B_MULTIPLIER) + ROBOT_WIDTH/2 - ROBOT_VERT_OFFSET]

BR_X_BOUND = [OBSTACLE_WIDTH/2 + BR_OUTER - (BR_A*BACKWARDS_A_B_MULTIPLIER) - ROBOT_WIDTH/2, 
              OBSTACLE_WIDTH/2 + (BR_A*BACKWARDS_A_B_MULTIPLIER) + ROBOT_WIDTH/2 + ROBOT_VERT_OFFSET]

BR_Y_BOUND = [OBSTACLE_WIDTH/2 + ROBOT_HEIGHT, 
              OBSTACLE_WIDTH/2 + (BR_B*BACKWARDS_A_B_MULTIPLIER) + ROBOT_WIDTH/2 - ROBOT_VERT_OFFSET]

ROBOT_MIN_CAMERA_DIST = 25


# +----------------+
# | map dimensions |
# +----------------+

MAP_WIDTH = 200 
MAP_HEIGHT = 200
GRID_WIDTH = 5 # for display on simulator
GRID_COORD = _DIST_STR # for cell grid (coords) 5. Max value < 1.5* min(DIST_BL, DIST_BR, ... DIST_FW)
GRID_THETA = 15 # for cell grid (theta) 15

