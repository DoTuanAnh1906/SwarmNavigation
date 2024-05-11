import numpy as np

NP_ZERO     = np.zeros(2)   # Zero vector
DT          = 0.1           # Time step
E           = 0.1           # Acceptable error
R_ROBOT     = 0.2           # Robot radius
SS_RANGE    = 2.0           # Sensing range
COLLISION_RANGE = 1.0       # Collision range
WALL_RANGE  = 1.0           # Wall range
OFFSET      = 2             # Random range

W_MTG       = 2.0           # Weight for move to goal
W_AC        = 1.0           # Weight for avoid collision
W_AO        = 0.7           # Weight for avoid obstacle
W_WF        = 2.5           # Weight for wall following
W_C         = 0.5           # Weight for cohesion

START_POS   = np.zeros(2)   # Start position
GOAL_POS    = np.array([10, 10]) # Goal position
OBS         = np.array([[4, 0], [10, 2], [6, 10], [2, 8]])    # Obstacle positions

N           = 10            # Number of robots
NUM_STEP    = 1500          # Limit number of steps