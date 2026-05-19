
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # Between X(3) and X(4): Rotate 45, Move forward 2m, Rotate 45
    angle34 = math.radians(45.0)
    
    rot1_45 = gtsam.Pose2(0.0, 0.0, angle34)
    move_2m = gtsam.Pose2(2.0, 0.0, 0.0)
    rot2_45 = gtsam.Pose2(0.0, 0.0, angle34)
    tot34 = rot1_45.compose(move_2m).compose(rot2_45)
    graph.add(gtsam.BetweenFactorPose2(X(3),X(4), tot34, ODOMETRY_NOISE))
    initial_estimate.insert(X(4), gtsam.Pose2(5.50, 1.50, 1.65))
    
    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)

    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    
    return graph, initial_estimate