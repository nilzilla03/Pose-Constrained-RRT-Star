import numpy as np

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.orientation = np.array([np.cos(theta), np.sin(theta)])

def check_case_1(p1: Pose, p2: Pose):
    # Compute baseline orientation
    baseline_orientation = np.arctan2(p2.y - p1.y, p2.x - p1.x)

    # Compute relative orientations
    orientation1 = np.arctan2(p1.orientation[1], p1.orientation[0])
    orientation2 = np.arctan2(p2.orientation[1], p2.orientation[0])
    
    # Compute angular difference
    angle_diff = np.abs(orientation1 - orientation2)

    # Check if poses satisfy Case 1
    if np.sign(np.cos(p1.theta - baseline_orientation)) != np.sign(np.cos(p2.theta - baseline_orientation)):
        return True
    else:
        return False

# Example poses
p1 = Pose(5, 0, np.radians(30))
p2 = Pose(0, 3, np.radians(30))

# Check if poses satisfy Case 1
if check_case_1(p1, p2):
    print("Poses satisfy Case 1")
else:
    print("Poses do not satisfy Case 1")








