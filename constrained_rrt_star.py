import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import scipy.special
from pyclothoids import Clothoid

# Pose has X, Y, and theta
class Pose:
    def __init__(self, x, y , theta):
        self.x = x
        self.y = y
        self.theta = theta # need to add boundary check for theta later
        self.orientation = np.array([np.cos(theta), np.sin(theta)])

    def display(self):
        plt.quiver(self.x, self.y, np.cos(self.theta), np.sin(self.theta))
        #plt.show()  


def ExtendClothoid(p : Pose , L : float , k : float):
    clothoid1 = Clothoid.StandardParams(p.x, p.y, p.theta, 0, k , L)
    p_end = Pose(clothoid1.XEnd, clothoid1.YEnd, clothoid1.ThetaEnd)
    plt.plot(*clothoid1.SampleXY(500))
    plt.show()
    return p_end

def angle_between(v1, v2):
    # Compute angle between vectors v1 and v2
    angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    # Ensure angle is within range [-pi, pi]
    if angle <= -np.pi:
        angle += 2 * np.pi
    elif angle > np.pi:
        angle -= 2 * np.pi
    return angle

def NodeCost(p1 : Pose, p2 : Pose):
    
    baseline_vector = np.array([p2.x - p1.x, p2.y - p1.y])

    # Compute vectors representing orientations of P1 and P2 relative to baseline
    orientation1_vector = np.array([np.cos(p1.theta), np.sin(p1.theta)])
    orientation2_vector = np.array([np.cos(p2.theta), np.sin(p2.theta)])

    # Compute angles between baseline and P1, P2 vectors
    angle1 = angle_between(baseline_vector, orientation1_vector)
    angle2 = angle_between(baseline_vector, orientation2_vector)

    #
   
    if np.sign(angle1) * np.sign(angle2) < 0:
            
        print("Case 1")
        
        #Finding the point of intersection
        m1 , c1 = np.tan(p1.theta) , (p1.y - np.tan(p1.theta)*p1.x)
        m2 , c2 = np.tan(p2.theta) , (p2.y - np.tan(p2.theta)*p2.x)
        Q_x = (c2 - c1) / (m1 - m2)
        Q_y = m1 * Q_x + c1

        #Length of P1Q
        P1Q_length = np.sqrt((Q_x - p1.x) ** 2 + (Q_y - p1.y) ** 2)

        #Direction vector from Q to P2
        dir_vector = np.array([p2.x - Q_x,  p2.y - Q_y])
        dir_angle = np.arctan2(dir_vector[1], dir_vector[0])

        #Length of P2Q
        P2Q_length = np.sqrt((Q_x - p2.x) ** 2 + (Q_y - p2.y) ** 2)

        print(P1Q_length, P2Q_length)

        if(P1Q_length < P2Q_length):
            scaling_factor = P1Q_length / P2Q_length
        else:
             scaling_factor = P2Q_length / P1Q_length

        P3_x = Q_x + scaling_factor * dir_vector[0]
        P3_y = Q_y + scaling_factor * dir_vector[1]

        #Length of P3Q = P1Q
        P3Q_length = np.sqrt((Q_x - P3_x) ** 2 + (Q_y - P3_y) ** 2)

        P3P2_length = np.sqrt((p2.x - P3_x) ** 2 + (p2.y - P3_y) ** 2)
        p3 = Pose(P3_x, P3_y, dir_angle)
        clothoid1 = Clothoid.G1Hermite(p1.x, p1.y, p1.theta, p3.x, p3.y, p3.theta)
        plt.plot([p3.x, p2.x], [p3.y, p2.y], 'r-')
        plt.plot([p1.x, p2.x], [p1.y, p2.y], 'g-')
        plt.plot(Q_x, Q_y, 'o')
        plt.plot(*clothoid1.SampleXY(500))
        plt.axis('equal')
        plt.grid(True)
        p1.display()
        p2.display()
        p3.display()
        plt.show()

        total_cost = clothoid1.length + P3P2_length
        print(f"Clothoid curvature (start): {clothoid1.KappaStart} , Clothoid curvature (end) : {clothoid1.KappaEnd}")
        return total_cost

    else:
        print("Case 2")
        N_x = (p1.x + p2.x)/2
        N_y = (p1.y + p2.y)/2
        N_theta = -(p1.theta + p2.theta)/2
        N = Pose(N_x, N_y, N_theta)
        cost1 = NodeCost(p1, N)
        cost2 = NodeCost(N, p2)
        total_cost = cost1 + cost2
        return total_cost


def Steer(p_from : Pose, x, L , K):
    d_min = np.inf
    k_opt = None
    p_new = None
    for k in K:
        p = ExtendClothoid(p_from, L, k)
        d = np.sqrt((p.x - x[0])**2 + (p.y - x[1])**2)
        if (d < d_min):
            d_min = d
            k_opt = k 
            p_new = p
    
    return p_new

p1 = Pose(0,0, np.radians(30))
p2 = Pose(5,0, np.radians(45))
cost = NodeCost(p1, p2)
print(f"Cost : {cost}")



        