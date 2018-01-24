import numpy as np
from utils import wrapToPi

def ctrl_pose(x,y,th,x_g,y_g,th_g):
    #(x,y,th): current state
    #(x_g,y_g,th_g): desired final state

    #Code pose controller
    #...TODO...#

    #Define control inputs (V,om) - without saturation constraints
    #Define Control constants k1,k2,k3
    k1 = 0.8
    k2 = 0.8
    k3 = 0.2
    
    alpha = wrapToPi(np.arctan2((y_g - y),(x_g - x)) - th)
    ro = ((y_g - y)**2 + (x_g - x)**2)**0.5
    delta = wrapToPi(np.arctan2((y_g - y),(x_g - x)) - th_g)



    V = k1 * ro * np.cos(alpha)
    om = k2 * alpha + k1 * np.sinc(alpha/np.pi) * np.cos(alpha) *(alpha + k3*delta)

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
