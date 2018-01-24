import numpy as np
from numpy import linalg
from P3_pose_stabilization import ctrl_pose

def ctrl_traj(x,y,th,ctrl_prev,x_d,y_d,xd_d,yd_d,xdd_d,ydd_d,x_g,y_g,th_g):
    # (x,y,th): current state
    # ctrl_prev: previous control input (V,om)
    # (x_d, y_d): desired position
    # (xd_d, yd_d): desired velocity
    # (xdd_d, ydd_d): desired acceleration
    # (x_g,y_g,th_g): desired final state

    # Timestep
    dt = 0.005

    # Gains
    kpx = 1.5 #...TODO...#
    kpy = 1
    kdx = 1
    kdy = 1

    # Define control inputs (V,om) - without saturation constraints
    # Switch to pose controller once "close" enough, i.e., when
    # the robot is within 0.5m of the goal xy position.
    #...TODO...#
    # If close enough
    if (((x-x_g)**2 + (y-y_g)**2)**0.5) <= 0.5:
        [V,om] = ctrl_pose(x,y,th,x_g,y_g,th_g)
    else:
        #Virtual Control Law
        u1 = xdd_d + kpx*(x_d - x) + kdx*(xd_d - ctrl_prev[0]*np.cos(th))
        u2 = ydd_d + kpy*(y_d - y) + kdy*(yd_d - ctrl_prev[0]*np.sin(th))
        V = ctrl_prev[0] + (((u1 + ctrl_prev[1]*ctrl_prev[0]*np.sin(th))**2 + (u2 - ctrl_prev[1]*ctrl_prev[0]*np.cos(th))**2)**0.5) * dt
        om = 1/V * (u2*np.cos(th) - u1*np.sin(th))

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
