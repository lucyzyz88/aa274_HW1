import numpy as np
import math
import scikits.bvp_solver
import matplotlib.pyplot as plt


def q1_ode_fun(tau, z):

    # Code in the BVP ODEs
    x_d = t_f *(-1) * (z[3]*np.cos(z[2,tau]) + z[4]*np.sin(z[2,tau]) )/2 * np.cos(z[2,tau])
    y_d = t_f *(-1) * (z[3]*np.cos(z[2,tau]) + z[4]*np.sin(z[2,tau]) )/2 * np.sin(z[2,tau])
    th_d = t_f * (-1) * z[5]/2
    px_d = 0
    py_d = 0
    pth_d = (-1) * t_f * (z[3]*np.cos(z[2,tau]) + z[4]*np.sin(z[2,tau])) * (z[3]*np.sin(z[2,tau]) - z[4]*np.cos(z[2,tau]))/2

    return #...TODO...#


def q1_bc_fun(za, zb):

    # lambda
    lambda_test = 1.0

    # goal pose
    x_g = 5
    y_g = 5
    th_g = -np.pi/2.0
    xf = [x_g, y_g, th_g]

    # initial pose
    x0 = [0, 0, -np.pi/2.0]

    # Code boundary condition residuals
    
    return (np.array([za[0] - x0[0]]),
    		np.array([za[1] - x0[1]]),
    		np.array([za[2] - x0[2]]),
    		np.array([zb[0] - xf[0]]),
    		np.array([zb[1] - xf[1]]),
    		np.array([zb[2] - xf[2]]),
            )  #...TODO...#

#Define solver state: z = [x, y, th, px, py, pth, r]
problem = scikits.bvp_solver.ProblemDefinition(num_ODE = 6,
                                      num_parameters = 0,
                                      num_left_boundary_conditions = 3,
                                      boundary_points = (0, 1),
                                      function = q1_ode_fun,
                                      boundary_conditions = q1_bc_fun)

soln = scikits.bvp_solver.solve(problem, solution_guess = (1,1,1,1,1,1,1
                                ))

dt = 0.005

# Test if time is reversed in bvp_solver solution
z_0 = soln(0)
flip = 0
if z_0[-1] < 0:
    t_f = -z_0[-1]
    flip = 1
else:
    t_f = z_0[-1]

t = np.arange(0,t_f,dt)
z = soln(t/t_f)
if flip:
    z[3:7,:] = -z[3:7,:]
z = z.T # solution arranged column-wise

# Recover optimal control histories TO...DO...
V = np.gradient(z_0,dt)/np.cos(z_2)
om = np.gradient(z_2,dt)

V = np.array([V]).T # Convert to 1D column matrices
om = np.array([om]).T

# Save trajectory data (state and controls)
data = np.hstack((z[:,:3],V,om))
np.save('traj_data_optimal_control',data)

# Plots
plt.rc('font', weight='bold', size=16)

plt.figure()
plt.plot(z[:,0], z[:,1],'k-',linewidth=2)
plt.quiver(z[1:-1:200,0],z[1:-1:200,1],np.cos(z[1:-1:200,2]),np.sin(z[1:-1:200,2]))
plt.grid('on')
plt.plot(0,0,'go',markerfacecolor='green',markersize=15)
plt.plot(5,5,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')

plt.figure()
plt.plot(t, V,linewidth=2)
plt.plot(t, om,linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'])

plt.show()
