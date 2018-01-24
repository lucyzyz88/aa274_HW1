import numpy as np
import math
import scikits.bvp_solver
import matplotlib.pyplot as plt


def q1_ode_fun(tau, z):
	# z = [x, y, th, px, py, pth, r]
    # Code in the BVP ODEs
    x_d = z[6] *(-1) * (z[3]*np.cos(z[2]) + z[4]*np.sin(z[2]) )/2 * np.cos(z[2])
    y_d = z[6] *(-1) * (z[3]*np.cos(z[2]) + z[4]*np.sin(z[2]) )/2 * np.sin(z[2])
    th_d = z[6] * (-1) * z[5]/2
    px_d = 0
    py_d = 0
    pth_d = (-1) * z[6] * (z[3]*np.cos(z[2]) + z[4]*np.sin(z[2])) * (z[3]*np.sin(z[2]) - z[4]*np.cos(z[2]))/2
    #Seven Variables, Seven Equations, the last one is r' = 0 because tf is a constant. tau is not an unknown variable
    #everything is dependent on tau.
    r_d = 0
    return np.array([x_d,y_d,th_d,px_d,py_d,pth_d,r_d])

def q1_bc_fun(za, zb):
	# za = z at time 0
	# 
    # lambda
    lambda_test = 1

    # goal pose
    x_g = 5
    y_g = 5
    th_g = -np.pi/2.0
    xf = [x_g, y_g, th_g]

    # initial pose
    x0 = [0, 0, -np.pi/2.0]
    # start conditions on the left has three equations
    bc_start = np.array([za[0]-x0[0], za[1]-x0[1], za[2]-x0[2]])
    # end conditions on the right has four equations
    bc_end = np.array([zb[0]-xf[0], zb[1]-xf[1], zb[2]-xf[2], lambda_test-(zb[4]**2)/4-(zb[5]**2)/4])
    return bc_start, bc_end

    # Code boundary condition residuals
    
#Define solver state: z = [x, y, th, px, py, pth, r]
#Side Note: The sizes of these arrays must add up to the total number of ODEs plus the number of unknown parameters.

problem = scikits.bvp_solver.ProblemDefinition(num_ODE = 7,
                                      num_parameters = 0,
                                      num_left_boundary_conditions = 3,
                                      boundary_points = (0, 1),
                                      function = q1_ode_fun,
                                      boundary_conditions = q1_bc_fun)

soln = scikits.bvp_solver.solve(problem, solution_guess = (1,1, -np.pi/2,-1,-1,5,10
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
# Weird multiplication
V = (-1./2) * (z[:,3] * np.cos(z[:,2]) +z[:,4] * np.sin(z[:,2]))
om =(-1./2) * z[:,5]

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
