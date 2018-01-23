import numpy as np
import math
from numpy import linalg
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

# Constants
t_f = 15
V_max = 0.5
om_max = 1

# Initial conditions
x_0 = 0
y_0 = 0
V_0 = V_max
th_0 = -np.pi/2

# Final conditions

x_f = 5
y_f = 5
V_f = V_max
th_f = -np.pi/2

# Solve Linear equations:
#...TODO...#
#Solve the X coefficients#
X_a = np.array([[1,0,0,0],[0,1,0,0],[1,t_f,t_f**2,t_f**3],[0,1,2*t_f,3*t_f**2]])
X_b = np.array([0,0,5,0])
X_coeff = linalg.solve(X_a,X_b)
print(X_coeff)
#Solve the Y coefficients#
Y_a = np.array([[1,0,0,0],[0,1,0,0],[1,t_f,t_f**2,t_f**3],[0,1,2*t_f,3*t_f**2]])
Y_b = np.array([0,-0.5,5,-0.5])
Y_coeff = linalg.solve(Y_a,Y_b)
print(Y_coeff)
# Compute traj=X_coeff[0]*t
dt = 0.005
N = int(t_f/dt)
t = dt*np.array(range(N+1)) # t[0],....,t[N]
t = t.T
data = np.zeros((N+1,9))
print(X_coeff[0])
# Compute trajectory, store in data, format: [x,y,th,V,om,xd,yd,xdd,ydd]
#...TODO...#
#x stored in data#
x = X_coeff[0] + X_coeff[1] * t + X_coeff[2] * t*t + X_coeff[3] * np.power(t,3) 
data[:,0] = x
#y stored in data#

y = Y_coeff[0] + Y_coeff[1] * t + Y_coeff[2] * t*t + Y_coeff[3] * np.power(t,3)
data[:,1] = y

#xd stored in data
xd = X_coeff[1] + 2*X_coeff[2]*t + 3*X_coeff[3]*t*t
data[:,5] = xd
#yd stored in data
yd = Y_coeff[1] + 2*Y_coeff[2]*t + 3*Y_coeff[3]*t*t
data[:,6] = yd
#print(xd)
#print(yd)
#th stored in data#
th = np.zeros(N+1)
for j in range(np.size(th)):
	if (xd[j] == 0):
		th[j] = -np.pi/2
	else:
		th[j] = np.arctan2(yd[j],xd[j])
data[:,2] = th
print(th)
#print(th)
#V stored in data
V = (xd**2+yd**2)**(0.5)
data[:,3] = V
print(V)
#xdd stored in data
xdd = 2 * X_coeff[2] + 6 * t * X_coeff[3]
data[:,7] = xdd
#Ydd stored in data
ydd = 2 * Y_coeff[2] + 6 * t * Y_coeff[3]
data[:,8] = ydd

#om stored in data
om = np.gradient(th,dt)
data[:,4] = om


## Re-scaling - Compute scaled trajectory quantities at the N points along the geometric path above
# Compute arc-length s as a function of t (HINT: use the function cumtrapz) integration of small segments
# s = ...TODO...#
ds = V
s = cumtrapz(ds,t,initial = 0)

# Compute V_tilde (HINT: at each timestep V_tilde should be computed as a minimum of
# the original value V, and values required to ensure both constraints are satisfied)
# V_tilde = ...TODO...#
V_tilde = np.zeros(N+1)
V_tildeom = np.zeros(N+1)
for i in range(np.size(V_tilde)):
	if om[i] != 0:
		V_tildeom[i] = V[i]/abs(om[i])
	else:
		V_tildeom[i] = math.inf
	V_tilde[i] = min(0.5, V[i], V_tildeom[i])

# for i in range(np.size(V_tilde)):
# 	if V[i] > 0.5:
# 		V_tilde[i] = 0.5
# 	else:
# 		V_tilde[i] = V[i]

# Compute tau (HINT: use the function cumtrapz)
# tau = ...TODO...#
dtau = 1/V_tilde
tau = cumtrapz(dtau,s,initial = 0)

# Compute om_tilde
# om_tilde = ...TODO...#
om_tilde = np.zeros(N+1)
om_tilde = om * V_tilde / V


# for i in range(np.size(om_tilde)):
# 	if om_tilde[i] > 1:
# 		om_tilde[i] = 1
# 	elif om_tilde[i] < -1:
# 		om_tilde[i] = -1

# Get new final time
tf_new = tau[-1]

# Generate new uniform time grid
N_new = int(tf_new/dt)
t_new = dt*np.array(range(N_new+1))
t_new = t_new.T

#print(N_new)
# Interpolate for state trajectory
data_scaled = np.zeros((N_new+1,9))
data_scaled[:,0] = np.interp(t_new,tau,data[:,0]) # x
data_scaled[:,1] = np.interp(t_new,tau,data[:,1]) # y
data_scaled[:,2] = np.interp(t_new,tau,data[:,2]) # th
# Interpolate for scaled velocities
data_scaled[:,3] = np.interp(t_new, tau, V_tilde)   # V
data_scaled[:,4] = np.interp(t_new, tau, om_tilde)  # om
# Compute xy velocities
data_scaled[:,5] = data_scaled[:,3]*np.cos(data_scaled[:,2]) # xd
data_scaled[:,6] = data_scaled[:,3]*np.sin(data_scaled[:,2]) # yd
# Compute xy acclerations
data_scaled[:,7] = np.append(np.diff(data_scaled[:,5])/dt,-V_f*data_scaled[-1,4]*np.sin(th_f)) # xdd
data_scaled[:,8] = np.append(np.diff(data_scaled[:,6])/dt, V_f*data_scaled[-1,4]*np.cos(th_f)) # ydd

# Save trajectory data
np.save('traj_data_differential_flatness',data_scaled)

# Plots
plt.rc('font', weight='bold', size=16)

plt.figure()
plt.plot(data_scaled[:,0], data_scaled[:,1],'k-',linewidth=2)
plt.grid('on')
plt.plot(x_0,y_0,'go',markerfacecolor='green',markersize=15)
plt.plot(x_f,y_f,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')

plt.figure()
plt.subplot(2,1,1)
plt.plot(t, data[:,3:5],linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'])
plt.title('Original')

plt.subplot(2,1,2)
plt.plot(t_new,data_scaled[:,3:5],linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'])
plt.title('Scaled')

plt.figure()
plt.plot(t,s,'b-',linewidth=2)
plt.grid('on')
plt.plot(tau,s,'r-',linewidth=2)
plt.xlabel('Time [s]')
plt.ylabel('Arc-length [m]')
plt.legend(['Original', 'Scaled'])

plt.show()
