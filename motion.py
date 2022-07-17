import numpy as np
from matplotlib import pyplot as plt


## Init time
t_start = 0
dt = 0.1
t_final = 10
timeVect = np.arange(t_start,t_final,dt)
N = int((t_final-t_start)/dt)

## Init param
s_prev = [0,0]
v_prev = [0,0]
a_prev = [0,0]
fric_coef = 0.2
g = 9.81
mass = 1 # kg
angle = 0
maxForce = 150
F_drag = [0,0]

## Drag param
rho = 1.225      # air density
Cd = 0.25        # drag coef
Area = 0.06      # cross section

## Allocate
a = np.zeros((N,2))
v = np.zeros((N,2))
s = np.zeros((N,2))
p = np.zeros((N,2))
Fxy = np.zeros((N,2))
Force = np.zeros((N,2))
stepForce = np.zeros((N,2))

## Target
# target = [5,2]
target = np.zeros((N,2))
y_time = np.linspace(t_start, t_final, N)
x_sin = np.sin(y_time)
for i in range(N):
    target[i,0] = y_time[i]
    target[i,1] = x_sin[i]

## PID
Kp = 4 
Ki = 1.5 
Kd = 3.2 
error = np.zeros((N,2))
error_prev = [0,0]
int_error = [0,0]
int_error_prev = [0,0]
d_error = [0,0]


def motion():

    for i in range(N):

        if i != 0:
            error[i,0] = target[i,0] - s_prev[0]
            error[i,1] = target[i,1] - s_prev[1]

            int_error[0] = int_error_prev[0] + (error_prev[0] + error[i,0])/2 * dt
            int_error[1] = int_error_prev[1] + (error_prev[1] + error[i,1])/2 * dt

            d_error[0] = (error[i,0] - error_prev[0])/dt
            d_error[1] = (error[i,1] - error_prev[1])/dt

            stepForce[i,0] = Kp * error[i,0] + Ki * int_error[0] + Kd * d_error[0]
            stepForce[i,1] = Kp * error[i,1] + Ki * int_error[1] + Kd * d_error[1]

            if stepForce[i,0] > maxForce:
                stepForce[i,0] = maxForce
            elif stepForce[i,0] < -maxForce:
                stepForce[i,0] = -maxForce
            if stepForce[i,1] > maxForce:
                stepForce[i,1] = maxForce
            elif stepForce[i,1] < -maxForce:
                stepForce[i,1] = -maxForce

            error_prev[0] = error[i,0]
            error_prev[1] = error[i,1]
            int_error_prev[0] = int_error[0]
            int_error_prev[1] = int_error[1]

        # Force balance
        F_fric = mass * 9.81 * fric_coef
        F_drag[0] = 0.5 * rho * Area * Cd * v_prev[0]**2
        F_drag[1] = 0.5 * rho * Area * Cd * v_prev[1]**2
        Force[i,0] = stepForce[i,0] - F_fric - F_drag[0]
        Force[i,1] = stepForce[i,1] - F_fric - F_drag[1]

        Fxy[i,0] = Force[i,0]
        Fxy[i,1] = Force[i,1]

        a[i,0] = Fxy[i,0] / mass
        a[i,1] = Fxy[i,1] / mass

        v[i,0] = v_prev[0] + (a_prev[0] + a[i,0])/2 * dt
        v[i,1] = v_prev[1] + (a_prev[1] + a[i,1])/2 * dt

        s[i,0] = s_prev[0] + (v_prev[0] + v[i,0])/2 * dt
        s[i,1] = s_prev[1] + (v_prev[1] + v[i,1])/2 * dt

        s_prev[0] = s[i,0]
        s_prev[1] = s[i,1]
        v_prev[0] = v[i,0]
        v_prev[1] = v[i,1]
        a_prev[0] = a[i,0]
        a_prev[1] = a[i,1]
        
    return a,v,s,Force


a,v,s,Force = motion()

plt.figure()
plt.subplot(2,2,1)
plt.plot(timeVect, s[:,0], 'g', label="Position")
plt.plot(timeVect, target[:,0], 'k--', label="Set Point")
plt.ylabel("X Position [m]")
plt.legend(loc="upper right")
plt.subplot(2,2,2)
plt.plot(timeVect,Force[:,0])
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.subplot(2,2,3)
plt.plot(timeVect, s[:,1], 'g', label="Position")
plt.plot(timeVect, target[:,1], 'k--', label="Set Point")
plt.ylabel("Y Position [m]")
plt.legend(loc="upper right")
plt.subplot(2,2,4)
plt.plot(timeVect,Force[:,1])
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.show()

plt.figure()
plt.plot(target[:,0], target[:,1], 'b', label="Target", alpha=0.5)
plt.plot(s[:,0], s[:,1], 'g', label="Position")
plt.legend(loc="upper right")
plt.xlabel("X position [m]")
plt.ylabel("Y position [m]")
plt.show()

plt.figure()
plt.plot(timeVect, error[:,0], 'g', label="Error X")
plt.plot(timeVect, error[:,1], 'b', label="Error Y")
plt.legend(loc="upper right")
plt.ylabel("Error [m]")
plt.xlabel("Time [s]")
plt.show()



