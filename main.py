import numpy as np
import time
import tkinter as tk


Window_Width = 1000
Window_Height = 600

# Time step
t = 0
dt = 0.02    # seconds

# Init Ball
dx = dy = 0
dxx = dyy = 0
init_pos = 300
init_pos_dummy = 500
position = np.array([init_pos,init_pos,init_pos_dummy,init_pos_dummy])
mass = 0.5
Force = 0
collisionForce = 0
collisionAngle = 0
angle = 0
a_prev = np.zeros(4)
v_prev = np.zeros(4)
s_prev = np.zeros(4)
rho = 1.225     # air density
Cd = 0.25       # drag coef
Area = 0.1      # cross section
fric_coef = 0.1


def create_animation_window():
  Window = tk.Tk()
  Window.title("Motion")

  Window.geometry(f'{Window_Width}x{Window_Height}')
  return Window

def create_animation_canvas(Window):
  canvas = tk.Canvas(Window)
  canvas.configure(bg="White")
  canvas.pack(fill="both", expand=True)
  return canvas

def animate_ball(Window,canvas,radius):
    global position, Force, dx, dy, angle
    global collisionForce, dxx, dyy, collisionAngle

    textVar = tk.StringVar()
    textVar.set("Hello")
    l = tk.Label(Window, textvariable = textVar)
    l.pack()

    ball = canvas.create_oval(
        init_pos-radius,
        init_pos-radius,
        init_pos+radius,
        init_pos+radius,
        fill="Red", outline="Black", width=2)

    ballDummy = canvas.create_oval(
        init_pos_dummy-radius,
        init_pos_dummy-radius,
        init_pos_dummy+radius,
        init_pos_dummy+radius,
        fill="Green", outline="Black", width=2)

    arrow = canvas.create_line(
        init_pos, 
        init_pos, 
        int(position[0]+Force*(0.1*np.cos(angle))), 
        int(position[1]+Force*(0.1*np.sin(angle))), 
        width=6,
        fill='blue',
        arrow=tk.LAST)  

    arrowDummy = canvas.create_line(
        init_pos, 
        init_pos, 
        int(position[2]+collisionForce*(0.1*np.cos(collisionAngle))), 
        int(position[3]+collisionForce*(0.1*np.sin(collisionAngle))), 
        width=6,
        fill='blue',
        arrow=tk.LAST) 

    while True:

        textVar.set(f'Force: {int(Force)} [N] \n Angle: {round(angle,2)} [rad]')

        # Get coord of mouse
        xtarget = Window.winfo_pointerx() - Window.winfo_rootx()
        ytarget = Window.winfo_pointery() - Window.winfo_rooty()

        # Compute angle from mouse
        xDiff = xtarget - position[0]
        yDiff = ytarget - position[1]
        angle = np.arctan2(yDiff,xDiff)

        # Compute force from mouse
        xyDiff = np.sqrt(xDiff**2+yDiff**2)
        Force = xyDiff*100

        # Compute next position
        dx, dy = motion(mass,Force,angle) 

        # Check for boundaries
        if position[0]+dx < 0:
            position[0] = Window_Width - 10
            canvas.coords(ball,position[0]-radius,position[1]-radius,position[0]+radius,position[1]+radius)
            #dx = 0
        elif position[0]+dx > Window_Width:
            position[0] = 10
            canvas.coords(ball,position[0]-radius,position[1]-radius,position[0]+radius,position[1]+radius)
            #dx = 0
        if position[1]+dy < 0:
            position[1] = Window_Height - 10
            canvas.coords(ball,position[0]-radius,position[1]-radius,position[0]+radius,position[1]+radius)
            #dy = 0
        elif position[1]+dy > Window_Height:
            position[1] = 10
            canvas.coords(ball,position[0]-radius,position[1]-radius,position[0]+radius,position[1]+radius)
            #dy = 0

        # Check for boundaries for dummy
        if position[2]+dxx < 0:
            position[2] = Window_Width - 20
            canvas.coords(ballDummy,position[2]-radius,position[3]-radius,position[2]+radius,position[3]+radius)
            #dxx = 0
        elif position[2]+dxx > Window_Width:
            position[2] = 20
            canvas.coords(ballDummy,position[2]-radius,position[3]-radius,position[2]+radius,position[3]+radius)
            #dxx = 0
        if position[3]+dyy < 0:
            position[3] = Window_Height - 20
            canvas.coords(ballDummy,position[2]-radius,position[3]-radius,position[2]+radius,position[3]+radius)
            #dyy = 0
        elif position[3]+dyy > Window_Height:
            position[3] = 20
            canvas.coords(ballDummy,position[2]-radius,position[3]-radius,position[2]+radius,position[3]+radius)
            #dyy = 0

        # Check for collision
        collisionX = position[0] - position[2]
        collisionY = position[1] - position[3]
        collisionVector = np.sqrt(collisionX**2+collisionY**2)
        if collisionVector <= radius:
            collisionAngle = np.arctan2(collisionY,collisionX)
            collisionForce = -collisionVector*2000
        else:
            collisionAngle = 0
            collisionForce = 0
        dxx, dyy = motionDummy(mass,collisionForce,collisionAngle) 

        # Update global position of the ball
        position[0] += dx
        position[1] += dy

        # Update arrow endpoints
        arrow_dx = int(position[0]+Force*0.005*(np.cos(angle)))
        arrow_dy = int(position[1]+Force*0.005*(np.sin(angle)))
        canvas.coords(arrow,position[0],position[1],arrow_dx,arrow_dy)

        # Update dummy
        position[2] += dxx
        position[3] += dyy

        # Update arrow endpoints
        arrow_dxx = int(position[2]+collisionForce*0.005*(np.cos(collisionAngle)))
        arrow_dyy = int(position[3]+collisionForce*0.005*(np.sin(collisionAngle)))
        canvas.coords(arrowDummy,position[2],position[3],arrow_dxx,arrow_dyy)

        # Update ball coord
        canvas.move(ball,dx,dy)
        canvas.move(ballDummy,dxx,dyy)
        
        # Update GUI
        Window.update()
        time.sleep(dt)

        


def motion(mass, Force, angle):
    global t, v_prev, a_prev, fric_coef

    g = 9.81
    a = np.zeros(2)
    v = np.zeros(2)
    s = np.zeros(2)
    Fxy = np.zeros(2)
    
    velocity = np.sqrt(v_prev[0]**2+v_prev[1]**2)
    F_drag = 0.5 * rho * Area * Cd * velocity
    F_fric = mass * 9.81 * fric_coef

    Force -= F_fric - F_drag

    Fxy[0] = Force * np.cos(angle)
    Fxy[1] = Force * np.sin(angle)

    a[0] = Fxy[0] / mass
    a[1] = Fxy[1] / mass

    v[0] = (a_prev[0] + a[0])/2 * dt
    v[1] = (a_prev[1] + a[1])/2 * dt

    s[0] = (v_prev[0] + v[0])/2 * dt
    s[1] = (v_prev[1] + v[1])/2 * dt

    v_prev[0] = v[0]
    v_prev[1] = v[1]
    a_prev[0] = a[0]
    a_prev[1] = a[1]

    return int(s[0]), int(s[1])


def motionDummy(mass, collisionForce, collisionAngle):
    global v_prev, a_prev

    a = np.zeros(2)
    v = np.zeros(2)
    s = np.zeros(2)
    Fxy = np.zeros(2)
    
    Fxy[0] = collisionForce * np.cos(collisionAngle)
    Fxy[1] = collisionForce * np.sin(collisionAngle)

    a[0] = Fxy[0] / mass
    a[1] = Fxy[1] / mass

    v[0] = (a_prev[2] + a[0])/2 * dt
    v[1] = (a_prev[3] + a[1])/2 * dt

    s[0] = (v_prev[2] + v[0])/2 * dt
    s[1] = (v_prev[3] + v[1])/2 * dt

    v_prev[2] = v[0]
    v_prev[3] = v[1]
    a_prev[2] = a[0]
    a_prev[3] = a[1]

    return int(s[0]), int(s[1])



if __name__ == "__main__":

    radius = 16
    Animation_Window = create_animation_window()
    Animation_canvas = create_animation_canvas(Animation_Window)
    animate_ball(Animation_Window,Animation_canvas,radius)