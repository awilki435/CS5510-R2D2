import matplotlib.pyplot as plt
import numpy as np

def main():
    R = 59.0551/2
    v_r = 0
    v_l = 0
    L = 35
    W = 10
    x = R
    y = 0
    theta = 0 
    deltaT = .1
    time = 0
    circumfranceT = 30
    trajectory = []

    while time < circumfranceT:
        if 0 < x < 2*R and time < 0.5 * circumfranceT:
            v_r = 20
            v_l = (R-W)/(R+W) * v_r
        else:
            v_r = 20
            v_l = (R-W/2)/(R+W/2) * v_r
        trajectory.append((x,y))
        x, y, theta = kinematics(W, v_l, v_r, x, y, theta)
        time = time + deltaT
    
    fig,ax = plt.subplots()
    xvals, yvals = zip(*trajectory)
    ax.plot(xvals,yvals, color="blue")
    path_x, path_y = create_circle(ax, R)
    window_limit = 70
    ax.set_xlim(left=-window_limit, right=window_limit)
    ax.set_ylim(bottom=-window_limit, top=window_limit)
    ax.set_aspect('equal', adjustable='box')
    plt.show()

def kinematics(W, v_l, v_r, x0, y0, theta0, dt=0.1):
    x = x0 + 0.5*(v_l + v_r) * np.cos(theta0) *dt
    y = y0 + 0.5*(v_l + v_r) * np.sin(theta0) *dt
    theta = theta0 + 1/W * (v_r - v_l) * dt
    return x,y,theta

def create_circle(ax, R):
    theta = 0
    x_vals = []
    y_vals = []
    while theta < 2 * np.pi + 0.1:
        y = R * np.sin(theta)
        x = R * np.cos(theta)
        theta = theta + 0.1
        x_vals.append(x)
        y_vals.append(y)
    ax.plot(x_vals,y_vals,color="green")
    return x_vals, y_vals

if __name__ == "__main__":
    main()