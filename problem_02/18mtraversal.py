

"""
x_t+1 = = x_t + -vsin(theta) * delta t
y_t+1 = y_t + vcos(theta) * delta t
theta_t+1 = theta_t + (v/L) tan(alpha) * delta t
"""
import matplotlib.pyplot as plt
import numpy as np

def main():
    alpha = 0
    v = 26.2467
    L = 35
    W = 10
    x = 0
    y = 0
    theta = 0 
    R = 59.0551
    deltaT = .5
    time = 0
    circumfranceT = 46.5 * 2
    trajectory = []
    
    while time < circumfranceT:
        trajectory.append((x,y))
        if time < 3.25:
            alpha = np.arctan2(2* L/R)
        else: 
            alpha = np.arctan2(L,R)
        x =  x + v * np.cos(theta) * deltaT
        y = y + v * np.sin(theta) * deltaT
        theta = theta + (v/L) * np.tan(alpha) * deltaT
        time = time + deltaT
    
    fig,ax = plt.subplots()
    xvals, yvals = zip(*trajectory)
    ax.plot(xvals,yvals, color="blue")
    create_circle(ax, R)
    window_limit = 70
    ax.set_xlim(left=-window_limit, right=window_limit)
    ax.set_ylim(bottom=-window_limit, top=window_limit)
    ax.set_aspect('equal', adjustable='box')
    plt.show()

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

if __name__ == "__main__":
    main()