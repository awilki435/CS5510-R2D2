import matplotlib.pyplot as plt
import numpy as np

def main():
    R = 59.0551/2
    v_r = 0
    v_l = 0
    L = 35
    W = 10
    x_eval = 0
    y_eval = -R
    theta_eval = 0 
    deltaT = .1
    time = 0
    circumfranceT = 30
    trajectory = []

    T = [1, 0.1, 0.01]

    for t in T:
        fig, ax = plt.subplots()
        i = 0 
        time = []
        while i < 100*t:
            x_eval = x_eval - 0.5*t*(v_r + v_l) * np.sin(theta_eval)
            y_eval = y_eval + 0.5*t*(v_r + v_l) * np.cos(theta_eval)
            theta_eval = theta_eval + t/W * (v_r - v_l)
            # assumming constant input
            v_l = v_l + t
            v_r = v_r + t
            i += 1

        window_limit = R+2
        ax.set_xlim(left=R-window_limit, right=R+window_limit)
        ax.set_ylim(bottom=-window_limit, top=window_limit)
        ax.set_aspect('equal', adjustable='box')
    plt.show()


def create_circle(ax, R):
    theta = 0
    x_vals = []
    y_vals = []
    while theta < 2 * np.pi + 0.1:
        y = R * np.sin(theta)
        x = R * np.cos(theta) + R
        theta = theta + 0.1
        x_vals.append(x)
        y_vals.append(y)
    ax.plot(x_vals,y_vals,color="green")
    return x_vals, y_vals

if __name__ == "__main__":
    main()