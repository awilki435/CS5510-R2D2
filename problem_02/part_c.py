import matplotlib.pyplot as plt
import numpy as np
import pytictoc


def main():
    R = 59.0551 / 2
    W = 10
    v_r = 20
    v_l = (R - W / 2) / (R + W / 2) * v_r

    T = [1, 0.1, 0.01]
    T_str = ["1", "0.1", "0.01"]
    times = []
    timer = pytictoc.TicToc()
    fig, ax = plt.subplots()
    for t in T:
        timer.tic()
        x_eval = R
        y_eval = 0
        theta = 0
        theta_prev = -1
        errors = []
        s_time = 0
        sim_time = []
        while theta <= 2 * np.pi + t:
            x_eval = x_eval - 0.5 * t * (v_r + v_l) * np.sin(theta)
            y_eval = y_eval + 0.5 * t * (v_r + v_l) * np.cos(theta)
            x_real = R * np.cos(theta)
            y_real = R * np.sin(theta)
            theta = theta + t / W * (v_r - v_l)
            error = np.linalg.norm([x_real - x_eval, y_real - y_eval])
            errors.append(error)
            s_time += t
            sim_time.append(s_time)
            if theta_prev == theta:
                break
            theta_prev = theta
        times.append(timer.tocvalue(restart=True))
        ax.plot(sim_time, errors, label=f"$\Delta$t = {t}")
        plt.title("Error Over Time for each $\Delta$t")
        plt.xlabel("Simulation Time")
        plt.ylabel("Error")
    plt.legend()
    fig2 = plt.figure(2)
    plt.bar(T_str, times, width=1)
    plt.title("Computation Time for each $\Delta$t")
    plt.xlabel("$\Delta$t")
    plt.ylabel("Computation Time (s)")
    plt.show()


if __name__ == "__main__":
    main()
