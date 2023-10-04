

"""
x_t+1 = = x_t + -vsin(theta) * delta t
y_t+1 = y_t + vcos(theta) * delta t
theta_t+1 = theta_t + (v/L) tan(alpha) * delta t
"""
import matplotlib
import matplotlib.pyplot as plt
import numpy


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
    mark = 0
    circumfranceT = 46.5
    trajectory = []

    while mark < circumfranceT:
        if mark < 2:
            alpha = 0
        else: 
            alpha = numpy.arctan(L/R)
        x =  x + -v * numpy.sin(theta) * deltaT
        y = y + v * numpy.cos(theta) * deltaT
        theta = theta + (v/L) * numpy.tan(alpha) * deltaT
        trajectory.append((x,y))
        mark = mark + deltaT
    
    fig,ax = plt.subplots()
    xvals, yvals = zip(*trajectory)
    ax.plot(xvals,yvals, color="blue")
    matplotlib.patches.Circle((0,0), R, color="green")
    plt.show()


if __name__ == "__main__":
    main()