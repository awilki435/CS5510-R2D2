# create a sqare program that has the car go forward for 2 sec then turn left

from Car import Car
import time


if __name__ == "__main__":
    cycles = 4

    car = Car()

    for i in range(cycles):
        car.control_car(100, 100)
        time.sleep(2)
        car.control_car(0, 0)
        time.sleep(0.1)
        car.control_car(-50, 50)
        time.sleep(0.5)
        car.control_car(0, 0)
        time.sleep(0.1)