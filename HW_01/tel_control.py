# control car. Run script as sudo
from Car import Car
import time
import keyboard

if __name__ == "__main__":
    car = Car()

    car.control_car(100,100)

    time.sleep(0.1)

    car.control_car(0,0)

    while True:
        time.sleep(0.2)
        if keyboard.is_pressed('w') or keyboard.is_pressed('up'):
            car.control_car(100, 100)
            print('w')
        elif keyboard.is_pressed('s') or keyboard.is_pressed('down'):
            car.control_car(-100, -100)
            print('s')
        elif keyboard.is_pressed('a') or keyboard.is_pressed('left'):
            car.control_car(-50, 50)
            print('a')
        elif keyboard.is_pressed('d') or keyboard.is_pressed('right'):
            car.control_car(50, -50)
            print('d')
        else:
            #print other key if pressed
            #print(keyboard.read_key())
            car.control_car(0, 0)
            print('stop')