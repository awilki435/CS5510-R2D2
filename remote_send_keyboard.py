import socket

# Raspberry Pi's IP address and port
raspberry_pi_ip = '192.168.0.135'
port = 6000  # Replace with the Raspberry Pi's port number


# control car. Run script as sudo
import time
import keyboard

if __name__ == "__main__":
    char = 'A'
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((raspberry_pi_ip, port))
        while True:
            time.sleep(0.2)
            if keyboard.is_pressed('w') or keyboard.is_pressed('up'):
                char = 'w'
                s.send(char.encode())
            elif keyboard.is_pressed('s') or keyboard.is_pressed('down'):
                char = 'a'
                s.send(char.encode())
            elif keyboard.is_pressed('a') or keyboard.is_pressed('left'):
                char = 's'
                s.send(char.encode())
            elif keyboard.is_pressed('d') or keyboard.is_pressed('right'):
                char = 'd'
                s.send(char.encode())
            else:
                #print other key if pressed
                #print(keyboard.read_key())
                print('stop')