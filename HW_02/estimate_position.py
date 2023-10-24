def main(args=None):
    png_count = 198
    file_name = "pose3.txt"
    x_count = 0
    y_count = 0
    z_pos = 0
    square_side = png_count/4
    x_increase = 1/square_side
    y_increase = 1/square_side
    for i in range(png_count + 1):  # You can replace this with your robot's logic
        # Replace these lines with code to get the current x, y, and z coordinates
        x, y, z = x_count, y_count, z_pos
        if(i <= square_side):
            x_count -= x_increase
        if(i <= 2 * square_side and i > square_side):
            y_count += y_increase
        if(i <= 3 * square_side and i > 2 * square_side):
            x_count += x_increase
        if(i <= 4 * square_side and i > 3 * square_side):
            y_count -= y_increase
        # Open the file in append mode ('a') to add new coordinates without overwriting
        if i != 0:
            with open(file_name, 'a') as file:
                file.write(f"{x} {y} {z}\n")
        else: 
            with open(file_name, 'w') as file:
                file.write(f"{x} {y} {z}\n")



if __name__ == '__main__':
    main()





