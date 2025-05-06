import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def load_data():
    with open("manipulator_path_animation.txt", "r") as f:
        # First line contains number of joints
        n = int(f.readline())
        # Rest contains joint angles
        data = np.loadtxt(f)
    return n, data

def compute_positions(angles, lengths):
    x = [0]
    y = [0]
    current_angle = 0
    for angle, length in zip(angles, lengths):
        current_angle += angle
        x.append(x[-1] + length * np.cos(current_angle))
        y.append(y[-1] + length * np.sin(current_angle))
    return x, y

def main():
    try:
        n_joints, angle_data = load_data()
        
        # Get link lengths (you'll need to input these to match your C++ program)
        lengths = []
        print(f"Manipulator with {n_joints} joints detected.")
        for i in range(n_joints):
            length = float(input(f"Enter length for link {i+1}: "))
            lengths.append(length)
        
        fig, ax = plt.subplots(figsize=(8, 8))
        line, = ax.plot([], [], 'bo-', linewidth=2)
        
        # Set plot limits based on total arm length
        total_length = sum(lengths)
        ax.set_xlim(-total_length*1.2, total_length*1.2)
        ax.set_ylim(-total_length*1.2, total_length*1.2)
        ax.grid(True)
        ax.set_title('Manipulator Movement Animation')
        ax.set_aspect('equal')

        def init():
            line.set_data([], [])
            return line,

        def update(frame):
            x, y = compute_positions(angle_data[frame], lengths)
            line.set_data(x, y)
            return line,

        ani = FuncAnimation(fig, update, frames=len(angle_data),
                          init_func=init, blit=True, interval=50)
        plt.show()
        
    except FileNotFoundError:
        print("Error: Animation data file not found.")
        print("Please run the C++ program first to generate the path data.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")

if __name__ == "__main__":
    main()