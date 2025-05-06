#!/usr/bin/env python3

import argparse
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FuncAnimation

def get_obstacles_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            obstacles = []
            for line in lines:
                obstacle = line.split()
                if len(obstacle) == 4:
                    obstacles.append([float(val) for val in obstacle])
        return obstacles
    else:
        print("Obstacle file not found - skipping obstacles.")
        return []

def get_path_and_robot_info(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
            num_joints = int(lines[0])
            link_lengths = [float(val) for val in lines[1].split()]
            path = []
            for line in lines[2:]:
                values = [float(val) for val in line.split()]
                angles = values[:num_joints]
                velocities = values[num_joints:]  # velocities not used here but could be
                path.append(angles)
        return num_joints, link_lengths, path
    else:
        print("Path file not found.")
        sys.exit(1)

def forward_kinematics(joint_angles, link_lengths, base=(0, 0)):
    positions = [np.array(base)]
    angle = 0.0
    current_pos = np.array(base)
    for i, joint in enumerate(joint_angles):
        angle += joint
        dx = link_lengths[i] * np.cos(angle)
        dy = link_lengths[i] * np.sin(angle)
        current_pos = current_pos + np.array([dx, dy])
        positions.append(current_pos.copy())
    return positions

def draw_environment(ax, obstacles):
    for obs in obstacles:
        ax.add_patch(patches.Rectangle((obs[0], obs[1]), obs[2], obs[3], fill=True, color='black'))

def set_plot_properties(ax):
    ax.grid(True)
    ax.set_aspect('equal', 'box')
    ax.set_facecolor('#f0f0f0')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_title('Manipulator Path Visualization')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

def plot_environment_and_path(obstacles, path, link_lengths):
    fig, ax = plt.subplots()
    draw_environment(ax, obstacles)
    for joint_angles in path:
        positions = forward_kinematics(joint_angles, link_lengths)
        positions = np.array(positions)
        ax.plot(positions[:, 0], positions[:, 1], 'g-')
        ax.scatter(positions[:, 0], positions[:, 1], s=10, c='green')
    set_plot_properties(ax)
    plt.savefig("manipulator_path.png", dpi=300)
    plt.close()

def animate_path(obstacles, path, link_lengths):
    fig, ax = plt.subplots()
    draw_environment(ax, obstacles)
    line, = ax.plot([], [], 'g-', linewidth=2)
    points, = ax.plot([], [], 'go', markersize=4)
    set_plot_properties(ax)

    def update(frame):
        angles = path[frame]
        positions = forward_kinematics(angles, link_lengths)
        pos_arr = np.array(positions)
        line.set_data(pos_arr[:, 0], pos_arr[:, 1])
        points.set_data(pos_arr[:, 0], pos_arr[:, 1])
        return line, points

    ani = FuncAnimation(fig, update, frames=len(path), blit=True, repeat=False)
    ani.save("manipulator_path.gif", writer='imagemagick', fps=20)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Visualize an n-joint manipulator path with obstacles.")
    parser.add_argument('--environment', type=str, default='obstacles.txt', help='Obstacle file (x y w h per line)')
    parser.add_argument('--path', type=str, default='path.txt', help='Path file (see format)')
    args = parser.parse_args()

    obs_path = os.path.join(os.getcwd(), args.environment)
    path_path = os.path.join(os.getcwd(), args.path)

    obstacles = get_obstacles_from_file(obs_path)
    # obstacles = []
    num_joints, link_lengths, path = get_path_and_robot_info(path_path)

    print(f"Loaded {len(obstacles)} obstacles and {len(path)} path configurations for a {num_joints}-joint manipulator.")

    plot_environment_and_path(obstacles, path, link_lengths)
    animate_path(obstacles, path, link_lengths)
    print("Saved static image as 'manipulator_path.png' and animation as 'manipulator_path.gif'.")

if __name__ == '__main__':
    main()
