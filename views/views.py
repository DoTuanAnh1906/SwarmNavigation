import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import os
from controller.robot import *
from const import const

def plot_single_robot(robot: Robot):
    data = robot.lst_hisPos

    x_goal, y_goal = const.GOAL_POS.T

    X = [point[0] for point in data]
    Y = [point[1] for point in data]

    length = len(data)
    img = plt.figure()
    ax = img.add_subplot()
    obstacle = Polygon(const.OBS)
    ax.add_patch(obstacle)

    plt.plot(X, Y, label='Path')
    plt.scatter(x_goal, y_goal, label='Goal Position', marker='*', s=100, color='red', zorder=length+1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(-1, 13)
    plt.ylim(-1, 13)
    plt.title('Robot Path')
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_multiple_robots(robots: list['Robot']):
    currentPath = os.getcwd()

    x_goal, y_goal = const.GOAL_POS.T

    img = plt.figure()
    ax = img.add_subplot()
    obstacle = Polygon(const.OBS)
    ax.add_patch(obstacle)

    for robot in robots:
        
        data = robot.lst_hisPos

        X = [point[0] for point in data]
        Y = [point[1] for point in data]

        length = len(data)

        plt.plot(X, Y, label='Path')
        plt.scatter(X[length-1], Y[length-1], marker='8', s=25)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.xlim(-1, 13)
        plt.ylim(-1, 13)
        plt.title('Robot Path')
        plt.grid(True)
    plt.scatter(x_goal, y_goal, marker='*', s=100, zorder=length+1)
    plt.savefig(currentPath + '/output/result.png')
    plt.close()

def plot_step_robots(robots: list['Robot']):
    currentPath = os.getcwd()
    iteration = len(robots[0].lst_hisPos)

    x_goal, y_goal = const.GOAL_POS.T

    for i in range(iteration):
        img = plt.figure()
        ax = img.add_subplot()
        obstacle = Polygon(const.OBS)
        ax.add_patch(obstacle)

        for robot in robots:
            data = robot.lst_hisPos

            X = [point[0] for point in data]
            Y = [point[1] for point in data]

            length = len(data)

            plt.plot(X[:i], Y[:i])
            plt.scatter(X[i-1], Y[i-1], marker='8', s=25)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.xlim(-1, 13)
            plt.ylim(-1, 13)
            plt.title('Robot Path')
            plt.grid(True)
        plt.scatter(x_goal, y_goal, marker='*', s=100, zorder=length+1)
        plt.savefig(currentPath + f'/output/step/result{i}.png')
        plt.close()
