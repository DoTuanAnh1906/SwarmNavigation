from const import const
from controller.robot import *
from views import views
from numpy.random import rand
from scipy.optimize import linear_sum_assignment

# create a list of robots
lst_robots: list['Robot'] = []

for i in range(const.N):
    start_pos = const.START_POS + const.OFFSET * rand(2)
    robot = Robot(i, start_pos)
    lst_robots.append(robot)

# set the initial iteration = 1
it = 1

# initialize the goal status
bln_goalSts = False

while it < const.NUM_STEP:
    # calculate the control signal for each robot
    lst_controls = []
    for i in range(const.N):
        control_signal, bln_goalSts = lst_robots[i].calc_control_signal(lst_robots, const.OBS, const.GOAL_POS)
        lst_controls.append(control_signal)

    # check if the goal is reached
    if bln_goalSts:
        break

    # update the state of each robot
    for i in range(const.N):
        lst_robots[i].update_pos(lst_controls[i], const.DT)

    # increment the iteration
    it += 1

# get the iteration number
print(it)

# export the robots position to image

# views.plot_single_robot(robot1)
views.plot_multiple_robots(lst_robots)
# views.plot_step_robots(lst_robots)