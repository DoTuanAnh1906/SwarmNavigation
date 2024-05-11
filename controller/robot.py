from const import const
import numpy as np
from numpy.linalg import norm

class Robot:
    def __init__(self, index: int, init_pos: np.array):
        """
        Initialize the robot with its index, initial position, and goal position.

        Parameters:
        -----------
            index: int - index of the robot
            init_pos: np.array - initial position of the robot

        Returns:
        --------
            None
        """
        self.int_index: int = index             # index of the robot
        self.np_curPos: np.array = init_pos     # current position of the robot
        self.lst_hisPos: list['np.array'] = [self.np_curPos]  # history of the robot's position
    
    
    def update_pos(self, np_control: np.array, dt: float):
        """
        Update the position of the robot based on the control signal and the time step.
    
        Parameters:
        -----------
            np_control: np.array - control signal
            dt: float - time step
            
        Returns:
        --------
            None
        """
        self.np_curPos = self.np_curPos + np_control*dt
        self.lst_hisPos.append(self.np_curPos)
    
    def calc_control_signal(self, lst_robots: list['Robot'], lst_obstacles: list[np.array], goal: np.array):
        """
        Calculate the control signal for the robot. 

        Parameters:
        -----------
            lst_robots: list - list of robots
            lst_obstacles: list - list of obstacles points
            goal: np.array - goal position

        Returns:
        --------
            np_control: np.array - control signal
            bln_goalSts: bool - goal status
        """
        # calculate the control signal for move to goal
        np_mtg, bln_goalSts = self.move_to_goal(lst_robots, goal)

        # calculate the control signal for avoid collision
        np_ac  = self.avoid_collision(lst_robots)

        # calculate the control signal for avoid obstacle
        np_ao  = self.avoid_obstacle(lst_obstacles)

        # calculate the control signal for wall following
        np_wf  = self.wall_following(lst_obstacles)

        # calculate the control signal for cohesion
        np_c = self.cohesion(lst_robots)

        # calculate the total control signal
        # if the robots swarm is in wall following mode:
            # - the robot will avoid the obstacles and the walls
            # - the robot will avoid the collision with other robots
            # - the robot will follow the wall
        # else:
            # - the robot will avoid the obstacles
            # - the robot will avoid the collision with other robots
            # - the robot will move to the goal
            # - the robot will cohesion with other robots
            
        if np.linalg.norm(np_wf) > const.E:
            np_control = np_ao * const.W_AO + np_wf * const.W_WF + np_ac * const.W_AC
        else:
            np_control = np_ao * const.W_AO + np_mtg * const.W_MTG + np_ac * const.W_AC + np_c * const.W_C

        return np_control, bln_goalSts
    
    def avoid_obstacle(self, lst_obstacles: list[np.array]):
        """
        Create control signal to avoid collision with the obstacles.

        Parameters:
        -----------
            lst_obstacles: list - list of obstacles points

        Returns:
        --------
            np_ao: np.array - control signal
        """
        # initialize the control signal
        np_ao = np.zeros(2)
        np_ao_i = np.zeros(2)

        # init distance between the robot and the obstacle
        min_dis = float('inf')

        # get the number of obstacles point
        num_obstacles = len(lst_obstacles)

        # iterate through the list of obstacles
        for i in range(num_obstacles):
            
            # get the first, second obstacle points and the current position of the robot
            first_obs = lst_obstacles[i]
            second_obs = lst_obstacles[i-1]
            current_pos = self.np_curPos

            # calculate the perpendicular point of the current position to the line of the two obstacles
            np_perp = self.perpendicular(current_pos, first_obs, second_obs)

            # calculate the relative position and distance between the perpendicular point and the current position
            rel_perp_dis = self.calc_relative_pos(np_perp, current_pos)
            perp_dis     = self.calc_norm_dis(np_perp, current_pos)

            # only add the control signal if the obstacle is within the sensing range of the robot
            if perp_dis < min_dis:
                
                # update the minimum distance
                min_dis = perp_dis

                # create the control signal
                np_ao_i = rel_perp_dis/perp_dis

        # add the control signal to the total control signal
        if min_dis < const.SS_RANGE:
            np_ao += (const.SS_RANGE - min_dis)/(const.SS_RANGE - const.R_ROBOT - 0.02)*np_ao_i
        
        return np_ao
    
    def wall_following(self, lst_walls: list[np.array]):
        """
        Create control signal to avoid collision with the walls.

        Parameters:
        -----------
            lst_walls: list - list of walls points

        Returns:
        --------
            np_wf: np.array - control signal
        """
        # initialize the control signal
        np_wf = np.zeros(2)
        np_wf_i = np.zeros(2)

        # init distance between the robot and the wall
        min_dis = float('inf')

        # get the number of walls point
        num_walls = len(lst_walls)

        # iterate through the list of walls
        for i in range(num_walls):
            
            # get the first, second wall points and the current position of the robot
            first_wall = lst_walls[i]
            second_wall = lst_walls[i-1]
            current_pos = self.np_curPos

            # calculate the perpendicular point of the current position to the line of the two walls
            np_perp = self.perpendicular(current_pos, first_wall, second_wall)

            # calculate the relative position and distance between the perpendicular point and the current position
            rel_perp_dis = self.calc_relative_pos(np_perp, current_pos)
            perp_dis     = self.calc_norm_dis(np_perp, current_pos)

            # only add the control signal if the wall is within the sensing range of the robot
            if perp_dis < min_dis:
                
                # update the minimum distance
                min_dis = perp_dis

                # create the control signal
                np_wf_i = rel_perp_dis/perp_dis

        # add the control signal to the total control signal
        if min_dis < const.WALL_RANGE:

            # calculate the vector parallel to the wall
            perp_vector = np.array([np_wf_i[1], -np_wf_i[0]])
            np_wf += perp_vector
        
        return np_wf

    def move_to_goal(self, lst_robots: list['Robot'], np_goalPos: np.array):
        """
        Create control signal to move the robot to the goal position.

        Parameters:
        -----------
            goal: np.array - goal position
            lst_robots: list - list of robots

        Returns:
        --------
            np_mtg: np.array - control signal
            bln_goalSts: bool - goal status
        """
        # initialize the goal status = False
        bln_goalSts = False

        # calculate the centroid of the robots swarm
        np_centroid = np.zeros(2)
        for robot in lst_robots:
            np_centroid += robot.np_curPos
        np_centroid /= len(lst_robots)

        # calculate the relative position and distance between the centroid of the robots swarm and the goal position
        rel_dis = self.calc_relative_pos(np_centroid, np_goalPos)
        dis     = self.calc_norm_dis(np_centroid, np_goalPos)

        # if the centroid of the robots swarm is already at the goal position, return zero vector and goal status
        if dis < const.E:
            bln_goalSts = True
            return const.NP_ZERO, bln_goalSts
        
        # create control signal
        np_mtg = rel_dis/dis
        return np_mtg, bln_goalSts

    def avoid_collision(self, lst_robots: list['Robot']):
        """
        Create control signal to avoid collision with other robots.

        Parameters:
        -----------
            lst_robots: list - list of robots

        Returns:
        --------
            np_ac: np.array - control signal
        """
        # initialize the control signal
        np_ac = np.zeros(2)
        
        # iterate through the list of robots
        for robot in lst_robots:

            if robot.int_index == self.int_index:
                # skip the current robot
                continue
            
            # calculate the relative position and distance between the current robot and the other robot
            rel_dis = self.calc_relative_pos(self.np_curPos, robot.np_curPos)
            dis     = self.calc_norm_dis(self.np_curPos, robot.np_curPos)

            # only add the control signal if the other robot is within the sensing radius of current robot
            if dis < const.COLLISION_RANGE:
                
                # create the control signal
                np_ac_i = - rel_dis/dis

                # add the control signal to the total control signal
                np_ac += (const.COLLISION_RANGE - dis)/(const.COLLISION_RANGE - const.R_ROBOT - 0.02)*np_ac_i
            
        return np_ac
    
    def cohesion(self, lst_robots: list['Robot']):
        """
        Create control signal to move the robot closer to the other robot
        which the distance between them is larger than the sensing range.

        Parameters:
        -----------
            lst_robots: list - list of robots

        Returns:
        --------
            np_c: np.array - control signal
        """
        # initialize the control signal
        np_c = np.zeros(2)
        
        # iterate through the list of robots
        for robot in lst_robots:

            if robot.int_index == self.int_index:
                # skip the current robot
                continue
            
            # calculate the relative position and distance between the current robot and the other robot
            rel_dis = self.calc_relative_pos(self.np_curPos, robot.np_curPos)
            dis     = self.calc_norm_dis(self.np_curPos, robot.np_curPos)

            # only add the control signal if the other robot is within the sensing radius of current robot
            if dis > const.SS_RANGE:

                # create the control signal
                np_c_i = - rel_dis/dis

                # add the control signal to the total control signal
                np_c += (const.SS_RANGE - dis)*np_c_i
            
        return np_c

    @staticmethod
    def calc_norm_dis(pos1: np.array, pos2: np.array):
        """
        Calculate the distance between two points.

        Parameters:
        -----------
            pos1: np.array - position 1
            pos2: np.array - position 2

        Returns:
        --------
            distance: float - distance between the two points
        """
        return norm(pos2 - pos1)

    @staticmethod
    def calc_relative_pos(pos1: np.array, pos2: np.array):
        """
        Calculate the relative position between two points.

        Parameters:
        -----------
            pos1: np.array - position 1
            pos2: np.array - position 2

        Returns:
        --------
            relative_pos: np.array - relative position between the two points
        """
        relative_pos = pos2 - pos1
        return relative_pos
    
    @staticmethod
    def perpendicular(x, a, b):
        """
        Calculate the perpendicular vector of a given vector.

        Parameters:
        -----------
            x: np.array - vector
            a: np.array - point a
            b: np.array - point b

        Returns:
        --------
            np_perp: np.array - perpendicular vector
        """
        d_ab = Robot.calc_norm_dis(b, a)
        d_ax = Robot.calc_norm_dis(x, a)
        d_bx = Robot.calc_norm_dis(x, b)

        # if a and b are different points
        if d_ab != 0 :

            # if x is between a and b
            if np.dot(a-b, x-b) * np.dot(b-a, x-a) >= 0:
                px = b[0] - a[0]
                py = b[1] - a[1]
                
                # calculate the perpendicular vector
                dAB = px*px + py*py
                u = ((x[0] - a[0])*px + (x[1] - a[1])*py) / dAB
                np_perp = [a[0] + u*px, a[1] + u*py]
            else:
                if d_ax < d_bx:
                    np_perp = a
                else:
                    np_perp = b

        # if a and b are the same point
        else:   
            np_perp = a

        return np_perp