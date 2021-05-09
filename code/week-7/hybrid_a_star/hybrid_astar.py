import numpy as np
import math

class HybridAStar:

    # HybridAstar Variable
        # NUM_THETA_CELLS 
            # Grid Cell의 개수 지정
        # omega_min / omega_max / omega_step
            # 조향각에 따라 state space에 대한 stack을 분리하기 위해 min,max와 resolution 특성값을 지정
        # speed / length
            # model의 단순 차량 모델의 특성값 설정

    NUM_THETA_CELLS = 90
    omega_min = -35
    omega_max = 35
    omega_step = 5
    speed = 1.0
    length = 0.5

    # Determine how many grid cells to have for theta-axis.
    # Define min, max, and resolution of steering angles
    # A very simple bicycle model


    # Initialize the search structure.
    def __init__(self, dim):
        self.dim = dim
        self.closed = np.zeros(self.dim, dtype=np.int)
        self.came_from = np.full(self.dim, None)

    # Expand from a given state by enumerating reachable states.
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        # Consider a discrete selection of steering angles.
        
        g2 = g + 1
        next_states = []

        for delta_t in range(self.omega_min, self.omega_max+1 , self.omega_step) :
            omega = self.speed / self.length * math.tan(np.pi / 180 * delta_t)
            next_x = x + self.speed * math.cos(theta)
            next_y = y + self.speed * math.sin(theta)
            next_theta = theta + omega
            next_g = g2
            next_f = next_g + self.heuristic(next_x, next_y, goal)

            if (self.idx(next_x) >= 0) and (self.idx(next_x) <= self.dim[1]) and (self.idx(next_y) >= 0) and (self.idx(next_y) <= self.dim[2]) :
                next_states.append({'x':next_x, 'y':next_y, 't':next_theta, 'g':next_g, 'f':next_f})

        return next_states
        # TODO: implement the trajectory generation based on
        # a simple bicycle model.
        # Let theta2 be the vehicle's heading (in radian)
        # between 0 and 2 * PI.
        # Check validity and then add to the next_states list.

    # Perform a breadth-first search based on the Hybrid A* algorithm.
    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the last component of the tuple start.
        # Determine the cell to contain the initial state, as well as the state itself.
        theta = start[-1]
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by the heuristic function.
        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for n in next_states:
                x2, y2, theta2 = n['x'], n['y'], n['t']
                if (x2 < 0 or x2 >= len(grid)) or (y2 < 0 or y2 >= len(grid[0])):    
                    continue

                stack2 = self.theta_to_stack_num(theta2)
                idx_x, idx_y = self.idx(x2), self.idx(y2)

                if (self.closed[stack2][idx_x][idx_y] == 0) and (grid[idx_x][idx_y] == 0) :
                    opened.append(n)
                    self.closed[stack2][idx_x][idx_y] = 1
                    self.came_from[stack2][idx_x][idx_y] = curr
                    total_closed += 1
        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        # TODO: implement a function that calculate the stack number
        # given theta represented in radian. Note that the calculation
        # should partition 360 degrees (2 * PI rad) into different
        # cells whose number is given by NUM_THETA_CELLS.
        # 특정 theta 값이 어떠한 steering stack에 포함되는지 index를 계산
        new_theta = np.fmod((theta + 2 * np.pi), (2 * np.pi))
        stack_num = np.round(new_theta * self.NUM_THETA_CELLS / (2 * np.pi)) % self.NUM_THETA_CELLS
        return int(stack_num)

    # Calculate the index of the grid cell based on the vehicle's position.
    def idx(self, pos):
        # We simply assume that each of the grid cell is the size 1 X 1.
        return int(np.floor(pos))

    # Implement a heuristic function to be used in the hybrid A* algorithm.
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        return 0

    # Reconstruct the path taken by the hybrid A* algorithm.
    def reconstruct_path(self, start, goal):
        # Start from the final state, and follow the link to the
        # previous state using the came_from matrix.
        curr = self.final
        x, y = curr['x'], curr['y']
        path = []
        while x != start[0] and y != start[1]:
            path.append(curr)
            stack = self.theta_to_stack_num(curr['t'])
            x, y = curr['x'], curr['y']
            curr = self.came_from[stack][self.idx(x)][self.idx(y)]
        # Reverse the path so that it begins at the starting state
        # and ends at the final state.
        path.reverse()
        return path
