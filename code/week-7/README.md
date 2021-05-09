# Week 7 - Hybrid A* Algorithm & Trajectory Generation

---

[//]: # (Image References)
[has-example]: ./hybrid_a_star/has_example.png
[ptg-example]: ./PTG/ptg_example.png

## Assignment: Hybrid A* Algorithm

In directory [`./hybrid_a_star`](./hybrid_a_star), a simple test program for the hybrid A* algorithm is provided. Run the following command to test:

```
$ python main.py
```

The program consists of three modules:

* `main.py` defines the map, start configuration and end configuration. It instantiates a `HybridAStar` object and calls the search method to generate a motion plan.
* `hybrid_astar.py` implements the algorithm.
* `plot.py` provides an OpenCV-based visualization for the purpose of result monitoring.

You have to implement the following sections of code for the assignment:

* Trajectory generation: in the method `HybridAStar.expand()`, a simple one-point trajectory shall be generated based on a basic bicycle model. This is going to be used in expanding 3-D grid cells in the algorithm's search operation.
* Hybrid A* search algorithm: in the method `HybridAStar.search()`, after expanding the states reachable from the current configuration, the algorithm must process each state (i.e., determine the grid cell, check its validity, close the visited cell, and record the path. You will have to write code in the `for n in next_states:` loop.
* Discretization of heading: in the method `HybridAStar.theta_to_stack_num()`, you will write code to map the vehicle's orientation (theta) to a finite set of stack indices.
* Heuristic function: in the method `HybridAStar.heuristic()`, you define a heuristic function that will be used in determining the priority of grid cells to be expanded. For instance, the distance to the goal is a reasonable estimate of each cell's cost.

You are invited to tweak various parameters including the number of stacks (heading discretization granularity) and the vehicle's velocity. It will also be interesting to adjust the grid granularity of the map. The following figure illustrates an example output of the program with the default map given in `main.py` and `NUM_THETA_CELLS = 360` while the vehicle speed is set to 0.5.

![Example Output of the Hybrid A* Test Program][has-example]

---

## Experiment: Polynomial Trajectory Generation

In directory [`./PTG`](./PTG), a sample program is provided that tests polynomial trajectory generation. If you input the following command:

```
$ python evaluate_ptg.py
```

you will see an output such as the following figure.

![Example Output of the Polynomial Trajectory Generator][ptg-example]

Note that the above figure is an example, while the result you get will be different from run to run because of the program's random nature. The program generates a number of perturbed goal configurations, computes a jerk minimizing trajectory for each goal position, and then selects the one with the minimum cost as defined by the cost functions and their combination.

Your job in this experiment is:

1. to understand the polynomial trajectory generation by reading the code already implemented and in place; given a start configuration and a goal configuration, the algorithm computes coefficient values for a quintic polynomial that defines the jerk minimizing trajectory; and
2. to derive an appropriate set of weights applied to the cost functions; the mechanism to calculate the cost for a trajectory and selecting one with the minimum cost is the same as described in the previous (Week 6) lecture.

Experiment by tweaking the relative weight for each cost function. It will also be very interesting to define your own cost metric and implement it using the information associated with trajectories.


# Week 7 - Hybrid A* Algorithm & Trajectory Generation [Assignment]

---

* Hybid A* : Hybrid A* 알고리즘의 지정한 조향각에 대한 stack 별 다음 state를 계산/확장하기 위한 expand 함수와 expand된 Map 정보를 통해 optimistic한 경로를 도출하기 위한 search 함수를 작성
* 

---

[assignment - 1]

- Hybrid A* 알고리즘은 Continuous한 state를 다루는 알고리즘을 discrete한 방식으로 계산하는 알고리즘이다. 따라서 지정된 차량의 입력(조향각) 최소, 최대값을 특정 갯수로 나누어 확장하고, 최적의 경로를 도출함.
 
- expand function
- 각 stack 마다 지정된 조향 입력에 따라 확장한다. 이때, 확장한 state가 유효한 state인지 판단하고, 유효하다면 next_state에 append
- Step 1. 조향 입력별로, 차량 모델에 따라 결정되는 x,y,theta 계산
- Step 2. 해당 state가 갖는 heuristic, g function 계산
- Step 3. 해당 state가 전체 Grid Map에서 유효한 범위 내에 포함되는지 판단하고, 유효하다면 next_state에 추가(누적)


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
        
        
- search function
        
        
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

