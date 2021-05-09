import numpy as np
import itertools

# Given map
grid = np.array([
    [1, 1, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 1]
])

# List of possible actions defined in terms of changes in
# the coordinates (y, x)
forward = [
    (-1,  0),   # Up
    ( 0, -1),   # Left
    ( 1,  0),   # Down
    ( 0,  1),   # Right
]

# Three actions are defined:
# - right turn & move forward
# - straight forward
# - left turn & move forward
# Note that each action transforms the orientation along the
# forward array defined above.
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

init = (4, 3, 0)    # Representing (y, x, o), where
                    # o denotes the orientation as follows:
                    # 0: up
                    # 1: left
                    # 2: down
                    # 3: right
                    # Note that this order corresponds to forward above.
goal = (2, 0)
cost = (2, 1, 20)   # Cost for each action (right, straight, left)

# EXAMPLE OUTPUT:
# calling optimum_policy_2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]

def optimum_policy_2D(grid, init, goal, cost):
    # Initialize the value function with (infeasibly) high costs.
    value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
    # Initialize the policy function with negative (unused) values.
    policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
    # Final path policy will be in 2D, instead of 3D.
    policy2D = np.full(grid.shape, ' ')

    # Apply dynamic programming with the flag change.
    change = True
    while change:
        change = False
        # This will provide a useful iterator for the state space.
        p = itertools.product(
            range(grid.shape[0]),
            range(grid.shape[1]),
            range(len(forward))
        )
        # Compute the value function for each state and
        # update policy function accordingly.
        for y, x, t in p:
            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
                if (y, x) == goal and value[(t, y, x)] > 0:
                    value[(t,y,x)] = 0
                    policy[(t,y,x)] = -9999
                    change = True
                    
            # Try to use simple arithmetic to capture state transitions.
            elif grid[(y, x)] == 0:
                for act in range(len(init)) :
                    o2 = (t + action[act])
                    if (o2 == 4) : 
                        o2 = 0
                    elif (o2 == -1) : 
                        o2 = 3
                    y2 = y + forward[o2][0]
                    x2 = x + forward[o2][1]

                    if (x2 >= 0) and (x2 < grid.shape[1]) and (y2 >= 0) and (y2 < grid.shape[0]) and (grid[(y2, x2)] == 0) :  
                        v2 = value[(o2, y2, x2)] + cost[act]
                        if v2 < value[(t, y ,x)] :  
                            value[(t, y, x)] = v2  
                            policy[(t, y, x)] = action[act]  
                            change = True  
        
    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
    y_tgt, x_tgt, o_tgt = init
    policy2D[(y_tgt, x_tgt)] = policy[(o_tgt, y_tgt, x_tgt)]

    while (policy[(o_tgt, y_tgt, x_tgt)] != -9999) :  
        if (policy[(o_tgt, y_tgt, x_tgt)] == action[0]) :
            o_next = (o_tgt - 1) % 4
            policy2D[(y_tgt, x_tgt)] = action_name[0]
        elif (policy[(o_tgt, y_tgt, x_tgt)] == action[1]) :
            o_next = o_tgt
            policy2D[(y_tgt, x_tgt)] = action_name[1]
        elif (policy[(o_tgt, y_tgt, x_tgt)] == action[2]) :
            o_next = (o_tgt + 1) % 4
            policy2D[(y_tgt, x_tgt)] = action_name[2]

        y_tgt += forward[o_next][0]
        x_tgt += forward[o_next][1]
        o_tgt = o_next
        policy2D[(y_tgt, x_tgt)] = policy[(o_tgt, y_tgt, x_tgt)]

    if policy[(o_tgt, y_tgt, x_tgt)] == -9999:  
        policy2D[(y_tgt, x_tgt)] = '*'    

    # Return the optimum policy generated above.
    return policy2D

print(optimum_policy_2D(grid, init, goal, cost))
