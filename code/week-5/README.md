# Week 5 - Path Planning & the A* Algorithm

---

## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.

# Week 5 - Path Planning & the A* Algorithm [Assignment]  

---

- dynamic programming 기반의 path planning algorithm에서 차량의 state를 orientaion까지 추가하여 구성
- cost를 변경하여, planning된 path에 대한 경로 확인

---

[assignment - 1]

Step 0. algorithm 변수 정의
    init : 차량의 초기 상태 변수 (y, x, o)
    goal : grid map에서 도착지 정보 (y, x)
    cost : 각 action에 대한 cost
    value : Map의 각 cell에 대한 cost를 저장하기 위한 변수 
    policy : Map의 각 cell마다 cost에 따라 결정된 목적지로 향하는 action
    
Step 2. Dynamic programming 반복
    각 cell의 cost와 목적지로 가기 위한 action을 저장하기 위한 변수 초기화
    
Step2-1. 
    target cell가 goal인 경우의 알고리즘 변수 지정 (cost = 0)
    if (y, x) == goal and value[(t, y, x)] > 0:
	value[(t,y,x)] = 0
	policy[(t,y,x)] = -9999
	change = True

Step2-2.
    target cell이 goal이 아니지만 유효한 좌표인 경우, 각 action을 통해 이동가능한 cell에 대한 체크
    가능한 action 중 가장 작은 cost를 갖는 action으로 target cell의 policy 지정
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


Step 3. policy plotting code
    계산한 policy에 대해 경로를 표시하기 code 

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
	

![image](https://user-images.githubusercontent.com/80522886/117574376-bdb43480-b117-11eb-8d6e-f0be80c5f0f0.png)


[assignment - 2]

각 action에 대한 cost를 변경함에 따라 알고리즘에 의해 결정되는 path는 달라짐
위 assignment-1에서는 cost를 (2,1,20)으로 설정했을 때 결정된 path이고
cost를 (2,1,10)으로 설정한 경우에는 아래와 같은 결과를 얻을 수 있다.

![image](https://user-images.githubusercontent.com/80522886/117574457-226f8f00-b118-11eb-911a-327ff939af71.png)



