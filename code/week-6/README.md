# Week 6 - Prediction & Behaviour Planning

---

## Assignment #1

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.

Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.


---

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

* determine which state transitions are possible from the current state (`successor_states()` function in the same module will be helpful),
* calculate cost for each state transition using the trajectory generated for each behaviour, and
* select the minimum cost trajectory and return it.

Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.

# Week 6 - Prediction & Behaviour Planning [Assignment]

---

* GNB (Gaussian Naive Bayes) : GNB를 이용한 prediction code에서 Train, prediction code 수정
* BP (Behavior Planning) : BP 프로젝트에서 Vehicle.py의 cost에 따라 다음 state를 계산하여 출력하는 Chosse_nest_state 수정 / 경로에 따라 cost function (distance, inefficienty)를 작성하여 cost function에 따라 빠르게 목적지에 도달하도록 수정

---

[assignment - 1]

- Train function
- 각 class 별로 Trainning data를 저장하여, 4개의 raw value에 대한 평균과 편차를 계산하여 출력한다.
- Step 1. Y는 Trainning data의 label로, label별로 해당되는 raw value를 append하여 matrix를 구성한다.
- Step 2. 구성된 label 별 raw value matrix에서 numpy의 mean, std 함수를 이용해 raw value 별로 평균과 편차를 계산한다.

        Train_data = {}

        for class_name in self.classes :
            Train_data[class_name] = []

        for ind in range(len(Y)) :
            label = Y[ind]
            Train_data[label].append(X[ind])

        self.class_Train = {label :{} for label in self.classes} 
        for label_tgt in self.classes :
            self.class_Train[label_tgt]['mean'] = np.mean(Train_data[label_tgt], 0)
            self.class_Train[label_tgt]['std'] = np.std(Train_data[label_tgt], 0)
                    
        return self.class_Train


- prediction function
- 각 class 별로 가우시안 확률 분포를 이용해 계측된 data에 대한 label 확률을 계산한다.
- Step 1. 각 class 별로 확률을 누적하여 계산 (proba = 1)
- Step 2. 계측 data를 앞서 Train function에서 도출된 raw value 별 평균과 편차를 기준으로 확률을 계산하여 곱한다.


        probas = np.zeros(len(self.classes))
        # Calculate Gaussian probability for each variable based on the
        # mean and standard deviation calculated in the training process.
        for ind_class in range(len(self.classes)):
            proba = 1

            # Multiply all the probabilities for variables, and then
            # normalize them to get conditional probabilities.
            for ind_obs in range(len(observation)) :
                var_mean = self.class_Train[self.classes[ind_class]]['mean']
                var_std = self.class_Train[self.classes[ind_class]]['std']
                proba *= gaussian_prob(observation[ind_obs], var_mean[ind_obs], var_std[ind_obs])
            probas[ind_class] = proba
        res = probas / probas.sum()
        
        # Return the label for the highest conditional probability.
        return self.classes[res.argmax(axis=0)]


[assignment - 2]

- Choose_next_state function
- Step 1. successor_states() 함수를 통해 고려가 필요한 (천이가 가능한) state 구분
- Step 2. generate_trajectory() 함수를 통해 천이 가능한 각 state들에 대하여 예상되는 Trajectory 도출
- Step 3. 각 Trajectory에 대한 Cost를 계산
- Step 4. 가능한 state들의 trajectory들 중 가장 작은 cost를 갖는 trajectory 출력

        possible_state = self.successor_states()

        cost_List = []
        cost_min = 9999
        
        ind = 0

        for state_tgt in possible_state : 
            trajectory_tgt = self.generate_trajectory(state_tgt, predictions)
            cost_tgt = calculate_cost(self, trajectory_tgt, predictions)
            cost_List.append({'state':state_tgt, 'cost':cost_tgt})

            if (ind == 0) : 
                cost_min = cost_tgt
                sate_min = state_tgt
                trajectory_min = trajectory_tgt
            else :
                if (cost_tgt < cost_min) :
                    cost_min = cost_tgt
                    sate_min = state_tgt
                    trajectory_min = trajectory_tgt
            ind += 1
        print(sate_min)
        return trajectory_min
	
	
- goal_distance_cost function
- 차량의 목적지가 존재하는 goal_lane과 현재 차량의 최종 lane, 이동하려는 lane을 이용하여, 목적지 lane과의 lane 차이 만큼의 cost를 출력하는 함수 구성
- lane 차이에 대한 error_Lat과 목적지에 대한 종방향 거리에 대한 error_Long을 이용하여, 목적지와의 거리가 가까워질 수록, lane 차이에 대한 cost가 더욱 크게 반영되도록 구성
- 횡방향 오차는 현재 차량의 최종 lane과 이동하려는 lane을 더하여, 차선유지 state와 차선 변경 state에 대한 cost를 차등하여 구분하도록 구성
- 횡방향 오차의 제곱을 종방향 오차로 나누어 cost 구성


	    error_Lat = abs((vehicle.goal_lane - data.intended_lane) + (vehicle.goal_lane - data.final_lane))
	    error_Long = data.end_distance_to_goal

	    cost = (error_Lat*error_Lat) / error_Long
	    return cost


- inefficiency_cost function
- 각 lane의 주행 속도를 이용하여, 빠르게 이동할 수 있는 lane으로 변경하도록 하는 cost 함수 구성
- 현재 차량의 주행 최종 lane의 주행 속도와 자차량의 속도 차, 이동하려는 lane의 주행 속도와 자차량의 속도 차를 cost에 누적
- velocity 함수의 nonetype 출력에 대한 예외처리 


	    cost = 0    
	    velocity_intended_lane = velocity(predictions, data.intended_lane)
	    if not (velocity_intended_lane is None) : 
		if (velocity_intended_lane < vehicle.target_speed) :
		    cost += vehicle.target_speed - velocity_intended_lane

	    velocity_final_lane = velocity(predictions, data.final_lane)
	    if not (velocity_final_lane is None) : 
		if  (velocity_final_lane < vehicle.target_speed) :
		    cost += vehicle.target_speed - velocity_final_lane

	    return cost

