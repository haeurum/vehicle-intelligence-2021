# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.


def motion_model(position, mov, priors, map_size, stdev):
    # Initialize the position's probability to zero.
    position_prob = 0.0
    for x_n in range(map_size) :
        position_prob += norm_pdf(position - x_n, mov, stdev)*priors[x_n]
    return position_prob

    # markov localization에서 모션 모델에 대한 probability를 계산하는 과정
    # 전체 좌표에서 prior belief position이 1 sample 거동했을때, 특정 position에 위치할 확률을 계산함
    # 따라서, 각 좌표에서 이전 좌표와 특정 대상 좌표에 차이가, 모델 모션 변화량을 mean으로 갖는 확률 분포를 이용하여
    # 누적하여 계산하였음.
    
def observation_model(landmarks, observations, pseudo_ranges, stdev):
    # Initialize the measurement's probability to one.
    distance_prob = 1.0
    if ((not observations) or (len(observations) > len(pseudo_ranges))) :
        distance_prob = 0
    else :
        for x_n in range(0, len(observations)) :
            distance_prob *= norm_pdf(observations[x_n], pseudo_ranges[x_n], stdev)

    # markov localization에서 계측 모델에 대한 probability를 계산하는 과정
    # 계측 정보가 없거나, 실제 대상 object보다 계측이 많은 경우의 확률은 존재하지 않기 때문에, 예외처리함
    # 정상적으로 계측 정보가 존재하는 경우에는, 계측된 object(landmark)에 대한 거리에 대해서 
    # 각 object(landmark)에 대한 map 기준 거리를 mean으로 갖는 확률 분포를 이용하여 계산하였음 
