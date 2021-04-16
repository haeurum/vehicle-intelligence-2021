# Week 3 - Kalman Filters, EKF and Sensor Fusion

---

[//]: # (Image References)
[kalman-result]: ./kalman_filter/graph.png
[EKF-results]: ./EKF/plot.png

## Kalman Filter Example

In directory [`./kalman_filter`](./kalman_filter), a sample program for a small-scale demonstration of a Kalman filter is provided. Run the following command to test:

```
$ python testKalman.py
```

This program consists of four modules:

* `testKalman.py` is the module you want to run; it initializes a simple Kalman filter and estimates the position and velocity of an object that is assumed to move at a constant speed (but with measurement error).
* `kalman.py` implements a basic Kalman fitler as described in class.
* `plot.py` generates a plot in the format shown below.
* `data.py` provides measurement and ground truth data used in the example.

The result of running this program with test input data is illustrated below:

![Testing of Kalman Filter Example][kalman-result]

Interpretation of the above results is given in the lecture.

In addition, you can run `inputgen.py` to generate your own sample data. It will be interesting to experiment with a number of data sets with different characteristics (mainly in terms of variance, i.e., noise, involved in control and measurement).

---

## Assignment - EFK & Sensor Fusion Example

In directory [`./EKF`](./EKF), template code is provided for a simple implementation of EKF (extended Kalman filter) with sensor fusion. Run the following command to test:

```
$ python run.py
```

The program consists of five modules:

* `run.py` is the modele you want to run. It reads the input data from a text file ([data.txt](./EKF/data.txt)) and feed them to the filter; after execution summarizes the result using a 2D plot.
* `sensor_fusion.py` processees measurements by (1) adjusting the state transition matrix according to the time elapsed since the last measuremenet, and (2) setting up the process noise covariance matrix for prediction; selectively calls updated based on the measurement type (lidar or radar).
* `kalman_filter.py` implements prediction and update algorithm for EKF. All the other parts are already written, while completing `update_ekf()` is left for assignment. See below.
* `tools.py` provides a function `Jacobian()` to calculate the Jacobian matrix needed in our filter's update algorithm.
*  `plot.py` creates a 2D plot comparing the ground truth against our estimation. The following figure illustrates an example:

![Testing of EKF with Sensor Fusion][EKF-results]

### Assignment

Complete the implementation of EKF with sensor fusion by writing the function `update_ekf()` in the module `kalman_filter`. Details are given in class and instructions are included in comments.



def update_ekf(self, z):
        H_j = Jacobian(self.x)
        S = np.dot(H_j, np.dot(self.P, H_j.T)) + self.R
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))

        px = self.x[0]
        py = self.x[1]
        vx = self.x[2]
        vy = self.x[3]
        H_x = [sqrt(px*px + py*py), atan2(py,px), (px*vx + py*vy)/sqrt(px*px + py*py)]

        y = z - H_x

        if y[1] > np.pi :
            y[1] += -2*np.pi
        elif y[1] < -np.pi :
            y[1] += 2*np.pi  

        self.x = self.x + np.dot(K, y)
        print(["SELF.X : ", round(self.x[0],2), round(self.x[1],2), round(self.x[2],2), round(self.x[3],2)])
        self.P = np.dot((np.eye(len(self.P)) - np.dot(K, H_j)), self.P)  

    # lidar와 radar를 이용하는 KF를 이용한 sensor-fusion logic 중 lidar 계측 정보에 대한 ekf update 부분을 작성
    # 대상 시스템에서는 계측 모델 (H matrix)이 비선형이기 때문에, Jacobian을 이용하여 update를 진행
    # 계측값에 대한 오차는 계측 모델을 직접 이용하여 계산하고, 그 외에는 Jacobian을 이용
 
