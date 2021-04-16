# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.





def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):

        # 각 파티클에 대해서 계측값과 맵의 Landmark에 대한 probability를 계산

        for p in self.particles :
            pos_Ego = {'x':p['x'], 'y':p['y']}


        # Step 1. 각 파티클에서 계측 가능한 랜드마트 추출
        # 센서 계측 범위 내 랜드마트를 추가
        # lm_fov : 계측 범위 내 랜드마크 list
            lm_fov = []
            for lm in map_landmarks :
                lm_pos = map_landmarks[lm]
                dist_lm = distance(lm_pos, pos_Ego)

                if (dist_lm <= sensor_range) :
                    lm_fov.append({'id':lm, 'x':lm_pos['x'], 'y':lm_pos['y']})
            

        # Step 2. Local 기준의 Observation data (센서 계측 정보)를 랜드마크와 비교하기 위해 Global기준 좌표 변환
        # 각 파티클의 상태를 좌표변환을 위한 자차량 상태(위치, 각도)로 사용 
        # Local to Global 좌표 변환 matrix 설정
        # association function 사용을 위한 변수 정의 (ID, X, Y 포함 list)
            ob_global_tgt = []
            [x_Ego, y_Ego, yaw_Ego] = [p['x'], p['y'], p['t']]
            RotMatrix_LtoG = np.matrix(([math.cos(yaw_Ego), -math.sin(yaw_Ego), x_Ego],
                        [math.sin(yaw_Ego), math.cos(yaw_Ego), y_Ego],
                        [0, 0, 1]))
          
            for ob in observations :    
                ob_local = np.array([ob['x'], ob['y'], 1])
                ob_global = np.dot(RotMatrix_LtoG, ob_local.T)
                ob_global_tgt.append({'x':ob_global[0,0], 'y':ob_global[0,1]})

        # Step 3. association function 활용하여, 계측 랜드마크와 맵 랜드마크 정보와 대응되는 랜드마크 추출
        # association function에서 계측된 센서 정보와 가장 가까운 랜드마크 정보(ID)를 출력
            lm_asso = self.associate(lm_fov, ob_global_tgt)
            p['assoc'] = [landmark['id'] for landmark in lm_asso]


        # Step 4. 각 파티클에서의 계측 랜드마크 위치와 맵 랜드마크 위치의 차이를 통해 probability 계산
        # 표준 정규분포를 이용해, X,Y에 대한 확률 각각 계산
        # 계측된 랜드마크에 대한 X,Y probability를 곱하여 전체 probability 계산

        # Step 5. 각 파티클의 weight와 associate된 landmark 정보를 갱신/적용함
            x_pdf = 1
            y_pdf = 1
            p_pdf = 1
            pdf_sum = 0

            tgt = 0
            for lm_fov in lm_asso :
                mu = [lm_fov['x'], lm_fov['y']]

                def norm_pdf(x, m, s):
                    one_over_sqrt_2pi = 1 / math.sqrt(2 * math.pi)
                    return (one_over_sqrt_2pi / s) * math.exp(-0.5 * ((x - m) / s) ** 2)

                lm_tgt_x = ob_global_tgt[tgt]['x']
                lm_tgt_y = ob_global_tgt[tgt]['y']

                x_pdf *= norm_pdf(lm_tgt_x, mu[0], std_landmark_x)
                y_pdf *= norm_pdf(lm_tgt_y, mu[1], std_landmark_y)
                p_pdf *= x_pdf*y_pdf
                pdf_sum += p_pdf
                tgt += 1
            p['w'] = p_pdf
            
            
def resample(self):

        weights = [p['w'] for p in self.particles]
        w_cumsum = np.cumsum(weights)
        w_mean = np.sum(weights) / len(weights)
        weight_idx = np.zeros(len(weights), dtype=np.int8)
        
        w_pointer = 0.0
        i = w_idx = 0
        while i > len(weights):
            if w_cumsum[w_idx] >= w_pointer:
                weight_idx[i] = w_idx
            else:
                weight_idx[i] = w_idx
                w_idx += 1
            w_pointer += w_mean
            i += 1
            
        new_particles = [self.particles[i].copy() for i in weight_idx]

        self.particles = []
        self.particles = new_particles.copy()   
