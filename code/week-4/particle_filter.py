import numpy as np
from helpers import distance
import math

import copy as cp

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)


    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.

        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                # print(p)

                if (min_dist < 0.0) or (dist < min_dist) :
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
                    
                association = {
                    'id': min_id,
                    'x': min_x,
                    'y': min_y,
                }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations


    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):

        for p in self.particles :
            pos_Ego = {'x':p['x'], 'y':p['y']}

            lm_fov = []
            for lm in map_landmarks :
                lm_pos = map_landmarks[lm]
                dist_lm = distance(lm_pos, pos_Ego)

                if (dist_lm <= sensor_range) :
                    lm_fov.append({'id':lm, 'x':lm_pos['x'], 'y':lm_pos['y']})

            ob_global_tgt = []
            [x_Ego, y_Ego, yaw_Ego] = [p['x'], p['y'], p['t']]
            RotMatrix_LtoG = np.matrix(([math.cos(yaw_Ego), -math.sin(yaw_Ego), x_Ego],
                        [math.sin(yaw_Ego), math.cos(yaw_Ego), y_Ego],
                        [0, 0, 1]))
          
            for ob in observations :    
                ob_local = np.array([ob['x'], ob['y'], 1])
                ob_global = np.dot(RotMatrix_LtoG, ob_local.T)
                ob_global_tgt.append({'x':ob_global[0,0], 'y':ob_global[0,1]})

            lm_asso = self.associate(lm_fov, ob_global_tgt)
            p['assoc'] = [landmark['id'] for landmark in lm_asso]

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


        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        # 5. Update the particle's weight by the calculated probability.
        pass


    # Resample particles with replacement with probability proportional to
    #   their weights.
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
        
                
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.
        pass

    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
