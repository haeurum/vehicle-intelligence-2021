import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''

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

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
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
