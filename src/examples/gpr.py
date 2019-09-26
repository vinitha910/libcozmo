#!/usr/bin/env python3

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import _pickle as pickle
import numpy as np

class GuassianProcressRegressor(object):
	def __init__(self, kernel=None, scalar=None):
        self.kernel = kernel
        self.scalar = scalar
        self.regressor = GaussianProcessRegressor(
            normalize_y=True, 
            n_restarts_optimizer = 0, 
            alpha=0.0001,
            kernel = RBF(length_scale=0.1))

    # Update regressor for (distance, angle) given data
    # \param X Input features (assumption: 2D matrix shaped n x 3)
    # \param Y Input labels (assumption: 1D vector of length n)
    # X, Y need to match in quantity
    def update_model(self, X, Y, kernel=None):
        assert(X.shape[0] == Y.shape[0])
        if self.scalar:
            scaler = StandardScaler()
            X = scaler.fit_transform(X)
        if self.kernel=='rbf' or self.kernel == 'RBF':
            rbf_feature = RBFSampler(gamma=1, random_state=1)
            X = rbf_feature.fit_transform(X)
        self.regressor = self.regressor.fit(X, Y)

    def save_models(self, directory, model_num):
        filename = directory + 'model_' + str(model_num) + '.pkl'
        pickle.dump(self.regressor, open(filename, 'wb'))

    def load_models(self, directory, model_num):
        filename = directory + 'model_' + str(model_num) + '.pkl'
        self.regressor = pickle.load(open(filename, 'rb'))

    # Predicts delta state given input
    # \param X the action to predict change in state
    def predict(self, X):
        if self.scalar:
            scaler = StandardScaler()
            X = scaler.fit_transform(X)
        if self.kernel=='rbf' or self.kernel == 'RBF':
            rbf_feature = RBFSampler(gamma=1, random_state=1)
            X = rbf_feature.fit_transform(X)
        return self.regressor.predict(X)

    def uncertainty(self, X_predict):
        prediction, std = self.regressor.predict(X_predict, return_std = True)
        return prediction, std