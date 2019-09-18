from sklearn.gaussian_process import GaussianProcessRegressor
import cPickle as pickle

def load_model(filename):
	return pickle.load(open(filename, 'rb'))