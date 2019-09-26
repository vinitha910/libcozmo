#!/usr/bin/env python3

from cozmopy import ObjectOrientedActionSpace

class ExplorationPolicies(object):
	def __init__(actionspace):
		self.actionspace = actionspace
		self.random = Random(self.actionspace) 

class Random:
	def __init__(actionspace):
		self.actionspace = actionspace

	def action(self, state):
		return random.choice([i for i in range(self.actionspace.size())])

class Novelty:
	def __init__(actionspace):
		self.actionspace = actionspace
		# action_id -> num times explored
		self.num_explored = {}
		self.actions = [i for i in range(self.actionspace.size())]

	def get_explored_action(self):
        if len(self.num_explored) >= self.actionspace.size():
            min_val = min(self.num_explored.values())
            min_explored = [k for k in self.num_explored.keys() if self.num_explored[k] == min_val]
            max_explored = list(set(actions).difference(min_explored))
        else:
            max_explored = self.num_explored.keys()
            min_explored = list(set(actions).difference(max_explored))

        return min_explored, max_explored

	def action(self, state):
		min_explored, max_explored = self.get_explored_action()
        if len(max_explored) == 0:
            return self.get_random_action()

        most_novel_action = None
        most_novel_action_score = 0
        for min_action in min_explored:
            action_dists = [self.actionspace.action_similarity(min_action, max_action) for max_action in max_explored]
            action_score = sum(action_dists)/len(action_dists)
            if action_score > most_novel_action_score:
                most_novel_action_score = action_score
                most_novel_action = min_action

        return self.actionspace(most_novel_action, state)