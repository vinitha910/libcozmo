#!/usr/bin/env python3

from cozmopy import ObjectOrientedActionSpace

class ExplorationPolicies(object):
	def __init__(actionspace):
		self.actionspace = actionspace
		self.random = Random(self.actionspace) 

class Random:
	def __init__(actionspace):
		self.actionspace = actionspace

	def action(self):
		return random.choice([i for i in range(self.actionspace.size())])

	def 