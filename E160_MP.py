import random

class E160_MP:

	def __init__(self ):

	def update_plan(self, quadcopter_battery, quad_batteries ):

	def create_transition_matrix(self, E, current_node, num_nodes, occupied_nodes, tour_nodes, recharge_nodes, quadcopter_battery, quad_batteries ):

	def weighted_sample(self, numbers, weights ):
		r = rand.random()
		total = 0
		for i in range(0, len(numbers)-1):
			total = total + weights[i]
			if r < total:
				rand_int = numbers(i)
				break
