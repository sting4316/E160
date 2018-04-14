import random
import numpy
class E160_MP:

	def __init__(self ):

	def update_plan(self, quadcopter_battery, quad_batteries):

		

	def create_transition_matrix(self, E, current_node, num_nodes, occupied_nodes, tour_nodes, recharge_nodes, quadcopter_battery, quad_batteries ):
		#clear old data
		for x in range(num_nodes):
			for y in range(num_nodes):
				T = 0

		#calculate battery probabilities for calculations
		avg_battery = numpy.mean(quad_batteries)
		battery_std = numpy.std(quad_batteries)


		#constants
		s = 0.1
		constant_sigma - 30
		battery_threshold_tour = 70
		battery_threshold_recharge = 90

		#sigmoids
		p_rel_recharge = CDF_logistic_distribution(quadcopter_battery, avg_battery, s)
		p_nom_recharge = CDF_logistic_distribution(quadcopter_battery, battery_threshold_recharge, s)

		p_rel_tour = CDF_logistic_distribution(quadcopter_battery, avg_battery, s)
		p_nom_tour = CDF_logistic_distribution(quadcopter_battery, battery_threshold_tour, s)


		#only do probabilities at nodes quad is at
		if current_node in recharge_nodes:
			p_recharge = ((battery_std/constant_sigma)*(1-p_rel_recharge)) + ((1-battery_std/constant_sigma)*(1-p_nom_recharge))
			p_tour = 1-p_recharge

			unoccupied_recharge_transitions = 
			unoccupied_tour_transitions = 

			if len(unoccupied_tour_transitions) == 0:
				T(current_node, current_node) = 1
			else:
				T(current_node, current_node) = p_recharge

				#equal probabilitiy of transitioning to any empty tour nodes
				for i in range(0, len(unoccupied_tour_transitions)-1):
					T(current_node, unoccupied_tour_transitions[i]) = p_tour/(len(unoccupied_tour_transitions)-1)

		elif current_node in tour_nodes:
			p_recharge = ((battery_std/constant_sigma)*(1-p_rel_tour))+((t-battery_std/constant_sigma)*(1-p_nom_tour))
			p_tour = 1-p_recharge

			unoccupied_recharge_transitions = get_unoccupied_transistions(E, current_node, occupied_nodes, recharge_nodes)
			unoccupied_tour_transitions = get_unoccupied_transistions(E, current_node, occupied_nodes, tour_nodes)

			if len(unoccupied_recharge_transitions)==0 and len(unoccupied_tour_transitions)==0:
        		T(current_node, current_node) = 1


			elif len(unoccupied_recharge_transitions)!=0 and len(unoccupied_tour_transitions)==0
        		T(current_node, current_node) = p_tour;
				for i in range(0, len(unoccupied_recharge_transitions)):
					T(current_node, unoccupied_recharge_transitions(i)) = p_recharge/(len(unoccupied_tour_transitions)-1)

			elif len(unoccupied_recharge_transitions)==0 and len(unoccupied_tour_transitions)!=0
        		T(current_node, current_node) = p_tour;
				for i in range(0, len(unoccupied_recharge_transitions)):
					T(current_node, unoccupied_recharge_transitions(i)) = p_recharge/(len(unoccupied_tour_transitions)-1)

			else:
				for i in range(0, len(unoccupied_tour_transitions)-1)
					T(current_node, unoccupied_tour_transitions(i)) = p_tour/(len(unoccupied_tour_transitions)-1)

				for i in range(0, len(unoccupied_recharge_transitions)-1)
					T(current_node, unoccupied_recharge_transitions(i)) = p_recharge/(len(unoccupied_recharge_transitions)-1)

		else:
			T(current_node, current_node) = 1


		return T

	def weighted_sample(self, numbers, weights ):
		r = rand.random()
		total = 0
		for i in range(0, len(numbers)-1):
			total = total + weights[i]
			if r < total:
				rand_int = numbers(i)
				break


	def CDF_logistic_distribution(x,mu, s):
		P = 1./(1+exp(-(x-mu)./s));
		return P



	def get_unoccupied_transistions(E, current_node, occupied_nodes, node_list):

		destination_nodes = E[current_node]
		unoccupied_nodes = set_diff(node_list, occupied_nodes)
		open_transitions = intersection(destination_nodes, unoccupied_nodes)



	def set_diff(a, b):
  		return list(set(a) - set(b))

	def intersection(a, b):
    	return list(set(a) & set(b))