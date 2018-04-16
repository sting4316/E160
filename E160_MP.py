import random as rand
import numpy
from E160_graph import *
import math
class E160_MP:

    def __init__(self):
        return


    def update_plan(self, graph, current_node, quadcopter_battery, quad_batteries):
        T=self.create_transition_matrix(graph, current_node, quadcopter_battery, quad_batteries)
        taus = T[current_node-1]
        print('Taus: ', taus)
        node_des = self.weighted_sample(graph.nodes, taus)
        print(node_des)

        return node_des


        

    def create_transition_matrix(self, graph, current_node, quadcopter_battery, quad_batteries ):
        #clear old data
        # Creates a 2D list that represents the transition matrix
        T = [[0 for col in range(graph.num_nodes)] for row in range(graph.num_nodes)] 

        #calculate battery probabilities for calculations
        avg_battery = numpy.mean(quad_batteries)
        battery_std = numpy.std(quad_batteries)


        #constants
        s = 0.1
        constant_sigma = 30
        battery_threshold_tour = 70
        battery_threshold_recharge = 90

        #sigmoids
        p_rel_recharge = self.CDF_logistic_distribution(quadcopter_battery, avg_battery, s)
        p_nom_recharge = self.CDF_logistic_distribution(quadcopter_battery, battery_threshold_recharge, s)

        p_rel_tour = self.CDF_logistic_distribution(quadcopter_battery, avg_battery, s)
        p_nom_tour = self.CDF_logistic_distribution(quadcopter_battery, battery_threshold_tour, s)


        #print(current_node)
        #print(graph.recharge_nodes)
        #only do probabilities at nodes quad is at
        if current_node in graph.recharge_nodes or current_node == graph.recharge_nodes:
            p_recharge = ((battery_std/constant_sigma)*(1-p_rel_recharge)) + ((1-battery_std/constant_sigma)*(1-p_nom_recharge))
            p_tour = 1-p_recharge

            #unoccupied_recharge_transitions = self.get_unoccupied_transistions(graph.edges, current_node, graph.occupied_nodes, graph.recharge_nodes)
            unoccupied_tour_transitions = self.get_unoccupied_transistions(graph.edges, current_node, graph.occupied_nodes, graph.tour_nodes)

            #print(unoccupied_recharge_transitions)
            #print(unoccupied_tour_transitions)
            if unoccupied_tour_transitions is None:
                print('RECHARGE-NO TOUR')
                T[current_node-1][current_node-1]= 1
            else:
                print('RECHARGE - TOUR')
                T[current_node-1][current_node-1] = p_recharge

                #equal probabilitiy of transitioning to any empty tour nodes
                for i in range(0, len(unoccupied_tour_transitions)):
                    T[current_node-1][unoccupied_tour_transitions[i]-1] = p_tour/(len(unoccupied_tour_transitions))

        elif current_node in graph.tour_nodes:
            p_recharge = ((battery_std/constant_sigma)*(1-p_rel_tour))+((1-battery_std/constant_sigma)*(1-p_nom_tour))
            p_tour = 1-p_recharge

            unoccupied_recharge_transitions = self.get_unoccupied_transistions(graph.edges, current_node, graph.occupied_nodes, graph.recharge_nodes)
            unoccupied_tour_transitions = self.get_unoccupied_transistions(graph.edges, current_node, graph.occupied_nodes, graph.tour_nodes)

            print('Tour Transistions: ', unoccupied_tour_transitions)
            print('Recharge Transistions: ', unoccupied_recharge_transitions)
            if unoccupied_recharge_transitions is None or unoccupied_recharge_transitions == [] and unoccupied_tour_transitions is None or unoccupied_tour_transitions == []:
                print('TOUR - NO TOUR AND NO RECHARGE')
                T[current_node-1][current_node-1] = 1


            elif unoccupied_recharge_transitions is not None  or unoccupied_recharge_transitions != [] and unoccupied_tour_transitions is None or unoccupied_tour_transitions == []:
                print('TOUR - NO TOUR BUT RECHARGE')
                T[current_node-1][current_node-1] = p_tour

                for i in range(0, len(unoccupied_recharge_transitions)):
                    T[current_node-1][unoccupied_recharge_transitions[i]-1] = p_recharge/(len(unoccupied_recharge_transitions))

            elif unoccupied_recharge_transitions is None or unoccupied_recharge_transitions == [] and unoccupied_tour_transitions is not None or unoccupied_tour_transitions != []:

                print('TOUR - TOUR BUT NO RECHARGE')
                T[current_node-1][current_node-1] = p_recharge

                for i in range(0, len(unoccupied_tour_transitions)):
                    T[current_node-1][unoccupied_tour_transitions[i]-1] = p_tour/(len(unoccupied_tour_transitions))

            else:
                print('TOUR - TOUR AND RECHARGE')
                for i in range(0, len(unoccupied_tour_transitions)):
                    
                    T[current_node-1][unoccupied_tour_transitions[i]-1] = p_tour/(len(unoccupied_tour_transitions))

                for j in range(0, len(unoccupied_recharge_transitions)):
                    T[current_node-1][unoccupied_recharge_transitions[j]-1] = p_recharge/(len(unoccupied_recharge_transitions))

        else:
            T[current_node-1][current_node-1]  = 1


        print(T)
        return T

    def weighted_sample(self, numbers, weights ):
        r = rand.random()
        total = 0
        for i in range(0, len(numbers)):
            total = total + weights[i]
            if r < total:
                rand_int = numbers[i]

                #print(rand_int)
                return rand_int


        print('Weighted Sample returned nothing')


    def CDF_logistic_distribution(self, x, mu, s):
        P = 1/(1+math.exp(-(x-mu)/s));
        return P



    def get_unoccupied_transistions(self, E, current_node, occupied_nodes, node_list):

        destination_nodes = E[current_node]
        #print(destination_nodes)
        unoccupied_nodes = self.set_diff(node_list, occupied_nodes)
        #print(occupied_nodes)
        #print(unoccupied_nodes)
        open_transitions = self.intersection(destination_nodes, unoccupied_nodes)

        return open_transitions



    def set_diff(self, a, b):
        #print(set(a))
        #print(set(b))
        #print(list(set(a) - set(b)))
        return list(set(a) - set(b))

    def intersection(self, a, b):
        #print('test:', list(set(a) & set(b)))
        return list(set(a) & set(b))