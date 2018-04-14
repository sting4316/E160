class E160_graph:

	def __init__ (self):

		scale = 1

		c1 = .25*(pow(5, .5) - 1)
		c2 = .25*(pow(5, .5) + 1)
		s1 = .25*pow((10 + 2*pow(5, .5)), .5)
		s2 = .25*pow((10 - 2*pow(5, .5)), .5)

		self.nodes = (1, 2, 3, 4, 5, 6)
		self.node_coordinates = {1: (0, 0), 2: (0, scale), 3: (s1, c1), 4: (s2, -c2), 5: (-s2, -c2), 6: (-s1, c1)}
		self.edges = {1: (2, 3, 4, 5, 6), 2: (1, 3), 3: (1, 4), 4: (1, 5), 5: (1, 6), 6: (1, 2)} 
		self.num_nodes = len(self.node_coordinates)
		self.recharge_nodes = (1)
		self.tour_nodes = (2, 3, 4, 5, 6)

		