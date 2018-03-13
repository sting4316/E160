import math
import random
import numpy as np
import copy
from E160_state import*
from scipy.stats import norm



class E160_PF:

	def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
		self.particles = []
		self.environment = environment
		self.numParticles = 400
		
		# maybe should just pass in a robot class?
		self.robotWidth = robotWidth
		self.radius = robotWidth/2
		self.wheel_radius = wheel_radius
		self.encoder_resolution = encoder_resolution
		self.wheel_circumference = self.wheel_radius*2*math.pi
		self.FAR_READING = 1000
		
		# PF parameters
		self.IR_sigma = 0.2 # Range finder s.d
		self.odom_xy_sigma = 1.25	# odometry delta_s s.d
		self.odom_heading_sigma = 0.75	# odometry heading s.d
		self.particle_weight_sum = 0

		# define the sensor orientations
		self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
		self.walls = self.environment.walls

		# initialize the current state
		self.state = E160_state()
		self.state.set_state(0,0,0)

		# TODO: change this later
		self.map_maxX = 1.0
		self.map_minX = -1.0
		self.map_maxY = 1.0
		self.map_minY = -1.0
		self.InitializeParticles()
		self.last_encoder_measurements =[0,0]

	def InitializeParticles(self):
		''' Populate self.particles with random Particle 
			Args:
				None
			Return:
				None'''
		self.particles = []
		for i in range(0, self.numParticles):
			#self.SetRandomStartPos(i)
			self.SetKnownStartPos(i)

			
	def SetRandomStartPos(self, i):
		# add student code here 
        x_pos = random.uniform(self.map_minX, self.map_maxX)
        y_pos = random.random(self.map_minY, self.map_maxY)
        theta = random.random(-math.pi, math.pi)
        weight = 1/self.numParticles
        
        self.particles(i, Particle(x_pos, y_pos, theta, weight))
        # end student code here
        pass

	def SetKnownStartPos(self, i):
		# add student code here 
		weight = 1/self.numParticles
        self.particles.insert(i, Particle(0, 0, 0, weight))
        
        
        # end student code here
        pass
            
	def LocalizeEstWithParticleFilter(self, encoder_measurements, sensor_readings):
		''' Localize the robot with particle filters. Call everything
			Args: 
				delta_s (float): change in distance as calculated by odometry
				delta_heading (float): change in heading as calcualted by odometry
				sensor_readings([float, float, float]): sensor readings from range fingers
			Return:
				None'''
		
        # add student code here 
		for i in range(0, self.numParticles):
			Propagate(encoder_measurements, i)
			self.particles[i].weight = CalculateWeight(sensor_readings, self.walls, i)

		# Only resample if a valid sensor measurement is read
		if sensor_readings[1] < self.FAR_READING:
			Resample()
        
        
        # end student code here
        
        
		return self.GetEstimatedPos()

	def Propagate(self, encoder_measurements, i):
		'''Propagate all the particles from the last state with odometry readings
			Args:
				delta_s (float): distance traveled based on odometry
				delta_heading(float): change in heading based on odometry
			return:
				nothing'''
        # add student code here 
        diffEncoder0 = encoder_measurements[0] - self.last_encoder_measurements[0] 
        diffEncoder1 = encoder_measurements[1] - self.last_encoder_measurements[1]

        self.last_encoder_measurements[0] = encoder_measurements[0]
        self.last_encoder_measurements[1] = encoder_measurements[1]

        if abs(diffEncoder0) > 1000 or abs(diffEncoder1) > 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0
            
        wheelDistanceR = (diffEncoder0*self.wheel_circumference)/self.encoder_resolution
        wheelDistanceL = (diffEncoder1*self.wheel_circumference)/self.encoder_resolution

        delta_s =(wheelDistanceL+wheelDistanceR)/2 + random.normalvariate(0, self.odom_xy_sigma)
        delta_theta = (wheelDistanceR-wheelDistanceL)/(2*self.radius) + random.normalvariate(0, self.odom_heading_sigma)

        delta_x = delta_s*math.cos(state.theta + delta_theta/2)
        delta_y = delta_s*math.sin(state.theta + delta_theta/2)

        newX = state.x + delta_x
        newY = state.y + delta_y
        newTheta = self.normalizeAngle(state.theta + delta_theta)

        self.particles[i].x = newX
        self.particles[i].y = newY
        self.particles[i].heading = newTheta

        # end student code here
        return
        
        
	def CalculateWeight(self, sensor_readings, walls, particle):
		'''Calculate the weight of a particular particle
			Args:
				particle (E160_Particle): a given particle
				sensor_readings ( [float, ...] ): readings from the IR sesnors
				walls ([ [four doubles], ...] ): positions of the walls from environment, 
							represented as 4 doubles 
			return:
				new weight of the particle (float) '''

		
        # add student code here 

        # Use front IR sensor
        expected_IR_reading = FindMinWallDistance(i, walls, self.sensor_orientation[1])
        newWeight = norm.pdf(sensor_readings[1], expected_IR_reading, IR_sigma)
        
        
        # end student code here
		return newWeight

	def Resample(self):
		'''Resample the particles systematically
			Args:
				None
			Return:
				None'''
        # add student code here 
        w_tot = max(map(lambda x: x[-1], self.particles))
        X_temp = []
        for i in range(numParticles):
        	w_i = self.particles[i].weight/w_tot

        	if w_i < 0.25:
        		X_temp.extend([self.particles[i]]*1)
        	else if w_i < 0.5:
        		X_temp.extend([self.particles[i]]*2)
        	else if w_i < 0.75:
        		X_temp.extend([self.particles[i]]*3)
        	else if w_i < 1.00:
        		X_temp.extend([self.particles[i]]*4)

        for i in range(numParticles):
        	r = int(random.uniform(0, len(X_temp)))
        	self.particles[i] = X_temp[r]
        
        # end student code here
        return
        



	def GetEstimatedPos(self):
		''' Calculate the mean of the particles and return it 
			Args:
				None
			Return:
				None'''
        # add student code here 
        
        
        
        # end student code here
        
		return self.state


	def FindMinWallDistance(self, particle, walls, sensorT):
		''' Given a particle position, walls, and a sensor, find 
			shortest distance to the wall
			Args:
				particle (E160_Particle): a particle 
				walls ([E160_wall, ...]): represents endpoint of the wall 
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall' (float)'''
        # add student code here 
        
        
        
        # end student code here
        
		return 0
    

	def FindWallDistance(self, particle, wall, sensorT):
		''' Given a particle position, a wall, and a sensor, find distance to the wall
			Args:
				particle (E160_Particle): a particle 
				wall ([float x4]): represents endpoint of the wall 
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall (float)'''
		# add student code here 
        
        
        
        # end student code here
        		
		return 0

	

	def angleDiff(self, ang):
		''' Wrap angles between -pi and pi'''
		while ang < -math.pi:
			ang = ang + 2 * math.pi
		while ang > math.pi:
			ang = ang - 2 * math.pi
		return ang

	class Particle:
		def __init__(self, x, y, heading, weight):
			self.x = x
			self.y = y
			self.heading = heading
			self.weight = weight

		def __str__(self):
			return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)



