import math
import random
import numpy as np
import copy
import time
from E160_state import*
from scipy.stats import norm


class E160_PF:

    def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
        self.particles = []
        self.environment = environment
        self.numParticles = 1000
        
        # maybe should just pass in a robot class?
        self.robotWidth = robotWidth
        self.radius = robotWidth/2
        self.wheel_radius = wheel_radius
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.encoder_resolution = encoder_resolution
        self.FAR_READING = 1000
        
        # PF parameters
        self.IR_sigma = 0.1 # Range finder s.d
        self.odom_xy_sigma = 100   # odometry delta_s s.d
        self.odom_heading_sigma = 0.75  # odometry heading s.d
        self.particle_weight_sum = 0

        # define the sensor orientations
        self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
        self.walls = self.environment.walls
        self.last_sensor_reading = [0, 0, 0]

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
            self.SetRandomStartPos(i)
            #self.SetKnownStartPos(i)

            
    def SetRandomStartPos(self, i):
        # add student code here 
        x_pos = random.uniform(self.map_minX, self.map_maxX)
        y_pos = random.uniform(self.map_minY, self.map_maxY)
        theta = random.uniform(-math.pi, math.pi)
        weight = 1/self.numParticles
        
        self.particles.insert(i, self.Particle(x_pos, y_pos, theta, weight))
        # end student code here
        pass

    def SetKnownStartPos(self, i):
        # add student code here 
        weight = 1/self.numParticles
        self.particles.insert(i, self.Particle(0, 0, 0, weight))
        
        
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
        if encoder_measurements != self.last_encoder_measurements:
            for i in range(0, self.numParticles):
                self.Propagate(encoder_measurements, i)
                self.particles[i].weight = self.CalculateWeight(sensor_readings, self.walls, self.particles[i])
            #    print(self.particles[i].weight)


            self.last_encoder_measurements[0] = encoder_measurements[0]
            self.last_encoder_measurements[1] = encoder_measurements[1]

            #Only resample if a new sensor measurement is read
            if sensor_readings != self.last_sensor_reading:
                self.Resample()
                self.last_sensor_reading = sensor_readings
        
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
        diffEncoder0 = self.last_encoder_measurements[0] -encoder_measurements[0] + random.normalvariate(0, self.odom_xy_sigma)
        diffEncoder1 = self.last_encoder_measurements[1] - encoder_measurements[1] + random.normalvariate(0, self.odom_xy_sigma)


        if abs(diffEncoder0) > 1000 or abs(diffEncoder1) > 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0
            
        wheelDistanceR = (diffEncoder0*self.wheel_circumference)/self.encoder_resolution
        wheelDistanceL = (diffEncoder1*self.wheel_circumference)/self.encoder_resolution

        delta_s =(wheelDistanceL+wheelDistanceR)/2 
        delta_theta = (wheelDistanceR-wheelDistanceL)/(2*self.radius) 

        delta_x = delta_s*math.cos(self.particles[i].heading + delta_theta/2)
        delta_y = delta_s*math.sin(self.particles[i].heading + delta_theta/2)

        newX = self.particles[i].x + delta_x
        newY = self.particles[i].y + delta_y
        newTheta = self.angleDiff(self.particles[i].heading + delta_theta)
        #print(newX)

        self.particles[i].x = newX
        self.particles[i].y = newY
        self.particles[i].heading = newTheta
        #print(self.particles[i].x)
        #print(self.particles[i].y)
        #print(self.particles[i].heading)
        #print('--------------------------')

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
        expected_IR_reading_left = self.FindMinWallDistance(particle, walls, self.sensor_orientation[0])
        expected_IR_reading_front = self.FindMinWallDistance(particle, walls, self.sensor_orientation[1])
        expected_IR_reading_right = self.FindMinWallDistance(particle, walls, self.sensor_orientation[2])
        newWeight_left = max(norm.pdf(sensor_readings[0], expected_IR_reading_left, self.IR_sigma), 0.001)
        newWeight_front = max(norm.pdf(sensor_readings[1], expected_IR_reading_front, self.IR_sigma), 0.001)
        newWeight_right = max(norm.pdf(sensor_readings[2], expected_IR_reading_right, self.IR_sigma), 0.001)

        newWeight = (newWeight_left +newWeight_right + newWeight_front)/3
        #print(newWeight)
        
        
        # end student code here
        return newWeight

    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''
        # add student code here 
        '''
        w_tot = max(map(lambda x: x.weight, self.particles))
        X_temp = []
        for i in range(self.numParticles):
            w_i = self.particles[i].weight/w_tot
            #print(w_i)

            if w_i < 0.2:
                for j in range(1):
                    X_temp += [self.particles[i]]
            if w_i < 0.4:
                for j in range(2):
                    X_temp += [self.particles[i]]
            elif w_i < 0.7:
                for j in range(30):
                    X_temp += [self.particles[i]]
            elif w_i < 0.9:
                for j in range(80):
                    X_temp += [self.particles[i]]
            elif w_i < .93:
                for j in range(90):
                    X_temp += [self.particles[i]]
            elif w_i < .95:
                for j in range(100):
                    X_temp += [self.particles[i]]
            elif w_i <= 1.00:
                for j in range(200):
                    X_temp += [self.particles[i]]

        for i in range(self.numParticles):
            r = int(random.uniform(0, len(X_temp)-1))
            self.particles[i].x = X_temp[i].x
            self.particles[i].y = X_temp[i].y
            self.particles[i].heading = X_temp[i].heading
            self.particles[i].weight = X_temp[i].weight

        print(len(self.particles))
        '''
        w_tot = sum(list(map(lambda x: x.weight, self.particles)))
        old_particles = self.particles

        for i in range(self.numParticles):
            r = random.random()*w_tot
            j = 0

            w_sum = old_particles[j].weight
            while w_sum < r:
                j = j +1
                w_sum = w_sum + old_particles[j].weight
            self.particles[i].x = old_particles[j].x
            self.particles[i].y = old_particles[j].y
            self.particles[i].heading = old_particles[j].heading
            self.particles[i].weight = old_particles[j].weight
            
        # end student code here
        return
        



    def GetEstimatedPos(self):
        ''' Calculate the mean of the particles and return it 
            Args:
                None
            Return:
                None'''
        # add student code here 
        xavg = 0
        yavg = 0
        thetaavg = 0
        
        for parts in self.particles:
            xavg += parts.x
            yavg += parts.y
            thetaavg += parts.heading

        xavg = xavg/self.numParticles
        yavg = yavg/self.numParticles
        thetaavg = thetaavg/self.numParticles
        
        state = E160_state()
        state.set_state(xavg, yavg, thetaavg)
        # end student code here
        
        return state



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
        #shortest = self.FindWallDistance(particle, walls[0], sensorT)

        shortest = self.FAR_READING
        for item in walls:
            x = self.FindWallDistance(particle, item, sensorT)
            #print(item, x)
            if abs(x) < abs(shortest):
                shortest = abs(x)

        #print(shortest)

        # end student code here
        
        return shortest
    

    def FindWallDistance(self, particle, wall, sensorT):
        ''' Given a particle position, a wall, and a sensor, find distance to the wall
            Args:
                particle (E160_Particle): a particle 
                wall ([float x4]): represents endpoint of the wall 
                sensorT: orientation of the sensor on the robot
            Return:
                distance to the closest wall (float)'''

        # add student code here 
        #ref dot products
        #https://stackoverflow.com/questions/4030565/line-and-line-segment-intersection?rq=1
        
        wall_points = self.getWallPoints(wall)
        
        p = np.array([particle.x, particle.y])
        r = np.array([math.cos(self.angleDiff(particle.heading + sensorT)), math.sin(self.angleDiff(particle.heading + sensorT))])

        q = np.array([ wall_points[0], wall_points[1]])
        s = np.array([wall_points[2]-wall_points[0], wall_points[3]-wall_points[1]])

        if np.cross(r, s) != 0:
            t = np.cross((q-p), s)/np.cross(r, s)
            u = np.cross((q-p), r)/np.cross(r, s)
        else:
            return  self.FAR_READING

        #calculate point of intersection
        if t >= 0 and t<= 1 and u >= 0 and u<= 1:
            intersection = p + t*r
            xinter = intersection[0]
            yinter = intersection[1]

            #calculate distance from particle to wall
            xsq = pow((particle.x - xinter), 2)
            ysq = pow((particle.y - yinter), 2)
            distance = pow((xsq + ysq), .5)
        else:
            return self.FAR_READING
        

        

            # end student code here
                    
        return distance

    def getWallPoints(self, wall):
        slope = wall.slope

        if slope == "vertical":
            X1 = wall.points[0]+wall.radius
            Y1 = wall.points[1]-wall.radius
            X2 = wall.points[4]-wall.radius
            Y2 = wall.points[5]+wall.radius

            wall_points = [X1, Y1, X2, Y2]
        
        # assume left point is first
        elif slope == "horizontal":
            X1 = wall.points[0]+wall.radius
            Y1 = wall.points[1]+wall.radius
            X2 = wall.points[4]-wall.radius
            Y2 = wall.points[5]-wall.radius

            wall_points = [X1, Y1, X2, Y2]
        return wall_points

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



