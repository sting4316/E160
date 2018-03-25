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
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.encoder_resolution = encoder_resolution
        self.FAR_READING = 1000
        
        # PF parameters
        self.IR_sigma = 0.2 # Range finder s.d
        self.odom_xy_sigma = 1.25   # odometry delta_s s.d
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
            #self.SetRandomStartPos(i)
            self.SetKnownStartPos(i)

            
    def SetRandomStartPos(self, i):
        # add student code here 
        x_pos = random.uniform(self.map_minX, self.map_maxX)
        y_pos = random.random(self.map_minY, self.map_maxY)
        theta = random.random(-math.pi, math.pi)
        weight = 1/self.numParticles
        
        self.particles(i, self.Particle(x_pos, y_pos, theta, weight))
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
        #for i in range(0, self.numParticles):
        #    self.Propagate(encoder_measurements, i)
        #    self.particles[i].weight = self.CalculateWeight(sensor_readings, self.walls, self.particles[i])

        # Only resample if a valid sensor measurement is read
        #if sensor_readings[1] != self.robot.last_measuremnt:
        #    self.Resample()
        
        
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
        diffEncoder0 = self.last_encoder_measurements[0] -encoder_measurements[0] 
        diffEncoder1 = self.last_encoder_measurements[1] - encoder_measurements[1] 

        self.last_encoder_measurements[0] = encoder_measurements[0]
        self.last_encoder_measurements[1] = encoder_measurements[1]

        if abs(diffEncoder0) > 1000 or abs(diffEncoder1) > 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0
            
        wheelDistanceR = (diffEncoder0*self.wheel_circumference)/self.encoder_resolution
        wheelDistanceL = (diffEncoder1*self.wheel_circumference)/self.encoder_resolution

        delta_s =(wheelDistanceL+wheelDistanceR)/2 #+ random.normalvariate(0, self.odom_xy_sigma)
        delta_theta = (wheelDistanceR-wheelDistanceL)/(2*self.radius) #+ random.normalvariate(0, self.odom_heading_sigma)

        delta_x = delta_s*math.cos(self.state.theta + delta_theta/2)
        delta_y = delta_s*math.sin(self.state.theta + delta_theta/2)

        newX = self.state.x + delta_x
        newY = self.state.y + delta_y
        newTheta = self.angleDiff(self.state.theta + delta_theta)

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
        expected_IR_reading = self.FindMinWallDistance(particle, walls, self.sensor_orientation[1])
        newWeight = norm.pdf(sensor_readings[1], expected_IR_reading, self.IR_sigma)
        print(sensor_readings[1])
        
        
        # end student code here
        return newWeight

    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''
        # add student code here 
        w_tot = max(map(lambda x: x.weight, self.particles))
        X_temp = []
        for i in range(self.numParticles):
            w_i = self.particles[i].weight/w_tot

            if w_i < 0.25:
                for j in range(1):
                    X_temp += [self.particles[i]]
            elif w_i < 0.5:
                for j in range(2):
                    X_temp += [self.particles[i]]
            elif w_i < 0.75:
                for j in range(3):
                    X_temp += [self.particles[i]]
            elif w_i <= 1.00:
                for j in range(4):
                    X_temp += [self.particles[i]]

        for i in range(self.numParticles):
            r = int(random.uniform(0, len(X_temp)-1))
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
        
        self.x = xavg
        self.y = yavg
        self.heading = thetaavg
        # end student code here
        
        return self.state
        
        
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
        shortest = self.FindWallDistance(particle, walls[0], sensorT)


        for item in walls:
            x = self.FindWallDistance(particle, item, sensorT)
            #print(item, x)
            if abs(x) < abs(shortest):
                shortest = abs(x)

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
        
        #i'm kinda confused on the wall notation but i am assuming it just goes 
        # [X1, Y1, X2, Y2] like the manual says?

        #ref dot products
        #https://stackoverflow.com/questions/4030565/line-and-line-segment-intersection?rq=1
        
        wall_points = self.getWallPoints(wall)
        o = np.array([particle.x, particle.y])
        a = np.array([wall_points[0], wall_points[1]])
        b = np.array([wall_points[2], wall_points[3]])
        v_1 = o-a
        v_2 = b-a
        v_3 = np.array([-math.sin(self.angleDiff(particle.heading + sensorT)), math.cos(self.angleDiff(particle.heading + sensorT))])
        print(particle.heading)

        t_1 = np.linalg.norm(np.cross(v_1, v_2))/np.dot(v_2, v_3)
        t_2 = np.dot(v_1, v_3)/np.dot(v_2, v_3)

        if t_1 >= 0 and t_2 >= 0 and t_2 <= 1:
            intersection = a + (b-a)*t_2
            xinter = intersection[0]
            yinter = intersection[1]

            #calculate distance from particle to wall
            xsq = pow((particle.x - xinter), 2)
            ysq = pow((particle.y - yinter), 2)
            distance = pow((xsq + ysq), .5)
        else:
            distance = self.FAR_READING
        #create constants
        #slope of line segment of wall
        #p = np.array([particle.x, particle.y])
        #r = np.array([math.cos(self.angleDiff(particle.heading + sensorT)), math.sin(self.angleDiff(particle.heading + sensorT))])
        #print(r)

        #q = np.array([ wall.points[0], wall.points[1]])
        #s = np.array([wall.points[2]-wall.points[0], wall.points[3]-wall.points[1]])

        #if np.cross(r, s) != 0:
        #    t = np.cross((q-p), s/np.cross(r, s))
        #    u = np.cross((q-p), r/np.cross(r, s))
        #else:
        #    return 0 

        #calculate point of intersection
        #intersection = p + t*r

        

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



