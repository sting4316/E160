from E160_state import *
from E160_PF import *
from E160_MP import *
import math
import datetime

class E160_robot:
    def __init__(self, environment, graph, address, robot_id):
        self.environment = environment
        self.graph = graph
        self.node_des = robot_id+1
        self.state_est = E160_state()
        init_xy = self.graph.get_coordinates(robot_id+1)
        self.state_est.set_state(init_xy[0], init_xy[1], 0)
        self.state_des = E160_state()
        self.state_des.set_state(init_xy[0], init_xy[1], 0)
        self.state_draw = E160_state()
        self.state_draw.set_state(init_xy[0], init_xy[1], 0)        
        self.state_odo = E160_state()
        self.state_odo.set_state(init_xy[0], init_xy[1], 0) # real position for simulation

        #self.v = 0.05
        #self.w = 0.1
        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.03
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()
        self.encoder_resolution = 1440
        
        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        
        self.Kpho = 1#1.0
        self.Kalpha = 2#2.0
        self.Kbeta = -0.5#-0.5
        self.max_velocity = 1

        self.point_tracked = False
        self.encoder_per_sec_to_rad_per_sec = 10

        #self.PF = E160_PF(environment, self.width, self.wheel_radius, self.encoder_resolution)
        self.MP = E160_MP()


        #add battery life stuff
        self.battery_life = 90
        self.discharge_rate = 0.1
        self.recharge_rate = 0.5
        self.recharge_threshold = 0.05
        
        self.current_point = -1
        self.path_tracking = False

        if self.environment.robot_mode == "SIMULATION MODE":
            self.Kpho = 3#2#1.0
            self.Kalpha = 8#10#2.0
            self.Kbeta = -1.5#-4#-0.5
            self.Kp = 2
            self.distance_threshold = 0.005
            self.angle_threshold = 0.015
            self.point_turn_threshold = 0.0005
        else:
            self.Kpho = 3#2#1.0
            self.Kalpha = 8#10#2.0
            self.Kbeta = -1.5#-4#-0.5
            self.Kp = 1.2
            self.distance_threshold = 0.05
            self.angle_threshold = 0.1
            self.point_turn_threshold = 0.01

    def update(self, deltaT, graph, batteries):
        
        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # update odometry
        delta_s, delta_theta = self.update_odometry(self.encoder_measurements)

        # update simulated real position, find ground truth for simulation
        self.state_odo = self.localize(self.state_odo, delta_s, delta_theta, self.range_measurements)

        # localize with particle filter
         # self.state_est = self.PF.LocalizeEstWithParticleFilter(self.encoder_measurements, self.range_measurements)
        self.state_est = self.update_state(self.state_est, delta_s, delta_theta)

        #update battery life of robot
        self.update_battery()

        # to out put the true location for display purposes only. 
        self.state_draw = self.state_odo

        # call motion planner
        if self.point_tracked:
            self.node_des = self.MP.update_plan(graph, self.node_des, self.battery_life, batteries)
            node_coordinates = self.graph.get_coordinates(self.node_des)
            self.state_des.set_state(node_coordinates[0],node_coordinates[1],0)
            print('xy: ', self.state_des.x, self.state_des.y)
            self.point_tracked = False

        
        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)
        
        #print(self.state_est.x, self.state_est.y, self.state_est.t)
        # self.state_est.set_state(0,0,0)
    
    
    def update_sensor_measurements(self, deltaT):
        
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]

            range_measurements = [pow((range_measurement)/(1.96*pow(10,4)), (-(1.0/1.051))) for range_measurement in range_measurements]
            for stuff in range(len(range_measurements)):
                range_measurements[stuff] = range_measurements[stuff]/100

            old_range_measurements = [0,0,0]
            for shit in range(len(range_measurements)):
                old_range_measurements[shit] = range_measurements[shit]

            range_measurements[0] = old_range_measurements[1]
            range_measurements[1] = old_range_measurements[0]

            for yee in range(len(range_measurements)):
                range_measurements[yee] = range_measurements[yee] + (self.radius/2)
                if range_measurements[yee] > 0.8:
                    range_measurements[yee] = 0.8
            range_measurements[0] = range_measurements[0] + 0.06
            range_measurements[2] = range_measurements[2] + 0.06
            range_measurements[1] = range_measurements[1] + 0.06


        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
   #         sensor1 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[0])
    #        sensor2 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[1])
     #       sensor3 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[2])
            range_measurements = [0, 0, 0]
        
        return encoder_measurements, range_measurements

        
        
    def localize(self, state_est, delta_s, delta_theta, range_measurements):
        state_est = self.update_state(state_est, delta_s, delta_theta)
    
        return state_est
    
    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
            
        return a
        
        
    def update_control(self, range_measurements):
        
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":   
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            #print('Wheel Speeds: ', desiredWheelSpeedR, desiredWheelSpeedL)
            
        return desiredWheelSpeedR, desiredWheelSpeedL
  


    def point_tracker_control(self):

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:

            
            ############ Student code goes here ############################################
            delta_x = self.state_des.x - self.state_est.x
            delta_y = self.state_des.y - self.state_est.y
            #print('Deltas: ', delta_x, delta_y)

            delta_theta = abs(self.angle_wrap(self.state_des.theta - self.state_est.theta))

            alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(delta_y, delta_x)))
            rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)

            if rho <= self.point_turn_threshold and delta_theta > self.angle_threshold:
                desiredV = 0
                desiredW = self.Kp*(self.angle_wrap(self.state_des.theta - self.state_est.theta))
            else:
                if alpha > -math.pi/2 and alpha <= math.pi/2:
                    rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)
                    alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(delta_y, delta_x)))
                    beta = self.angle_wrap(-self.state_est.theta -alpha)

                    beta = self.angle_wrap(beta + self.angle_wrap(self.state_des.theta))

                    desiredV = self.Kpho*rho
                    desiredW = self.Kalpha*alpha + self.Kbeta*beta
                    #print('Forward loop')
                else:
                    rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)
                    alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(-delta_y, -delta_x)))
                    beta = self.angle_wrap(-self.state_est.theta -alpha)

                    beta = self.angle_wrap(beta + self.angle_wrap(self.state_des.theta))

                    desiredV = -self.Kpho*rho
                    desiredW = self.Kalpha*alpha + self.Kbeta*beta
                    #print('Reverse Loop')

            # A positive omega des should result in a positive spin
            desiredRotRateR = -desiredV/self.wheel_radius + self.radius*desiredW/self.wheel_radius
            desiredRotRateL = -desiredV/self.wheel_radius - self.radius*desiredW/self.wheel_radius


            if abs(desiredRotRateR*self.wheel_radius) > self.max_velocity or abs(desiredRotRateL*self.wheel_radius) > self.max_velocity:
                #print('Max Velocity at', rho, delta_theta)
                scaling_factor = self.max_velocity/max(abs(desiredRotRateR*self.wheel_radius), abs(desiredRotRateL*self.wheel_radius))
                desiredRotRateR *= scaling_factor
                desiredRotRateL *= scaling_factor


            desiredWheelSpeedR = desiredRotRateR*(256/(5*math.pi))
            desiredWheelSpeedL = desiredRotRateL*(256/(5*math.pi))

            if rho < self.distance_threshold: #and abs(delta_theta) < self.angle_threshold:
                print('Reached Point')
                desiredWheelSpeedR = 0
                desiredWheelSpeedL = 0
                self.point_tracked = True
            
            pass 
        # the desired point has been tracked, so don't move
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0
                
        return desiredWheelSpeedR,desiredWheelSpeedL

    
    def send_control(self, R, L, deltaT):
        
        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
    
    
    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        #print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]
    
    def simulate_range_finder(self, state, sensorT):
        '''Simulate range readings, given a simulated ground truth state'''
        p = self.PF.Particle(state.x, state.y, state.theta, 0)

        return self.PF.FindMinWallDistance(p, self.environment.walls, sensorT)

    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0:^0} {1:^1} {2:^2} {3:^3} \n'.format('StateX', 'StateY', 'NodeDes', 'BatteryLife'))
        f.close()

        
        
    def log_data(self):
        f = open(self.file_name, 'a+')
        
        # edit this line to have data logging of the data you care about
        data = ['%.2f' % x for x in [self.state_est.x, self.state_est.y, self.node_des, self.battery_life]]
        
        f.write(' '.join(data) + '\n')
        f.close()
        
        
    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor =int(R*256/100) #int(L*274/100)                                                         
   


    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

         # ****************** Additional Student Code: Start ************


        # Calculate difference in movement from last time step
        diffEncoder0 = +(-encoder_measurements[0]+self.last_encoder_measurements[0]);
        diffEncoder1 = -(-encoder_measurements[1]+self.last_encoder_measurements[1]);
        
        # At the first iteration, zero out
        if abs(diffEncoder0)> 1000 or abs(diffEncoder1)> 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0

        #Localization
        wheelDistanceL = - 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder0); # Negative since this is left wheel
        
        wheelDistanceR = + 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder1); # Positive since this is right wheel
        

        # print wheelDistanceL
        # remember for next time
        self.last_encoder_measurements[0] = encoder_measurements[0];
        self.last_encoder_measurements[1] = encoder_measurements[1];

        # Calculate v x dt and w x dt
        delta_s = 0.5 * (wheelDistanceR + wheelDistanceL);
        delta_theta = -0.5 / self.radius * (wheelDistanceR - wheelDistanceL);

        
        # ****************** Additional Student Code: End ************
            
        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta

    
    
    
    def update_state(self, state, delta_s, delta_theta):
        
          # ****************** Additional Student Code: Start ************
        state.x = state.x + delta_s*math.cos(state.theta+delta_theta/2)
        state.y = state.y + delta_s*math.sin(state.theta+delta_theta/2)
        state.theta = state.theta + delta_theta
        
        
        # ****************** Additional Student Code: End ************
            
        # keep this to return the updated state
        return state



    #update battery life function
    def update_battery(self):
        old_battery_life = self.battery_life

        if self.node_des in self.graph.recharge_nodes or self.node_des == self.graph.recharge_nodes:
            if self.at_node():
                new_battery_life = old_battery_life + self.recharge_rate
                self.battery_life = min(new_battery_life, 100)
            else:
                new_battery_life = old_battery_life - self.discharge_rate
                self.battery_life = max(new_battery_life, 0)
        else:
            new_battery_life = old_battery_life - self.discharge_rate
            self.battery_life = max(new_battery_life, 0)

    def at_node(self):
        delta_x = self.state_des.x - self.state_est.x
        delta_y = self.state_des.y - self.state_est.y
        dist = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)

        if dist < self.recharge_threshold:
            return True
        else:
            return False