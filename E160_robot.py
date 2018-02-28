
from E160_state import *
import math
import time
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)
        #self.v = 0.05
        #self.w = 0.1
        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.03
        self.wheel_circumference = 2*math.pi*self.wheel_radius
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
        
        self.Kpho = 1#3.79#1.0
        self.Kalpha = 2#3.8#2.0
        self.Kbeta = -.5#-4#-0.5
        self.max_velocity = 0.1
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        self.current_point = -1
        self.path_tracking = False

        if self.environment.robot_mode == "SIMULATION MODE":
            self.distance_threshold = 0.005
            self.angle_threshold = 0.008
        else:
            self.distance_threshold = 0.05
            self.angle_threshold = 0.1
        
        
    def update(self, deltaT):
        
        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # localize
        self.state_est = self.localize(self.state_est, self.encoder_measurements, self.range_measurements)
        
        # call motion planner
        #self.motion_planner.update_plan()
        
        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)
    
    
    def update_sensor_measurements(self, deltaT):
        
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]
            
        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            range_measurements = [0,0,0]
        
        return encoder_measurements, range_measurements

        
        
    def localize(self, state_est, encoder_measurements, range_measurements):
        delta_s, delta_theta = self.update_odometry(encoder_measurements)
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
            if self.path_tracking:
                self.path_tracker()   

            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            
        return desiredWheelSpeedR, desiredWheelSpeedL


    def path_tracker(self):
        point_list = [(0.2, 0, 0),
                        (.4, .2, 0),
                        (0, -.2, math.pi/2)]

        if self.point_tracked:
            if self.current_point + 1 != len(point_list):
                self.current_point += 1
                time.sleep(.5)

            print('Updating Des')
            print(point_list[self.current_point])

            self.state_des.set_state(point_list[self.current_point][0], point_list[self.current_point][1], point_list[self.current_point][2])
            self.point_tracked = False

        return


    def point_tracker_control(self):

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:

            
            ############ Student code goes here ############################################
            delta_x = self.state_des.x - self.state_est.x
            delta_y = self.state_des.y - self.state_est.y

            delta_theta = abs(self.angle_wrap(self.state_des.theta - self.state_est.theta))

            alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(delta_y, delta_x)))

            #print('Pre Alpha', alpha)


            if alpha > -math.pi/2 and alpha <= math.pi/2:
                rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)
                alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(delta_y, delta_x)))
                beta = self.angle_wrap(-self.state_est.theta -alpha)

                beta = self.angle_wrap(beta + self.angle_wrap(self.state_des.theta))

                desiredV = self.Kpho*rho
                desiredW = self.Kalpha*alpha + self.Kbeta*beta
                print('Forward loop')
            else:
                rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)
                alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(-delta_y, -delta_x)))
                beta = self.angle_wrap(-self.state_est.theta -alpha)

                beta = self.angle_wrap(beta + self.angle_wrap(self.state_des.theta))

                desiredV = -self.Kpho*rho
                desiredW = self.Kalpha*alpha + self.Kbeta*beta
                print('Reverse Loop')

            # A positive omega des should result in a positive spin
            desiredRotRateR = -desiredV/self.wheel_radius + self.radius*desiredW/self.wheel_radius
            desiredRotRateL = -desiredV/self.wheel_radius - self.radius*desiredW/self.wheel_radius


            if abs(desiredRotRateR*self.wheel_radius) > self.max_velocity or abs(desiredRotRateL*self.wheel_radius) > self.max_velocity:
                scaling_factor = self.max_velocity/max(abs(desiredRotRateR*self.wheel_radius), abs(desiredRotRateL*self.wheel_radius))
                desiredRotRateR *= scaling_factor
                desiredRotRateL *= scaling_factor


            desiredWheelSpeedR = desiredRotRateR*(256/(5*math.pi))
            desiredWheelSpeedL = desiredRotRateL*(256/(5*math.pi))

            print('R Wheel Speed: ', desiredWheelSpeedR)
            print('L Wheel Speed: ', desiredWheelSpeedL)
            print('Alpha', alpha)
            print('Beta', beta)
            #print('Distance to point: ', rho)
            print('Delta Theta: ', delta_theta)
            if rho < self.distance_threshold and abs(delta_theta) < self.angle_threshold:
                desiredWheelSpeedR = 0
                desiredWheelSpeedL = 0
                self.point_tracked = True
            
            
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
    
        
    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} \n'.format('V', 'W', 'RW', 'LW', 'A', 'B'))
        f.close()

        
        
    def log_data(self):
        f = open(self.file_name, 'a+')
        
        # edit this line to have data logging of the data you care about
        data = [str(x) for x in []]
        
        f.write(' '.join(data) + '\n')
        f.close()
        
        
    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)                                                         
   


    def update_odometry(self, encoder_measurements):

         # ****************** Additional Student Code: Start ************
        #Left Wheel = 69.18 mm
        #Right Wheel = 69.13 mm
        #Average Wheel = 69.155 +- 0.01 mm

        diffEncoder0 = encoder_measurements[0] - self.last_encoder_measurements[0]
        diffEncoder1 = encoder_measurements[1] - self.last_encoder_measurements[1]

        self.last_encoder_measurements[0] = encoder_measurements[0]
        self.last_encoder_measurements[1] = encoder_measurements[1]

        if diffEncoder0 > 1000 or diffEncoder1 > 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0
            
        wheelDistanceR = (diffEncoder0*self.wheel_circumference)/self.encoder_resolution
        
        wheelDistanceL = (diffEncoder1*self.wheel_circumference)/self.encoder_resolution

        delta_s =(wheelDistanceL+wheelDistanceR)/2

        delta_theta = (wheelDistanceR-wheelDistanceL)/self.radius


        # ****************** Additional Student Code: End ************
            
        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta 

    
    
    
    def update_state(self, state, delta_s, delta_theta):
        
         # ****************** Additional Student Code: Start ************
        
        delta_x = delta_s*math.cos(state.theta + delta_theta/2)
        delta_y = delta_s*math.sin(state.theta + delta_theta/2)

        newX = state.x + delta_x
        newY = state.y + delta_y
        newTheta = self.normalizeAngle(state.theta + delta_theta)

        state.set_state(newX, newY, newTheta)
        
        # ****************** Additional Student Code: End ************


        # keep this to return the updated state
        return state

    def normalizeAngle(self, angle):

        newAngle = angle
        while (newAngle <= -math.pi):
            newAngle += 2*math.pi
        while (newAngle > math.pi):
            newAngle -= 2*math.pi
        return newAngle
        
        
        
        
        
        
        