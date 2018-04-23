from E160_robot import *
from E160_state import *
from E160_wall import *
from E160_graph import *
import serial
import time
from xbee import XBee


class E160_environment:

    
    def __init__(self):
        self.width = 3.0
        self.height = 5.0
        
        # set up walls, putting top left point first
        self.walls = []
        self.walls.append(E160_wall([-0.65, 0.65, -0.65, -0.65],"vertical"))
        self.walls.append(E160_wall([0.65, 0.65, 0.65, -0.65],"vertical"))
        self.walls.append(E160_wall([-0.65, 0.65, 0.65, 0.65],"horizontal"))
        #self.walls.append(E160_wall([0, 0, 0, -0.65],"vertical"))
        #self.walls.append(E160_wall([0, -.1, 0, -0.5],"vertical"))
        #self.walls.append(E160_wall([0, -0.4, 1, -0.4],"horizontal"))
        # self.walls.append(E160_wall([-0.5, -0.5, 0.5, -1],"horizontal"))
        # self.walls.append(E160_wall([-0.5, -0.5, 0.0, -1.0],"vertical"))

        #initialize graph
        self.graph = E160_graph()

            
        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE"#"SIMULATION MODE" or "HARDWARE MODE"
        self.control_mode = "AUTONOMOUS CONTROL MODE"

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):
            self.serial_port = serial.Serial('COM4', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")
        
        # Setup the robots
        self.num_robots = 3
        self.robots = []
        self.graph.occupied_nodes = []
        self.batteries = []

        self.adds = []

        for i in range (0,self.num_robots):
            
            # TODO: assign different address to each bot
            r = E160_robot(self, self.graph, '\x00\x0C', i)
            self.robots.append(r)
            self.graph.occupied_nodes.append(i+1)
            self.batteries.append(r.battery_life)
            print(r)
    
    def update_robots(self, deltaT):
        
        # loop over all robots and update their state
        for r in self.robots:
            
            # set the control actuation
            r.update(deltaT, self.graph, self.batteries)
            self.graph.update_occupied_nodes(self.robots)
        
        self.update_battery_lives(self.robots)
        
        
    def log_data(self):
        
        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()
            
    def quit(self):
        self.xbee.halt()
        self.serial.close()

    def update_battery_lives(self, robots):
        self.batteries =[]
        for r in robots:
            self.batteries.append(r.battery_life)
            
            
            
            