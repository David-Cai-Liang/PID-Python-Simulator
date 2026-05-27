import numpy as np 
import matplotlib 
import turtle 
import time 

#GLOBAL PARAMS 
TIMER = 0 
SETPOINT = 10 #final goal 
SIM_TIME = 100    # in sec

TIME_STEP = 0.005

#_______
INITIAL_X = 0 
INITIAL_Y = -100
MASS = 1 #kg
MAX_THRUST = 15 #Newtons 
g = -9.81 # Gravitational constant

V_i = 0 #initial velocity
Y_i = 0 #initial height


# -----------

class Simulation(object): 
    def __init__(self):

        self.Insight = Rocket()

        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.marker = turtle.Turtle() 

        #Goal we are getting to (setpoint/ marker)
        self.marker.penup()
        self.marker.left(180)
        self.marker.goto(15,SETPOINT)
        self.marker.color('red')
        self.sim = True
        self.timer = 0

    #do our simulation cycles?
    def cycle(self): 
        while(self.sim): 
        
            #get a thrust output from our PID
            thrust = 10 #newtons
            self.Insight.set_ddy(thrust)
            self.Insight.set_dy()
            self.Insight.set_y()
            time.sleep(TIME_STEP)
            self.timer +=1 

            if self.timer > SIM_TIME: 
                self.sim = False
            
            elif self.Insight.get_y() > 800: 
                self.sim = False
            
            elif self.Insight.get_y() < -800: 
                self.sim = False




            
    #1 degree of freedom problem 2 sep PID problem


class Rocket(object):
    def __init__(self):
        global Rocket
        self.Rocket = turtle.Turtle()
        self.Rocket.shape('square')
        self.Rocket.color('black')
        self.Rocket.penup()
        self.Rocket.goto(INITIAL_X, INITIAL_Y)
        self.Rocket.speed(0)

        #physics
        self.ddy = 0 # v acceleration
        self.dy = V_i # ver velocity
        self.y = INITIAL_Y
        #Y_i #


    def set_ddy(self, thrust): # v acceleration
        self.ddy = g + thrust/MASS # thrust impact system ==> output of PID controller

    def get_ddy(self):
        return self.ddy 

    def set_dy(self):
        self.dy += self.ddy #2nd derivate ddy has it's own Intergral this is  Velocity
   
    def get_dy(self):
        return self.dy


    def set_y(self):
        # self.y += self.dy
        self.Rocket.sety(self.y + self.dy)
        # return self.y

    def get_y(self):
        self.y = self.Rocket.ycor()
        return self.y


class PID(object): 
    def __init__(self, KP, KI, KD, target): 
        self.kp = KP
        self.ki = KI 
        self.kd = KD
        


def main():
    # while(TIMER < 5):
    sim = Simulation()
    #     time.sleep(1)
    #     timer +=1
    sim.cycle()

main()