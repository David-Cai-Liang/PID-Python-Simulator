import numpy as np 
import matplotlib 
import turtle 
import time 

#GLOBAL PARAMS 
TIMER = 0 
SETPOINT = 10 #final goal 
SIM_TIME = 10 

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
        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.marker = turtle.Turtle() 

        #Goal we are getting to 
        self.marker.penup()
        self.marker.left(180)
        self.marker.goto(15,SETPOINT)
        self.marker.color('red')
        self.sim = True
        self.timer = 0

    def cycle(self): 
        while(self.sim): 
            if self.timer > SIM_TIME: 
                self.sim = False

            
    #1 degree of freedom problem 2 sep PID problem


class Rocket(object):
    def __init__(self):
        global Rocket
        self.Rocket = turtle.Turtle()
        self.Rocket.shape('Square')
        self.Rocket.color('black')
        self.Rocket.goto(INITIAL_X, INITIAL_Y)
        self.Rocket.speed(0)

        #physics
        self.ddy = 0
        self.dy = V_i
        self.y = Y_i
        



def main():
    # while(TIMER < 5):
    sim = Simulation()
    #     time.sleep(1)
    #     timer +=1
    sim.cycle()

main()