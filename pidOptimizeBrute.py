import numpy as np

#GLOBAL PARAMS
TIMER = 0
SETPOINT = 100 #final goal
SIM_TIME = 45    # in sec

TIME_STEP = 0.05

#_______
INITIAL_X = 0
INITIAL_Y = -400
MASS = 1 #kg
MAX_THRUST = 100 #Newtons
g = -9.81 # Gravitational constant

V_i = 0 #initial velocity
Y_i = 0 #initial height


# -----------

class Simulation(object):
    def __init__(self,KP=27,KI=0,KD=37):

        self.Insight = Rocket()
        self.timer = 0
        # Proportional Control
        self.pid = PID(KP,KI,KD,SETPOINT)

    # simulation cycles
    # give access to pid implementation
    def cycle(self):
        while(self.timer < SIM_TIME):

            # generate thrust output using PID
            thrust = max(0,min(MAX_THRUST,self.pid.compute(self.Insight.get_y())))
            self.Insight.set_ddy(thrust)
            self.Insight.set_dy()
            self.Insight.set_y()
            self.timer += TIME_STEP

            if self.Insight.get_y() > 200:
                return 1e10

            if self.Insight.get_y() < -500:
                return 1e10

        return self.pid.get_abs_integral_error()

class Rocket(object):
    def __init__(self):
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
        self.dy += self.ddy * TIME_STEP

    def get_dy(self):
        return self.dy


    def set_y(self):
        self.y += self.dy * TIME_STEP
        return self.y

    def get_y(self):
        return self.y


class PID(object):
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD # quickly adjust for subpoint swing or overshoot subpoint
        self.set_point = target
        self.error = 0
        self.integral_error = 0
        self.abs_integral_error = 0

        self.error_last = 0
        self.derivative_error = 0
        self.output = 0

    def compute(self, pos):
        self.error_last = self.error
        self.error = self.set_point - pos;
        self.derivative_error = (self.error - self.error_last) / TIME_STEP
        self.integral_error += self.error * TIME_STEP
        self.abs_integral_error += (self.error) ** 2  * TIME_STEP
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        return self.output

    def get_abs_integral_error(self):
        return self.abs_integral_error

def main():
    kp = 0
    ki = 0
    kd = 0
    best_error = 1e10
    for i in np.linspace(0,150,151):
        for j in np.linspace(1,5,11):
            for k in np.linspace(0,150,151):
                sim = Simulation(i,j,k)
                curr_error = sim.cycle()
                if curr_error < best_error:
                    best_error = curr_error
                    kp = i
                    ki = j
                    kd = k
    print(kp,ki,kd)
    print("Error:",best_error)

    # sim = Simulation()
    # error = sim.cycle()
    # print("Error:",error)
main()
