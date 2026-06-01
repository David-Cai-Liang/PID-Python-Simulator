import numpy as np
import random

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
    global_best_error = 1e10
    global_best_kp, global_best_ki, global_best_kd = 0, 0, 0
    num_restarts = 750
    iterations_per_run = 250

    print(f"Beginning Stochastic Search with {num_restarts} Random Restarts...")

    for run in range(num_restarts):
        # Pick a wildly different starting point for this run
        current_kp = random.uniform(0, 150)
        current_ki = random.uniform(0, 5)
        current_kd = random.uniform(0, 150)

        sim = Simulation(current_kp, current_ki, current_kd)
        local_best_error = sim.cycle()

        for generation in range(iterations_per_run):
            # FIX 2: Adaptive Step Size (Progressively narrow the search radius)
            # Early generations jump far; late generations fine-tune locally
            progress = generation / iterations_per_run
            tweak_scale = 1.0 - progress  # Shrinks from 1.0 down to 0.0

            # Calculate dynamic mutation bounds
            kp_nudge = random.uniform(-15.0, 15.0) * tweak_scale
            ki_nudge = random.uniform(-0.5, 0.5) * tweak_scale
            kd_nudge = random.uniform(-15.0, 15.0) * tweak_scale

            test_kp = max(0.0, min(150.0, current_kp + kp_nudge))
            test_ki = max(0.0, min(5.0, current_ki + ki_nudge))
            test_kd = max(0.0, min(150.0, current_kd + kd_nudge))

            sim = Simulation(test_kp, test_ki, test_kd)
            curr_error = sim.cycle()

            if curr_error < local_best_error:
                local_best_error = curr_error
                current_kp, current_ki, current_kd = test_kp, test_ki, test_kd

        # If this specific restart found a better valley than all previous runs:
        if local_best_error < global_best_error:
            global_best_error = local_best_error
            global_best_kp = current_kp
            global_best_ki = current_ki
            global_best_kd = current_kd
            print(f"-> Found better global valley on Restart #{run+1}! Error: {global_best_error:.2f}")

    print("\n--- GLOBAL OPTIMIZATION COMPLETE ---")
    print(f"Best Kp: {global_best_kp:.2f} | Best Ki: {global_best_ki:.2f} | Best Kd: {global_best_kd:.2f}")
    print(f"Best Error: {global_best_error:.2f}")
main()
