import turtle
import time

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
    def __init__(self,KP=10,KI=0,KD=46):

        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.screen.tracer(0)

        self.Insight = Rocket()

        self.marker = turtle.Turtle()
        self.marker.hideturtle()
        self.marker.penup()
        self.marker.goto(-100, SETPOINT)
        self.marker.color('red')
        self.marker.pendown()
        self.marker.goto(100, SETPOINT)

        # Scoreboard Setup for displaying Error on Screen
        self.writer = turtle.Turtle()
        self.writer.hideturtle()
        self.writer.color('blue')
        self.writer.penup()
        self.writer.goto(-600, 350) # Position text in top-left corner

        self.sim = True
        self.timer = 0

        # Proportional Control
        self.pid = PID(KP,KI,KD,SETPOINT)

    # simulation cycles
    # give access to pid implementation
    def cycle(self):
        while(self.sim):

            # generate thrust output using PID
            thrust = max(0,min(MAX_THRUST,self.pid.compute(self.Insight.get_y())))

            self.Insight.set_ddy(thrust)
            self.Insight.set_dy()
            self.Insight.set_y()

            # Update the text error scoreboard dynamically
            self.writer.clear()
            current_error = SETPOINT - self.Insight.get_y()
            total_ise = self.pid.get_abs_integral_error()
            self.writer.write(
                f"Time:            {self.timer:>8.2f}\n"
                f"Instant Error:   {current_error:>8.2f}\n"
                f"Cumulative ISE: {total_ise:>8.2f}",
                align="left",
                font=("Courier", 16, "normal")
            )

            # Refresh graphics safely
            self.screen.update()

            time.sleep(TIME_STEP)
            self.timer += TIME_STEP

            if self.timer > SIM_TIME:
                self.sim = False

            elif self.Insight.get_y() > 800:
                self.sim = False

            elif self.Insight.get_y() < -800:
                self.sim = False

        if (self.timer > SIM_TIME):
            print("Test Success")
            print("Error:",self.pid.get_abs_integral_error())
        else:
            print("Test Failure")

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
        self.dy += self.ddy * TIME_STEP

    def get_dy(self):
        return self.dy


    def set_y(self):
        self.y += self.dy * TIME_STEP
        self.Rocket.sety(self.y)

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
        # print("error:",self.error)
        # print("integral_error:",self.integral_error)
        # print("derivative_error:",self.derivative_error)
        # print("Output:",self.output)
        return self.output

    def get_abs_integral_error(self):
        return self.abs_integral_error

def main():

    sim = Simulation()
    sim.cycle()

main()
