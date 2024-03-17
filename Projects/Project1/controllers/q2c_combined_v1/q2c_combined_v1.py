from controller import Robot
import math

MAX_SPEED = 6.28
GOAL = [0.35471, 0.35471]

class Controller(Robot):
    SPEED = 6
    timeStep = 64
    def __init__(self):
        
        super(Controller, self).__init__()

        # Get all necessary devices (proximity sensors, pen, and gps)
        self.ps0 = self.getDevice('ps0')
        self.ps7 = self.getDevice('ps7')
        self.ps1 = self.getDevice('ps1')
        self.ps6 = self.getDevice('ps6')
        self.ps5 = self.getDevice('ps5')
        self.ps2 = self.getDevice('ps2')
        self.pen = self.getDevice('pen')
        self.gps = self.getDevice('gps')

        # Enable all devices
        self.ps0.enable(self.timeStep)
        self.ps7.enable(self.timeStep)
        self.ps1.enable(self.timeStep)
        self.ps6.enable(self.timeStep)
        self.ps5.enable(self.timeStep)
        self.ps2.enable(self.timeStep)
        self.gps.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def combine_APF(self, ps0_value, ps1_value, ps2_value, ps5_value, ps6_value, ps7_value, gps_values):
        rep_coeff_l = 1
        rep_coeff_r = 1
        att_coeff_l = 1
        att_coeff_r = 1

        # Call the indivudal APF models to retrieve left and right forces
        att_force_left, att_force_right = self.goal_APF(gps_values)
        rep_force_left, rep_force_right = self.obstacles_APF(ps0_value, ps1_value, ps2_value, ps5_value, ps6_value, ps7_value)

        if att_force_left == 0 and att_force_right == 0:
            return att_force_left, att_force_right
          
        if rep_force_left != 0 and att_force_left != 0:
            att_coeff_l = 0.05
        if rep_force_right != 0 and att_force_right != 0:
            att_coeff_r = 0.05

        print(f'rep_L = {rep_force_left} rep_R = {rep_force_right}')
        print(f'att_L = {att_force_left} att_R = {att_force_right}')
        print("---------------------------")
        # Compute Net Force by summing them:
        final_left_force = att_force_left * att_coeff_l + rep_force_left * rep_coeff_l
        final_right_force = att_force_right * att_coeff_r + rep_force_right * rep_coeff_r
        #return rep_force_left, rep_force_right
        return final_left_force, final_right_force
    
    
    def goal_APF(self, gps_values):
        # Compute the distance between robot and goal
        dist = math.sqrt( pow(gps_values[0] - GOAL[0], 2) + pow(gps_values[1] - GOAL[1],2))
        
        # Calculate the angle between the current orientation and the direction to the goal
        angle_to_goal = math.atan2(GOAL[1] - gps_values[1], GOAL[0] - gps_values[0])
        # Get the current velocity vector
        current_velocity = self.gps.getSpeedVector()
        # Calculate the angle of the current velocity vector
        current_angle = math.atan2(current_velocity[1], current_velocity[0])
        # Calculate the angle difference between the current orientation and the goal
        angle_difference = angle_to_goal - current_angle
        # Use proportional control to adjust the velocities
        angular_velocity = 5 * angle_difference
        # Calculate the adjusted velocities
        
        left_force = MAX_SPEED - angular_velocity
        right_force = MAX_SPEED + angular_velocity
       
        if left_force > MAX_SPEED:
            left_force = MAX_SPEED
        if right_force > MAX_SPEED:
            right_force = MAX_SPEED
        if left_force < -MAX_SPEED:
            left_force = -MAX_SPEED
        if right_force < -MAX_SPEED:
            right_force = -MAX_SPEED

        # Set the adjusted velocities
        print(f'dist = {dist}')
        #if (math.fabs(dist - 0.01) < 0.01):
        if dist < 0.14:
            left_force = 0
            right_force = 0
            print(dist)

        return left_force, right_force


    def obstacles_APF(self, ps0_value, ps1_value, ps2_value, ps5_value, ps6_value, ps7_value):

        print(f"ps0_value: {ps0_value}")
        print(f"ps1_value: {ps1_value}")
        print(f"ps2_value: {ps2_value}")
        print(f"ps5_value: {ps5_value}")
        print(f"ps6_value: {ps6_value}")
        print(f"ps7_value: {ps7_value}")
        print("---------------------------------------------------------")
        
        MAX_SV = 4095
        MIN_SV = 34
        MAX_SP = 6.28
        MIN_SP = 3.14
        #RIGHT SIDE OF ROBOT
        if (ps0_value > 80 or ps1_value > 100 or ps2_value > 120) :
            right_sensor_value = (max(ps0_value, ps1_value, ps2_value) - MIN_SV) / (MAX_SV - MIN_SV)
            # Map sensor values to speed values: MIN + (norm_sensor * (MAX - MIN))
            right_force = (MIN_SP + (right_sensor_value * (MAX_SP - MIN_SP)))
            #right_force = right_sensor_value / (4095/6.28)
            #right_force = (max(ps0_value, ps1_value, ps2_value) / (4095/6.28))
        else:
            right_force = 2
        #LEFT SIDE OF ROBOT
        if (ps7_value > 80 or ps6_value > 100 or ps5_value > 120) :
                    # Normalize sensor values:  Value - MIN / MAX - MIN
            left_sensor_value = (max(ps7_value, ps6_value, ps5_value) - MIN_SV) / (MAX_SV - MIN_SV)
            # Map sensor values to speed values: MIN + (norm_sensor * (MAX - MIN))
            left_force = (MIN_SP + (left_sensor_value * (MAX_SP - MIN_SP)))
            #left_force = (max(ps7_value, ps6_value, ps5_value) / (4095/6.28))
            #left_force = left_sensor_value / (4095/6.28)

        else:
            left_force = 2
            
        if left_force < right_force:
            left_force = left_force * -1
        elif left_force > right_force:
            right_force = right_force * -1
            
        return left_force, right_force

    def read_values(self):
        # Get the distance sensor values (front ones only)
        ps0_value = self.ps0.getValue()      
        ps1_value = self.ps1.getValue()
        ps2_value = self.ps2.getValue()
        ps5_value = self.ps5.getValue()
        ps6_value = self.ps6.getValue()
        ps7_value = self.ps7.getValue()
        # Get the current GPS coordinates
        gps_values = self.gps.getValues()
        # Return all values
        return ps0_value, ps1_value, ps2_value, ps5_value, ps6_value, ps7_value, gps_values

    def run(self):
        # Set the pen to write
        self.pen.write(True)

        while self.step(self.timeStep) != -1:
            ps0_value, ps1_value, ps2_value, ps5_value, ps6_value, ps7_value, gps_values = self.read_values()
            left_velocity, right_velocity = self.combine_APF(ps0_value, ps1_value, ps2_value, ps5_value, ps6_value, ps7_value, gps_values)
            # left_velocity = left_velocity**2
            # right_velocity = right_velocity**2         
            if left_velocity > MAX_SPEED:
                left_velocity = MAX_SPEED
            if right_velocity > MAX_SPEED:
                right_velocity = MAX_SPEED
            if left_velocity < -MAX_SPEED:
                left_velocity = -MAX_SPEED
            if right_velocity < -MAX_SPEED:
                right_velocity = -MAX_SPEED

            
            self.left_motor.setVelocity(left_velocity)
            self.right_motor.setVelocity(right_velocity)

controller = Controller()
controller.run()
