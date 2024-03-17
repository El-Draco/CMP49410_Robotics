# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""

"""

from controller import Robot
import math

MAX_SPEED = 6.28
GOAL = [-0.35471, -0.35471]

class Controller(Robot):
    SPEED = 6
    timeStep = 64
    def __init__(self):
        
        super(Controller, self).__init__()

        self.ps0 = self.getDevice('ps0')
        self.ps7 = self.getDevice('ps7')
        self.ps1 = self.getDevice('ps1')
        self.ps6 = self.getDevice('ps6')
        self.ps5 = self.getDevice('ps5')
        self.ps2 = self.getDevice('ps2')
        
        self.pen = self.getDevice('pen')
        self.ps0.enable(self.timeStep)
        self.ps7.enable(self.timeStep)
        self.ps1.enable(self.timeStep)
        self.ps6.enable(self.timeStep)
        self.ps5.enable(self.timeStep)
        self.ps2.enable(self.timeStep)
        

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timeStep)

    def follow_goal(self):
        return GOAL


    def move_to_result(self, resultant_x, resultant_y):
        # Get the current GPS coordinates
        gps_values = self.gps.getValues()

        # Calculate the angle between the current orientation and the direction to the goal
        angle_to_goal = math.atan2(resultant_y - gps_values[1], resultant_x - gps_values[0])

        # Get the current velocity vector
        current_velocity = self.gps.getSpeedVector()

        # Calculate the angle of the current velocity vector
        current_angle = math.atan2(current_velocity[1], current_velocity[0])

        # Calculate the angle difference between the current orientation and the goal
        angle_difference = angle_to_goal - current_angle

        # Use proportional control to adjust the velocities
        angular_velocity = 2 * angle_difference

        # Calculate the adjusted velocities
        left_velocity = MAX_SPEED - angular_velocity
        right_velocity = MAX_SPEED + angular_velocity
        dist = math.sqrt( pow(gps_values[0] - GOAL[0], 2) + pow(gps_values[1] - GOAL[1],2))
        if left_velocity > MAX_SPEED:
            left_velocity = MAX_SPEED
        if right_velocity > MAX_SPEED:
            right_velocity = MAX_SPEED
        if left_velocity < -MAX_SPEED:
            left_velocity = -MAX_SPEED
        if right_velocity < -MAX_SPEED:
            right_velocity = -MAX_SPEED
        # Set the adjusted velocities
        if (math.fabs(dist - 0.01) < 0.01):
            left_velocity = 0
            right_velocity = 0
            print(dist)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def avoid_barrels(self):
        ps0_value = self.ps0.getValue()       
        ps1_value = self.ps1.getValue()
        ps2_value = self.ps2.getValue()
        ps5_value = self.ps5.getValue()
        ps6_value = self.ps6.getValue()
        ps7_value = self.ps7.getValue()
        
        
        print(f"ps0_value: {ps0_value}")
        print(f"ps1_value: {ps1_value}")
        print(f"ps2_value: {ps2_value}")
        print(f"ps5_value: {ps5_value}")
        print(f"ps6_value: {ps6_value}")
        print(f"ps7_value: {ps7_value}")
        print("---------------------------------------------------------")
        
        # print(f'left_sensor = {ps5_value} and right sensor = {ps2_value}')
        #RIGHT SIDE OF ROBOT
        if (ps0_value > 80 or ps1_value > 100 or ps2_value > 120) :
            right_force = (max(ps0_value, ps1_value, ps2_value) / (4095/6.28))
        else:
            right_force = 3
        #LEFT SIDE OF ROBOT
        if (ps7_value > 80 or ps6_value > 100 or ps5_value > 120) :
            left_force = (max(ps7_value, ps6_value, ps5_value) / (4095/6.28))
        else:
            left_force = 3
        return right_force, left_force

    def run(self):
        print("Press 'G' to read the GPS device's position")
        print("Press 'V' to read the GPS device's speed vector")
        self.pen.write(True)
        while self.step(self.timeStep) != -1:
            key = chr(self.keyboard.getKey() & 0xff)
            if key == 'G':
                gps_values = self.gps.getValues()
                print(f'GPS position: {gps_values[0]} {gps_values[1]} {gps_values[2]}')
            elif key == 'V':
                speed_vector_values = self.gps.getSpeedVector()
                print(f'GPS speed vector: {speed_vector_values[0]} {speed_vector_values[1]} {speed_vector_values[2]}')
            
            left_velocity, right_velocity = self.avoid_barrels()
            
            
            self.left_motor.setVelocity(left_velocity)
            self.right_motor.setVelocity(right_velocity)

            # read all necessary sensors ==> follow goal, avoid obstacle, avoid wall
            # feed sensors into apf model ==> combines all behaviors
            # finally, feed resulting apf vector to move_to_result    
            # X,y = self.compute_result()
            # self.move_to_result(X, y)

controller = Controller()
controller.run()
