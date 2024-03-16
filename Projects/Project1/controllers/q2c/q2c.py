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
        self.pen = self.getDevice('pen')
        self.ps0.enable(self.timeStep)
        self.ps7.enable(self.timeStep)

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
        ps7_value = self.ps7.getValue()

        left_force = ps0_value
        right_force = ps7_value

        return left_force, right_force

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
            self.left_motor.setVelocity(left_velocity / (4095/6.28))
            self.right_motor.setVelocity(right_velocity / (4095/6.28))


            # read all necessary sensors ==> follow goal, avoid obstacle, avoid wall
            # feed sensors into apf model ==> combines all behaviors
            # finally, feed resulting apf vector to move_to_result    
            # X,y = self.compute_result()
            # self.move_to_result(X, y)

controller = Controller()
controller.run()
