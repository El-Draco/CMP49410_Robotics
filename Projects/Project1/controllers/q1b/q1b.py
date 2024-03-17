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

        # Get and enable the necessary devices
        # In this case, only the front sensors are necessary since it is assumed there are no obstacles
        self.ps0 = self.getDevice('ps0')
        self.ps1 = self.getDevice('ps1')
        self.pen = self.getDevice('pen')
        self.ps0.enable(self.timeStep)
        self.ps1.enable(self.timeStep)

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def turn_towards_goal(self):
        # Get the current GPS coordinates
        gps_values = self.gps.getValues()

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
        # if the robot is within 0.01 m of the goal, stop
        if (math.fabs(dist - 0.01) < 0.01):
            left_velocity = 0
            right_velocity = 0
            print(dist)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def run(self):
        self.pen.write(True)
        while self.step(self.timeStep) != -1:
          
            gps_values = self.gps.getValues()
            dist = math.sqrt( pow(gps_values[0] - GOAL[0], 2) + pow(gps_values[1] - GOAL[1],2))
            
            speed = dist * math.sqrt(2) * 0.74
            if (speed > MAX_SPEED):
                speed = MAX_SPEED  
            
            self.left_motor.setVelocity(speed)
            self.right_motor.setVelocity(speed)
            self.turn_towards_goal()

controller = Controller()
controller.run()
