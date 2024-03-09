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

MAX_SPEED = 6.28

class Controller(Robot):
    SPEED = 6
    timeStep = 64
    def __init__(self):
        
        super(Controller, self).__init__()

        self.ps0 = self.getDevice('ps0')
        self.ps1 = self.getDevice('ps1')
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
        
        
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timeStep)

    def run(self):
        print("Press 'G' to read the GPS device's position")
        print("Press 'V' to read the GPS device's speed vector")
        while self.step(self.timeStep) != -1:
            key = chr(self.keyboard.getKey() & 0xff)
            if key == 'G':
                gps_values = self.gps.getValues()
                print(f'GPS position: {gps_values[0]} {gps_values[1]} {gps_values[2]}')
            
            elif key == 'V':
                speed_vector_values = self.gps.getSpeedVector()
                print(f'GPS speed vector: {speed_vector_values[0]} {speed_vector_values[1]} {speed_vector_values[2]}')

            ps0_value = self.ps0.getValue()
            ps1_value = self.ps1.getValue()
            if ps1_value > 500:
                # If both distance sensors are detecting something, this means that
                # we are facing a wall. In this case we need to move backwards.
                if ps0_value > 200:
                    left_speed = -self.SPEED / 2
                    right_speed = -self.SPEED
                else:
                    # we turn proportionnaly to the sensors value because the
                    # closer we are from the wall, the more we need to turn.
                    left_speed = -ps1_value / 100
                    right_speed = (ps0_value / 100) + 0.5
            elif ps0_value > 500:
                left_speed = (ps1_value / 100) + 0.5
                right_speed = -ps0_value / 100
            else:  # if nothing was detected we can move forward at maximal speed.
                left_speed = self.SPEED
                right_speed = self.SPEED

            if left_speed > MAX_SPEED:
                left_speed = MAX_SPEED
            if right_speed > MAX_SPEED:
                right_speed = MAX_SPEED
            if left_speed < -MAX_SPEED:
                left_speed = -MAX_SPEED
            if right_speed < -MAX_SPEED:
                right_speed = -MAX_SPEED
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
