#!/usr/bin/env python3

# Short program for driving the robot arm using a keyboard.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from curtsies import Input

VELOCITY = 200.0
SHIFT_VELOCITY = 20.0
GRIPPER_RATE = 5
SHIFT_GRIPPER_RATE = 1
MIN_GRIPPER = 0.0
MAX_GRIPPER = 180.0

class KeyboardController(Node):

    def __init__(self):
        super().__init__('arm_keyboard_controller')

        self.vel_publisher = self.create_publisher(Float64MultiArray, '/diffbot_base_controller/commands', 10)
        self.gripper_publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

    def run(self):
        print("""
        x: Force stop
        
        Manipulating arm ({rpm:.2f} rpm / {shift_rpm:.2f} rpm with Shift):
             w
        a    s    d
        
        ↑z       x↓
        
        Manipulating gripper:
        open:  q
        close: e

        Esc to quit
        """.format(rpm=VELOCITY, shift_rpm=SHIFT_VELOCITY))
        
        ros_vel = Float64MultiArray()
        ros_gripper = Float64MultiArray()
        vel = [0.0, 0.0, 0.0]
        gripper_pos = 0.0

        with Input() as input_generator:
            while True:
                key = input_generator.send(0.1)
                if key == '<ESC>':  # Escape
                    break

                elif key == 'W':  # Forward-slow
                    vel[1] = SHIFT_VELOCITY
                elif key == 'S':  # Reverse-slow
                    vel[1] = -SHIFT_VELOCITY
                elif key == 'w':  # Forward
                    vel[1] = VELOCITY
                elif key == 's':  # Reverse
                    vel[1] = -VELOCITY
                else:
                    vel[1] = 0.0

                if key == 'A':  # Left-slow
                    vel[0] = SHIFT_VELOCITY
                elif key == 'D':  # Right-slow
                    vel[0] = -SHIFT_VELOCITY
                elif key == 'a':  # Left
                    vel[0] = VELOCITY
                elif key == 'd':  # Right
                    vel[0] = -VELOCITY
                else:
                    vel[0] = 0.0

                if key == 'Z':  # Up-slow
                    vel[2] = SHIFT_VELOCITY
                elif key == 'C':  # Down-slow
                    vel[2] = -SHIFT_VELOCITY
                elif key == 'z':  # Up
                    vel[2] = VELOCITY
                elif key == 'c':  # Down
                    vel[2] = -VELOCITY
                else:
                    vel[2] = 0.0

                if key == 'Q': # Open gripper-slow
                    gripper_pos += SHIFT_GRIPPER_RATE
                elif key == 'E':  # Close grippper-slow
                    gripper_pos -= SHIFT_GRIPPER_RATE
                elif key == 'q':  # Open gripper
                    gripper_pos += GRIPPER_RATE
                elif key == 'e':  # Close grippper
                    gripper_pos -= GRIPPER_RATE

                if key == 'x':  # Force stop - comes last to override others
                    vel = [0.0, 0.0, 0.0]
                    gripper_pos = 0.0

                # Maintain gripper position in range
                gripper_pos = max(min(gripper_pos, MAX_GRIPPER), MIN_GRIPPER)

                # Publish the messages
                ros_vel.data = [vel[0], vel[1], vel[2]]
                ros_gripper.data = [gripper_pos]

                self.vel_publisher.publish(ros_vel)
                self.gripper_publisher.publish(ros_gripper)

def main(args=None):
    rclpy.init(args=args)
    k = KeyboardController()
    k.run()
    k.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()