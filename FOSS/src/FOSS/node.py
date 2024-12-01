#!/usr/bin/env python
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node
import keyboard


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__("keyboard_control_node")
        self.get_logger().info("Keyboard Control Node has started!")
        self.running = True

    def move_forward(self):
        self.get_logger().info("Moving forward...")

    def move_backward(self):
        self.get_logger().info("Moving backward...")

    def turn_left(self):
        self.get_logger().info("Turning left...")

    def turn_right(self):
        self.get_logger().info("Turning right...")

    def stop(self):
        self.get_logger().info("Stopping the robot...")

    def shutdown(self):
        self.get_logger().info("Shutting down the node...")
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        node.get_logger().info("Use arrow keys to control the robot. Press 'Esc' to quit.")

        while node.running:
            if keyboard.is_pressed("up"):
                node.move_forward()
            elif keyboard.is_pressed("down"):
                node.move_backward()
            elif keyboard.is_pressed("left"):
                node.turn_left()
            elif keyboard.is_pressed("right"):
                node.turn_right()
            elif keyboard.is_pressed("esc"):
                node.shutdown()
            else:
                node.stop()

    except KeyboardInterrupt:
        node.shutdown()

    finally:
        node.get_logger().info("Exiting...")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
