# Copyright 2016 Open Source Robotics Foundation, Inc.
#
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

from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped
import os
from pynput import keyboard

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('differential_drive')
        self.w=0
        self.v=0
        self.vehicle = os.getenv('HOSTNAME') 
        self.publisher_ = self.create_publisher(Twist2DStamped,
                            f'/{self.vehicle}/lane_controler_node/car_cmd', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        listener = keyboard.Listener(on_press=on_press)
        listener.start()  
        listener.join() 


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def on_press(self,key):
        if key == keyboard.Key.esc:
            return False  # stop listener
        try:
            k = key.char 
        except:
            k = key.name 
        if k in ['up', 'down', 'left', 'right']:
            if k == 'up':
                self.v+=0.1
            elif k == 'down':
                self.v-=0.1
            elif k == 'left':
                self.w-= 0.1
            elif k == 'right':
                self.w+= 0.1
            


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
