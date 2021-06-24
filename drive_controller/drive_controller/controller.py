import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import os
from pynput import keyboard

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('differential_drive')
        self.w=0
        self.v=0
        self.vehicle = os.getenv('HOSTNAME') 
        self.publisher_ = self.create_publisher(Twist2DStamped,
                            f'/None/lane_controller_node/car_cmd', 10)
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()  
        listener.join() 


    def on_press(self,key):
        msg = Twist2DStamped()
        #msg = WheelsCmdStamped()
        if key == keyboard.Key.esc:
            return False  # stop listener
        try:
            k = key.char 
        except:
            k = key.name 

        if k in ['up', 'down', 'left', 'right']:
            if k == 'up':
                self.v=0.02
            elif k == 'down':
                self.v-=0.02
            elif k == 'left':
                self.w-= 0.1
            elif k == 'right':
                self.w+= 0.1

            if self.w > 0.2:
                self.w = 0.2
            elif self.w < -0.2:
                self.w = -0.2




        msg.omega = float(self.w)
        msg.v = float(self.v)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pubs: v{msg.v} w{msg.omega} ')


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
