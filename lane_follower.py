import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

from mhenn_msgs.msg import LaneLines
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped
import numpy as np
import math
import os
import sys


def stabilize_steering_angle(
          curr_steering_angle,
          new_steering_angle,
          num_of_lane_lines,
          max_angle_deviation_two_lines=1,
          max_angle_deviation_one_lane=1):
    if num_of_lane_lines == 2 :
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
            + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle



class LaneFollowerNode(Node):

    def __init__(self):
        self.angle = 0.0
        super().__init__('lane_follower_node')
        self.pose = {'x':0,'y':0,'theta':0}
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv('HOSTNAME')
        self.mode = 'stop'
        #create_edge_win('edgeparams')
        # Subscribes to the output of the lane_controller_node and IK node
        self.action_pub = self.create_publisher(Twist2DStamped,
                                                   f'/{self.vehicle}/lane_controller_node/car_cmd', 10)
        
        self.flow_pub = self.create_subscription(String,
                                             f'/Flow', self.flow_cb, 10)

        self.cam_pub = self.create_subscription(LaneLines,
                                             f'/LaneLines', self.lane_cb, 10)

        self.pose_sub = self.create_subscription(Pose2D,
                                             f'/{self.vehicle}/pose',self.set_pose, 10)                                                

    def set_pose(self, msg):
        self.pose['x'] = msg.x
        self.pose['y'] = msg.y
        self.pose['theta'] = msg.theta

    def flow_cb(self, msg):
        print(msg.data)
        self.mode = msg.data

    def post_twist(self, v, omega):
        twist_message = Twist2DStamped()
        twist_message.header.stamp = self.get_clock().now().to_msg()
        twist_message.v = v
        twist_message.omega = float(omega)

        self.action_pub.publish(twist_message)
 

    def lane_cb(self, lane_msg):

        # just for testing if the image callback works
        self.get_logger().info('I received an image')

        if self.mode == 'stop':
            self.post_twist(0.0,0.0)
            return

        # publish a twist message with desired speed and turning rate
        ##############################################################
        height, width, _ = lane_msg.im_shape
        middle_line = lane_msg.middle_line 
        right_line = lane_msg.right_line 
        break_line = lane_msg.break_line
        x_offset = y_offset = 1
        leng = 0 
        
        b = 1
        if middle_line and right_line:
            leng = 2
            _, _, left_x2, _ = middle_line
            _, _, right_x2, _ = right_line
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            y_offset = int(height / 2)
        elif middle_line or right_line:
            leng = 1
            lane_lines = middle_line if middle_line else right_line
            x1, _, x2, _ = lane_lines
            x_offset = x2 - x1
            y_offset = int(height / 2)
            b = 0.5

        if self.mode == 'straight':
            b = 0.01

        if not middle_line and not right_line:
             omega = self.angle
        else:
            angle_to_mid_radian = math.atan(x_offset / y_offset) * b
            angle_to_mid_radian = stabilize_steering_angle(self.angle, angle_to_mid_radian, leng)
            omega =  angle_to_mid_radian /5
            self.angle = omega


        ##############################################################
        #default values for v and omega
        if omega == 0.0:
            v = -0.01
        else:
            v = 0.015
        
        self.post_twist(v,omega)
       ################################################
        #
        # publish twist message here
        #
        ###############################################



def main(args=None):

    rclpy.init(args=args)
    lane_follower_node = LaneFollowerNode()
    rclpy.spin(lane_follower_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
