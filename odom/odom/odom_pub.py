import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import  Twist2DStamped, WheelsCmdStamped
import tf2_ros 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point, Pose, Quaternion, Twist, Vector3
from tf2_msgs.msg import TFMessage
from numpy import sin, cos
import time
import quaternion
import numpy as np
import os
import sys


class OdomNode(Node):
    def __init__(self):
        super().__init__("OdometryNode")

        self.x = 0. #3.72341583
        self.y = 0. #2.54024623
        self.th =  0.
        self.last_time = self.get_clock().now()
        self.vl , self.vr = 0.,0.
        self.v, self.omega, self.lav, self.llv = 0., 0., 0., 0.


        self.vehicle = os.getenv('HOSTNAME')
        self.ik_action_sub = self.create_subscription(WheelsCmdStamped,
                                                      '/{}/wheels_driver_node/wheels_cmd'.format(self.vehicle),
                                                      self._set_odom_vals, 10)
        
        self.ik_action_sub = self.create_subscription(Twist2DStamped,
                                                      '/{}/lane_controller_node/car_cmd'.format(self.vehicle),
                                                      self._set_Stamped, 10)
        self.odom_pub = self.create_publisher(Odometry,'/odom', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self._odom_cb)

    def quaternion_from_euler(self, roll, pitch, yaw):

        x = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        y = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        z = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        w = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        return [x,y,z,w]


    def quat_to_msg(self, quat):
        q = Quaternion()
        q.x = quat[0]
        q.y = quat[1]
        q.z = quat[2]
        q.w = quat[3]
        return q


    def _set_odom_vals(self, msg):
        self.vl = msg.vel_left
        self.vr = msg.vel_right
    
    def _set_Stamped(self, msg):
        self.v = msg.v
        self.omega = msg.omega
 
    def _odom_cb(self):
        current_time = self.get_clock().now()
        
        vl = self.vl
        vr = self.vr

        dt = (current_time - self.last_time).nanoseconds / 1000000000
        lv =  ((vr + vl)/2)   # self.v
        vy = 0. 
        av =   (vr- vl) 
        th = float(self.th) * dt
        x = cos(th) * lv * dt
        y = -sin(th) * lv * dt
        th_dot = av * dt
        self.x += (cos(self.th) * x - sin(self.th) * y)/8
        self.y += (sin(self.th) * x + cos(self.th) * y)/8
        self.th += th_dot
        if lv == 0:
            av = 0.
        self.set_msg(lv,av, current_time)



    def set_msg(self, v , vth, current_time):


        quat = self.quat_to_msg(self.quaternion_from_euler(0.,0.,self.th))

        br = tf2_ros.TransformBroadcaster(self)
                       

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "duckiebot/footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.
        t.transform.rotation =  quat
        br.sendTransform(t)

    
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.;

        odom.pose.pose.orientation = quat
    
        odom.child_frame_id = "duckiebot/footprint"
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.
        odom.twist.twist.angular.z = self.th #self.th
 
        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)

    odom = OdomNode()

    rclpy.spin(odom)

    odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

