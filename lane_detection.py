import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, CameraInfo
from mhenn_msgs.msg import LaneLines
import numpy as np
import math
import os
import cv2
import sys

from cv_bridge import CvBridge

def nothing(x):
    pass

def create_edge_win(name):
    cv2.namedWindow(name)

    hmin = smin = vmin = hmax = smax = vmax = 0

    cv2.createTrackbar('hmin', name, 0, 179, nothing)
    cv2.createTrackbar('smin', name, 0, 255, nothing)
    cv2.createTrackbar('vmin', name, 0, 255, nothing)
    cv2.createTrackbar('hmax', name, 0, 179, nothing)
    cv2.createTrackbar('smax', name, 0, 255, nothing)
    cv2.createTrackbar('vmax', name, 0, 255, nothing)

    cv2.setTrackbarPos('hmax', name, 179)
    cv2.setTrackbarPos('smax', name, 255)
    cv2.setTrackbarPos('vmax', name, 255)
    thresh = minl = maxg= 0
    
    cv2.createTrackbar('thresh',   name, 0, 255, nothing)
    cv2.createTrackbar('MinLength',name, 0, 255, nothing)
    cv2.createTrackbar('maxGap',   name, 0, 255, nothing)
    
    cv2.setTrackbarPos('thresh',   name, 30)
    cv2.setTrackbarPos('MinLength',name, 30)
    cv2.setTrackbarPos('maxGap',   name, 60)



def average_slope_intercept(frame, line_segments, only_left):
    if line_segments is None:
        return [],[]
    
    left_line, right_line = [], []
    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/2
    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:

            if x1 == x2 :
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0 or only_left and slope < -0.1 and only_left:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            elif slope > 0.1:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
            
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        left_line = make_points(frame, left_fit_average)

    right_fit_average = np.average(right_fit, axis=0)

    if len(right_fit) > 0:
        right_line = make_points(frame, right_fit_average)

    
    return left_line, right_line

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [x1, y1, x2, y2]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def read_hsv(name):
    hmin = cv2.getTrackbarPos('hmin',name) 
    smin = cv2.getTrackbarPos('smin',name) 
    vmin = cv2.getTrackbarPos('vmin',name) 
    hmax = cv2.getTrackbarPos('hmax',name) 
    smax = cv2.getTrackbarPos('smax',name) 
    vmax = cv2.getTrackbarPos('vmax',name) 
    return [hmin,smin,vmin], [hmax,smax,vmax]



def read_hough():
    thresh  = cv2.getTrackbarPos('thresh','edgeparams') 
    minLength  = cv2.getTrackbarPos('MinLength','edgeparams') 
    maxGap  = cv2.getTrackbarPos('maxGap','edgeparams') 
    return thresh, minLength, maxGap 


def middle_lane(im):
    middle_low = np.array([90,50,90])
    middle_high = np.array([110,255,255])
    #hsv_min, hsv_max = read_hsv('edgeparams')
    #border_low = np.array(hsv_min)
    #border_high = np.array(hsv_max)
    #return cv2.inRange(im, border_low, border_high)
    return cv2.inRange(im, middle_low, middle_high)


def border_line(im):
    border_low = np.array([0,0,156])
    border_high = np.array([179,40,255])
    #hsv_min, hsv_max = read_hsv('edgeparams')
    #border_low = np.array(hsv_min)
    #border_high = np.array(hsv_max)
    return cv2.inRange(im, border_low, border_high)

def break_line(im):
    border_low = np.array([112,50,100])
    border_high = np.array([130,250,245])
    #hsv_min, hsv_max = read_hsv('edgeparams')
    #border_low = np.array(hsv_min)
    #border_high = np.array(hsv_max)
    return cv2.inRange(im, border_low, border_high)



def edge_detect(im, side, pts, config):
    
    h, w = im.shape[0], im.shape[1]
    gray = cv2.GaussianBlur(im, (5, 5), 0)

    edges = cv2.Canny(gray, 150, 250)
    
    mask = np.zeros(im.shape[:2], dtype="uint8")
    
    cv2.fillPoly(mask, [pts], 255)
    
    masked = cv2.bitwise_and(edges, edges, mask=mask)
    #cv2.imshow('edges' + str(side) , masked)


    t = 40
    ll = 75
    lg = 60
    if config:
        t, ll, lg = read_hough()

    lines = cv2.HoughLinesP(
        masked,
        rho=1.0,
        theta=np.pi / 180,
        threshold=t,
        minLineLength=ll,
        maxLineGap=lg
    )
    return lines[:1] if isinstance(lines, np.ndarray) else None


def get_lane_lines(im):
    config = False
    h,w , _ = im.shape

    pts = np.array([[0, h], [0, h * .6], [w, h * .6], [w, h]], np.int32)

    dim = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    middle_im = middle_lane(dim)
    border_im = border_line(dim)
    edges = np.zeros(im.shape, dtype=np.uint8)

    lines = edge_detect(middle_im, 0, pts, config)
    middle_line, _ = average_slope_intercept(edges,lines,True)
    #print_lines(im, lines, 'l1')
    pts = np.array([[w*.25, h], [w*.25, h * .6], [w, h * .6], [w, h]], np.int32)
    lines = edge_detect(border_im, 1, pts, config)
    #print_lines(im, lines, 'l2')
    _, right_line = average_slope_intercept(edges,lines, False)
    return middle_line, right_line 

def print_lines(im,lines,name):
    #print(type(lines))
    if not isinstance(lines, np.ndarray):
        return
    edges = np.zeros(im.shape, dtype=np.uint8)

    for line in lines:
        for x1 ,y1, x2, y2 in line:
            cv2.line(edges, (x1,y1), (x2,y2), (255,255,255), 1)
    cv2.imshow(name, edges)


def get_break_line(im):
    config = False
    h,w,_ = im.shape
    pts = np.array([[w*.35, h], [w*.35, h * .7], [w, h * .7], [w, h]], np.int32)
    dim = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    break_im = break_line(dim)
    edges = np.zeros(im.shape, dtype=np.uint8)
    lines = edge_detect(break_im, 0,pts, config)
    return lines


class LaneDetectNode(Node):

    def __init__(self):
        self.angle = 0.0
        super().__init__('lane_detect_node')
        self.pose = {'x':0,'y':0,'theta':0}
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv('HOSTNAME')
        #create_edge_win('edgeparams')
        # Subscribes to the output of the lane_controller_node and IK node
        self.cam_pub = self.create_subscription(CompressedImage,
                                             f'/{self.vehicle}/camera/image/compressed', self.image_cb, 10)
        self.action_pub = self.create_publisher(LaneLines, '/LaneLines', 10) 


    def image_cb(self, image_msg):
        # init ROS opencv bridge
        bridge = CvBridge()

        # convert ros compressed image message to opencv image
        cv_image = bridge.compressed_imgmsg_to_cv2(image_msg,'rgb8')
        
        ml, rl = get_lane_lines(cv_image)
        bl = get_break_line(cv_image) 
        lane_msg = LaneLines()
        lane_msg.im_shape = cv_image.shape
        lane_msg.middle_line = ml
        lane_msg.right_line = rl
        if isinstance(bl, np.ndarray):
            bl = list(bl[0][0])
            bl = [int(x) for x in bl]
            lane_msg.break_line = bl
        else:
            lane_msg.break_line = []

        self.action_pub.publish(lane_msg)
        ################################################
        #
        # publish twist message here
        #
        ###############################################



def main(args=None):

    rclpy.init(args=args)
    lane_detect_node = LaneDetectNode()
    rclpy.spin(lane_detect_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_detect_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
