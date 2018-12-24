#!/usr/bin/env python
import roslib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import geometry_msgs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import numpy as np
import sys
# ref wiki.ros.org/rviz 

def ExtractPointsFromBaseScan(data):
    input = np.empty(shape= [0,2])
    angle = data.angle_min 
    points_list = []
    for index, r in enumerate(data.ranges):
        if r == 3.0:
            r = 0
        x = r * np.cos(angle)
        y = r * np.sin(angle)    
        input = np.append(input, [[x,y]], axis=0)
        angle = angle + data.angle_increment
    RansacImplementation(input)


def FindDistanceToALine(p1, p2, p):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x = p[0]
    y = p[1]
    denominator = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    numerator = abs((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1)
    distance = (numerator/denominator)
    return distance

def createPoint(x,y):
    p = geometry_msgs.msg.Point()
    p.x = x
    p.y = y
    p.z = 0
    return p


def CheckForZero(p):
    return p[0] == 0 or p[1] == 0

def RansacImplementation(input):

    if(input.shape[0] == 0):
        print('no obstacles')
        return
    # Ransac parameters
    ransac_iterations = 10  # number of iterations
    ransac_threshold = 0.05    # threshold
    ransac_ratio = 0.8 

    ratio = 0.0   
    points_list = []  
    model = []
    model_inliers = []
    for i in range(ransac_iterations):
        shuffle = np.random.shuffle(input)
        #print(input)
        #print('=============================================================')
        initial_points = input[0:2]
        test_points = input[2:]
        if not (CheckForZero(initial_points[0]) or CheckForZero(initial_points[1])):
            inliers = np.empty(shape= [0,2])
            num_of_inliers = 0.0
            for j in range(test_points.shape[0]):
                if not (CheckForZero(test_points[j])):
                    dis = FindDistanceToALine(initial_points[0], initial_points[1], test_points[j])
                    if(dis < ransac_threshold):
                        inliers = np.append(inliers, [test_points[j]], axis=0)
                        num_of_inliers += 1
                        points_list.append(createPoint(initial_points[0][0], initial_points[0][1]))
                        points_list.append(createPoint(initial_points[1][0], initial_points[1][1]))
            if(num_of_inliers/float(test_points.shape[0]) > ratio):
                ratio = num_of_inliers/float(test_points.shape[0])
                model = initial_points
                model_inliers = inliers

    print('Final =============================================================')
    print(ratio)
    print(model)
    CreateLinesMarker(points_list)
    

def CreateLinesMarker(lines_list):
    rate = rospy.Rate(10)
    lines = Marker()
    lines.header.frame_id = "/base_laser_link"
    lines.ns = "points_and_lines"
    lines.header.stamp = rospy.Time.now()
    lines.id = 1
    lines.pose.orientation.w = 1.0
    lines.action = Marker.ADD
    lines.type = Marker.LINE_LIST
    lines.scale.x = 0.1
    lines.scale.y = 0.1
    # lines are green
    lines.color.b = 1.0
    lines.color.a = 1.0
    t = rospy.Duration()
    lines.lifetime = t

    lines.points = lines_list
    # Publish the MarkerArray
    publisher.publish(lines) 
    rate.sleep()


if __name__ == '__main__':
    k=0
    rospy.init_node('perception_controller')
    publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    sub = rospy.Subscriber("base_scan", LaserScan, ExtractPointsFromBaseScan)
    print("done") 
    rospy.spin()   
