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
import tf
from math import radians
# ref wiki.ros.org/rviz 
input = np.empty(shape= [0,2])
pos_x = 0
pos_y = 0
orien_z = 0
laser_ranges = []
pi = 3.14
wall_line = []

def ExtractPointsFromBaseScan(data):
    input = np.empty(shape= [0,2])
    global laser_ranges
    laser_ranges = data.ranges
    angle = data.angle_min 
    points_list = []
    ran = range(150,270)
    for index, r in enumerate(data.ranges):
        if r < 3.0 and index in ran:
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

def FindDistance(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    denominator = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return denominator < 2

def createPoint(x,y):
    p = geometry_msgs.msg.Point()
    p.x = x
    p.y = y
    p.z = 0
    return p

def CheckForZero(p):
    return p[0] == 0 or p[1] == 0

def RansacImplementation(input):
    global wall_line
    if(input.shape[0] == 0):
        return
    # Ransac parameters
    ransac_iterations = 5  # number of iterations
    ransac_threshold = 0.05    # threshold
    ransac_ratio = 0.8 

    ratio = 0.0   
    points_list = []  
    model = []
    model_inliers = []
    for i in range(ransac_iterations):
        shuffle = np.random.shuffle(input)
        initial_points = input[0:2]
        test_points = input[2:]
        inliers = np.empty(shape= [0,2])
        num_of_inliers = 0.0
        for j in range(test_points.shape[0]):
            dis = FindDistanceToALine(initial_points[0], initial_points[1], test_points[j])
            if(dis < ransac_threshold):
                inliers = np.append(inliers, [test_points[j]], axis=0)
                num_of_inliers += 1
        if(num_of_inliers/float(test_points.shape[0]) > ratio):
            ratio = num_of_inliers/float(test_points.shape[0])
            model = initial_points
            model_inliers = inliers

    if(len(model) != 0):
        points_list.append(createPoint(model[0][0], model[0][1]))
        points_list.append(createPoint(model[1][0], model[1][1]))
        CreateLinesMarker(points_list, 0)

    #if not (FindDistance(model[0], model[1])):
    wall_line = model


def CreateLinesMarker(lines_list, id):
    rate = rospy.Rate(1)
    lines = Marker()
    lines.header.frame_id = "/base_laser_link"
    lines.ns = "points_and_lines"
    lines.header.stamp = rospy.Time.now()
    lines.id = id
    lines.pose.orientation.w = 1.0
    lines.action = Marker.ADD
    lines.type = Marker.LINE_LIST
    lines.scale.x = 0.3
    lines.scale.y = 0.
    if(id == 1):
        lines.color.g = 1.0
    else:
        lines.color.b = 1.0
    lines.color.a = 1.0
    t = rospy.Duration()
    lines.lifetime = t

    lines.points = lines_list
    # Publish the MarkerArray
    publisher.publish(lines) 
    #rate.sleep()

def CallbackBPGT(msg):
    global orien_z, pos_x, pos_y
    msg_pos = msg.pose.pose.position
    msg_or = msg.pose.pose.orientation
    pos_x = msg_pos.x
    pos_y = msg_pos.y
    orien = tf.transformations.euler_from_quaternion([ msg_or.x, msg_or.y, msg_or.z, msg_or.w])
    orien_z = orien[2]

def CheckObstaclesInTheWay():
    global laser_ranges
    obstacles = 0
    if(len(laser_ranges) != 0):
        for r in range(170, 210):
            if(laser_ranges[r] < 1.5):
                obstacles += 1
    return obstacles > 0

def CheckObstaclesInTheFront():
    print('checking')
    global laser_ranges
    obstacles = 0
    if(len(laser_ranges) != 0):
        for r in range(50, 210):
            if(laser_ranges[r] < 1.5):
                obstacles += 1
    return obstacles > 0

def CheckObstaclesInTheLeft():
    global laser_ranges
    obstacles = 0
    if(len(laser_ranges) != 0):
        for r in range(90, 170):
            if(laser_ranges[r] < 1):
                obstacles += 1
    return obstacles > 5

def ComputeAng_VelForGS(goal_angle):
    if(abs(goal_angle) < pi/10):
        return 0
    return goal_angle

def ComputeAng_VelForWF(line, robot_angle):
    slope = (line[1][1] - line[0][1])/(line[1][0] - line[0][0])
    print(" line slope = %s" % math.atan(slope))
    """ if(abs(math.atan(slope) - robot_angle) < pi/10):
        return 0 """
    return math.atan(slope) - robot_angle

def setVelocity(linear_vel, angular_vel):
    cmd = Twist()
    cmd.linear.x = linear_vel
    cmd.angular.z = angular_vel
    vel_pub.publish(cmd)


def bug2_implementation():
    global orien_z, pos_x, pos_y, wall_line
    atGoal = False
    goal = [4.5, 9.0]
    threshold_distance = 0.5
    linear_vel = 0 
    angular_vel = 0
    robot_state = 'GOALSEEK'
    line = []
    cmd = Twist()
    x = pos_x
    y = pos_y
    while not atGoal:
        print('===============================================')

        goal_distance = math.sqrt((goal[0]-pos_x)**2 + (goal[1]-pos_y)**2)
        robot_angle = orien_z #yaw
        goal_slope = math.atan((goal[1]-y)/(goal[0]-x))
        goal_angle = goal_slope - robot_angle
        points_list = [] 
        points_list.append(createPoint(pos_x, pos_y))
        points_list.append(createPoint(goal[0], goal[1]))
        CreateLinesMarker(points_list, 2)
        print('goal_distance %s' % goal_distance)
        print('robot_angle %s' % robot_angle)
        print('goal_slope %s' % goal_slope) 
        print('goal_angle %s' % goal_angle) 

        if(goal_distance < threshold_distance):
            linear_vel = 0
            angular_vel = 0
            atGoal = True
        else:
            print('State %s' % robot_state)
            if(robot_state == 'GOALSEEK'):
                angular_vel = ComputeAng_VelForGS(goal_angle)
                linear_vel = 2.5
                if(CheckObstaclesInTheFront()):
                    cmd.linear.x = 0
                    
                    robot_state = 'WALLFOLLOW'
                    line = wall_line
                    points_list = [] 
                    points_list.append(createPoint(line[0][0], line[0][1]))
                    points_list.append(createPoint(line[1][0], line[1][1]))
                    CreateLinesMarker(points_list, 1)
                    print('Obstacles are there - line %s' % line)
                    
            if(robot_state == 'WALLFOLLOW'):
                angular_vel = ComputeAng_VelForWF(wall_line, robot_angle)
                print('angular vel %s' % angular_vel)
                cmd.linear.x = 0
                cmd.angular.z = angular_vel
                vel_pub.publish(cmd)
                linear_vel = 0.3
                angular_vel = 0
                points_list.append(createPoint(wall_line[0][0], wall_line[0][1]))
                points_list.append(createPoint(wall_line[1][0], wall_line[1][1]))
                CreateLinesMarker(points_list, 1)
                if not CheckObstaclesInTheWay():
                    print('front clear')
                    linear_vel = 0.8
                """ if(CheckObstaclesInTheLeft()):
                    line = wall_line
                    angular_vel = ComputeAng_VelForWF(line, robot_angle) """
                if not CheckObstaclesInTheFront():
                    robot_state = 'GOALSEEK'
                

            print('Final State %s' % robot_state)
            print('Linear vel %s' % linear_vel)
            print('angular vel %s' % angular_vel)
        setVelocity(linear_vel, angular_vel)
        print('===============================================')
        rospy.sleep(1.0)
                

if __name__ == '__main__':
    rospy.init_node('bug2_controller')
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.Subscriber("base_scan", LaserScan, ExtractPointsFromBaseScan)
    rospy.Subscriber('base_pose_ground_truth', Odometry, CallbackBPGT)
    bug2_implementation()
    print("done") 
    rospy.spin()   
