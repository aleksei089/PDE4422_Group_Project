#!/usr/bin/env python
import rospy
import math
import sys
import tf
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

# initialize node, publisher and msg type
rospy.init_node("move_robot")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
velocity_msg = Twist()

# we publish the velocity at 4 Hz (4 times per second)
rate = rospy.Rate(10)  

# initialize transformation frames
tf_listener = tf.TransformListener()
odom_frame = 'odom'
base_frame = 'base_footprint'

# variables for proportional control
k_h_gain = 1
k_v_gain = 1

try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = 'base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
        rospy.signal_shutdown("tf Exception")


def compute_distance(x1,y1,x2,y2):
    """
        function to compute distance between points 

        Inputs: 
            x1: x-coordinate of point1  
            y1: y-coordinate of point1 
            x2: x-coordinate of point2 
            y2: y-coordinate of point2
        Outputs:
            dist: distance between 2 points

    """
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

def get_odom_data():
    """ function to get odometry data 
    """
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return Point(*trans), rotation[2]


last_rotation=0
def go_to_goal(goal):
    """ function to move robot towards a goal points

    Inputs:
        goal: list of x,y coordinates of goal position        
    Output:
        angular_velocity: angular velocity of the robot
        linear_velocity: linear velocity of the robot
    """
	
    global goal_index
    global last_rotation
    (position, rotation) = get_odom_data()
	
    # get goal and distance to goal
    goal_x, goal_y = goal
    distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
    
    # if goal has not been reached
    if distance_to_goal > 0.15:
        angular_velocity, linear_velocity, rotation = get_a_l_r(goal)
		
        """(position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
        
        # We would like to restrict the domain to (0, 2pi)
        if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
            if 0 > goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif 0 <= goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal
        if last_rotation > math.pi - 0.1 and rotation <= 0:
            rotation = 2 * math.pi + rotation
        elif last_rotation < -math.pi + 0.1 and rotation > 0:
            rotation = 2 * math.pi + rotation

        # find angular velocity using proportional control
        angular_velocity = k_v_gain * angle_to_goal-rotation

        # find linear velocity using proportional control
        distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
        linear_velocity = min(k_h_gain * distance_to_goal, 0.14)

        # upper limit on max angular velocity
        if angular_velocity > 0:
            angular_velocity = min(angular_velocity, 1.5)
        else:
            angular_velocity = max(angular_velocity, -1.5)"""

        last_rotation = rotation

    # take robot to next goal if current goal has been reached
    else:
        goal_index += 1
        angular_velocity = 0
        linear_velocity = 0
    return angular_velocity, linear_velocity

def get_a_l_r(goal):
    global last_rotation
    goal_x, goal_y = goal
    (position, rotation) = get_odom_data()
    x_start = position.x
    y_start = position.y
    angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
    
    # We would like to restrict the domain to (0, 2pi)
    if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
        if 0 > goal_y > y_start:
            angle_to_goal = -2 * math.pi + angle_to_goal
        elif 0 <= goal_y < y_start:
            angle_to_goal = 2 * math.pi + angle_to_goal
    if last_rotation > math.pi - 0.1 and rotation <= 0:
        rotation = 2 * math.pi + rotation
    elif last_rotation < -math.pi + 0.1 and rotation > 0:
        rotation = 2 * math.pi + rotation

    # find angular velocity using proportional control
    angular_velocity = k_v_gain * angle_to_goal-rotation

    # find linear velocity using proportional control
    distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
    linear_velocity = min(k_h_gain * distance_to_goal, 0.14)+0.3

    # upper limit on max angular velocity
    if angular_velocity > 0:
        angular_velocity = min(angular_velocity, 1.5)+0.15
    else:
        angular_velocity = max(angular_velocity, -1.5)-0.15

    return angular_velocity, linear_velocity, rotation 

# path way points for turtlebot 
waypoints = [(3.75,5.55), (4,-2),(-3.29,-3), (-1.75,5.40)]

# initialize global variables
goal_index = 0
front = 0
left_15 = 0
right_15 = 0
left_90 = 0
right_90 = 0

def sensor_callback(msg):
    """callback function for the scan subscriber
    """
    global goal_index
    global front
    global left_15
    global right_15
    global left_90
    global right_90
    front = msg.ranges[0]
    left_15 = msg.ranges[30]
    right_15 = msg.ranges[330]
    left_90 = msg.ranges[90]
    right_90 = msg.ranges[270]

    #rospy.loginfo("Distance from obstacle (front): {f}".format(f=front))
    #rospy.loginfo("Distance from obstacle (left): {l}".format(l=left_15))
    #rospy.loginfo("Distance from obstacle (right): {r}".format(r=right_15))
    #rospy.loginfo("Distance from obstacle (left 90): {ln}".format(ln=left_90))
    #rospy.loginfo("Distance from obstacle (right 90): {rn}".format(rn=right_90))

# inistialize subsriber
rospy.Subscriber("scan", LaserScan, sensor_callback)

while not rospy.is_shutdown():

    # threshold distance from obstacle
    #thresh = 0.3
    forwardThresh = 0.7
    sideThresh = 0.6

    # if no obstacle -- go to goal
    if front > forwardThresh and left_15 > sideThresh and right_15 > sideThresh:        
        # break if all goals have been reached
        if goal_index > len(waypoints) -1 :
            rospy.loginfo('All goals have been reached')
            break
        rospy.loginfo("Robot moving towards goal")
        # go towards goal
        velocity_msg.angular.z, velocity_msg.linear.x = go_to_goal(waypoints[goal_index])

    # if obstacle detected on front, turn right
    elif front > forwardThresh:
        if left_15 < sideThresh:
            rospy.loginfo('Front Left')
            velocity_msg.angular.z = -1.5
            velocity_msg.linear.x = 0.0
        elif right_15 < sideThresh:
            rospy.loginfo("Front Right")
            velocity_msg.angular.z = 1.5
            velocity_msg.linear.x = 0.0
        elif left_90 < sideThresh:
            rospy.loginfo('Front 90 Left')
            velocity_msg.angular.z = -1.5
            velocity_msg.linear.x = 0.0
        elif right_90 < sideThresh:
            rospy.loginfo("Front 90 Right")
            velocity_msg.angular.z = 1.5
            velocity_msg.linear.x = 0.0
        #else:
            #velocity_msg.angular.z = 0.0
            #velocity_msg.linear.x = 1.5

    if front < forwardThresh:
        velocity_msg.angular.z = 1.5
        velocity_msg.linear.x = 0.0


    # publish velocity
    pub.publish(velocity_msg)
    rate.sleep()
