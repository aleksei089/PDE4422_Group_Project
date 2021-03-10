#!/usr/bin/env python

import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2

# WAYPOINTS = [[0.157, 1.44], [-1.53, 1.4], [-1.82, 3.6], [0.643, 5], [5,0.643], [3.6,-1.82], [1.4,-1.53], [1.44, 0.157]]

WAYPOINTS = [[1.85, 0.12], [3, 1.50], [3, 3.5], [-2, 3], [-1.17, 1.17], [-1, 0.0], [0.0,0.0]]


class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():

    def choose(self):
        choice='q'
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESSE A KEY:")
        rospy.loginfo("|'0': Box 1 ")
        rospy.loginfo("|'1': Box Near Box2 ")
        rospy.loginfo("|'2': White Box on right side of Box3")
        rospy.loginfo("|'3': Black Box on left side of Box3 ")
        rospy.loginfo("|'q': Quit ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")
        choice = input()
        return choice

    def __init__(self):

        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

		# declare the coordinates of interest 
        self.xCafe = 3.75 
        self.yCafe = 5.55
        self.xOffice1 = 4
        self.yOffice1 = -2
        self.xOffice2 = -3.29
        self.yOffice2 = -3.29
        self.xOffice3 = -1.75
        self.yOffice3 = 5.40
        self.goalReached = False
        

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.trajectory = list()

        # initiliaze
        choice = self.choose()

        if (choice == 0):

                self.goalReached = self.move_to_point(self.xCafe, self.yCafe)
		
        elif (choice == 1):

                self.goalReached = self.move_to_point(self.xOffice1, self.yOffice1)

        elif (choice == 2):
			
                self.goalReached = self.move_to_point(self.xOffice2, self.yOffice2)
		
        elif (choice == 3):

                self.goalReached = self.move_to_point(self.xOffice3, self.yOffice3)

        if (choice!='q'):

            if (self.goalReached):
                rospy.loginfo("Congratulations!")
            else:
                rospy.loginfo("Hard Luck!")


        while choice != 'q':
            choice = self.choose()
            if (choice == 0):

				self.goalReached = self.move_to_point(self.xCafe, self.yCafe)
		
            elif (choice == 1):

				self.goalReached = self.move_to_point(self.xOffice1, self.yOffice1)

            elif (choice == 2):
		
				self.goalReached = self.move_to_point(self.xOffice2, self.yOffice2)
		
            elif (choice == 3):

				self.goalReached = self.move_to_point(self.xOffice3, self.yOffice3)

            if (choice!='q'):

                if (self.goalReached):
                    rospy.loginfo("Congratulations!")
                else:
                    rospy.loginfo("Hard Luck!")

        # track a sequence of waypoints
        """for point in WAYPOINTS:
            self.move_to_point(point[0], point[1])
            rospy.sleep(0.5)
        self.stop()
        rospy.logwarn("Action done.")

        # plot trajectory
        data = np.array(self.trajectory)
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.show()"""

   
    def callback(msg):
        #we access the ranges data from the from LaserScan and check if there is any obstable withing the distance of 0.5 meters 
        global move 
        rospy.loginfo(msg)
        if msg.ranges[0] > 0.5:
            #if there is no obstacle within 0.5 meters we command the turtlebot to move forward by publishing to linear.x
            move.linear.x = 0.5
            move.angular.z = 0.0

        else:
		    #If there is an obstacle detected within 0.5 meters we command the turtlebot to turn by publishing to angular.z
		    move.linear.x = 0.0
		    move.angular.z = 0.5
        self.vel_pub.publish(move)
        move = Twist()


    def move_to_point(self, x, y):

        # Compute orientation for angular vel and direction vector for linear vel
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)

        # We should adopt different parameters for different kinds of movement
        self.pid_theta.setPID(1, 0, 0)     # P control while steering
        self.pid_theta.setPoint(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        #self.pid_theta.setPID(1, 0, 0)   # PI control while moving
        self.pid_theta.setPID(1, 0.02, 0.2)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > 0.6:
                linear = linear/abs(linear)*0.6

            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x = linear
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()


    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(2)


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            #rospy.loginfo("odom: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))


if __name__ == '__main__':
    try:
        turtlebot_move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
