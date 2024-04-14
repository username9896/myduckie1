#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 

class DistanceReader:
    def __init__(self):
        
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose,self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self,msg):
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
        move_forward = Twist()
        move_forward.linear.x = 1.0  # Move forward at 1 m/s

    # Create a Twist message to turn
        turn = Twist()
        turn.linear.y = 1.0  # Rotate at 1 rad/s

        move_backwards = Twist()
        move_backwards.linear.x = -1.0  # Move forward at 1 m/s
        x1 = 0
        y1 = 0

    # Create a Twist message to turn
        turn_down = Twist()
        turn_down.linear.y = -1.0  # Rotate at 1 rad/s

        while True:


            rospy.loginfo("Turtle Position: %s %s", msg.x, msg.y)

            velocity_publisher.publish(move_forward)
            time.sleep(1)

        # Stop moving
            velocity_publisher.publish(Twist())

        # Turn for 2 seconds
            velocity_publisher.publish(turn)
            time.sleep(1)

            velocity_publisher.publish(Twist())

            velocity_publisher.publish(move_backwards)
            time.sleep(1)

            velocity_publisher.publish(Twist())

            velocity_publisher.publish(turn_down)
            time.sleep(1)


        # Stop turning
            velocity_publisher.publish(Twist())


        ########## YOUR CODE GOES HERE ##########
        # Calculate the distance the turtle has travelled and publish it
            x1 = x1 + msg.x
            y1 = y1 + msg.y
            distance = x1 + y1
            rospy.loginfo("Total distance: %s", distance)
        ###########################################

if __name__ == '__main__': 

    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass
        
