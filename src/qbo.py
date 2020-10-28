#!/usr/bin/python3

# import the necessary packages
import rospy
import time

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import uInt8
from std_msgs.msg import Int16MultiArray

class expressivenes():
    """ Class class_name.

    This class controls the expressivenes of the robot
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        """

        #Subscribe to ROS topics
        self.exp_sub = rospy.Subscriber("expression", uInt8, self.callback)
        self.motor_x_sub = rospy.Subscriber("motor_x", Int16MultiArray, self.cb_motor_x)
        self.motor_y_sub = rospy.Subscriber("motor_y", Int16MultiArray, self.cb_motor_y)

        #Define the ROS publishers
        self.motor_x_pub = rospy.Publisher("motor_x", Int16MultiArray, queue_size=0)
        self.motor_y_pub = rospy.Publisher("motor_y", Int16MultiArray, queue_size=0)
        self.mouth_pub = rospy.Publisher("set_expression", uInt8, queue_size=0)

        #Define object as msg type
        self.motor_x_msg = Int16MultiArray()
        self.motor_x_msg.data = [0,0]

        self.motor_y_msg = Int16MultiArray()
        self.motor_y_msg.data = [0,0]

        self.mouth_msg = uInt8()
        self.mouth_msg.data = 0

        self.motor_x_initial = 0
        self.motor_y_initial = 0

        print("[INFO] Node started")

    def none(self):
        self.motor_x_msg.data[0] = 0
        self.motor_x_msg.data[1] = 100

        self.motor_y_msg.data[0] = 0
        self.motor_y_msg.data[1] = 100

        self.mouth_msg.data = 0

        self.motor_x_pub.publish(self.motor_x_msg)
        self.motor_y_pub.publish(self.motor_y_msg)
        self.mouth_pub.publish(self.mouth_msg)


    def happy(self):
        """Expression happy

        Move the head quickly in yes movement. Put happy face
        In case any other expression has been called, it stops.
        """
        initial = self.motor_y_initial

        self.mouth_msg.data = 1
        self.mouth_pub.publish(self.mouth_msg)

        self.motor_y_msg.data[0] = -100
        self.motor_y_msg.data[1] = 300
        self.motor_y_pub.publish(self.motor_y_msg)
        time.sleep(0.2)

        self.motor_y_msg.data[0] = 100
        self.motor_y_msg.data[1] = 300
        self.motor_y_pub.publish(self.motor_y_msg)
        time.sleep(0.2)

        self.motor_y_msg.data[0] = -100
        self.motor_y_msg.data[1] = 300
        self.motor_y_pub.publish(self.motor_y_msg)
        time.sleep(0.2)

        self.motor_y_msg.data[0] = 100
        self.motor_y_msg.data[1] = 100
        self.motor_y_pub.publish(self.motor_y_msg)
        time.sleep(0.2)

        self.motor_y_msg.data[0] = initial
        self.motor_y_msg.data[1] = 100
        self.motor_y_pub.publish(self.motor_y_msg)
        time.sleep(1)

    def sad(self):
        pass

    def guilty(self):
        pass

    def run_loop(self):
        """ Infinite loop.

        When ROS is closed, it exits.
        """
        while not rospy.is_shutdown():
            #functions to repeat until the node is closed
            rospy.spin()

    def stopping_node(self):
        """ROS closing node

        Is the function called when ROS node is closed."""
        print("\n\nBye bye! :)\n\n")

    def callback(self, data):
        """ROS callback

        This void is executed when a message is received"""

        if data.data == 1:
            self.happy()
        elif data.data == 2:
            self.sad()
        elif data.data == 3:
            self.guilty()
        else:
            self.none()



    def cb_motor_x(self, data):
        """ROS callback

        In this callback the value of the x received is saved into motor_x_initial
        """
        self.motor_x_initial = data.data[0]

    def cb_motor_y(self, data):
        """ROS callback

        In this callback the value of the x received is saved into motor_x_initial
        """
        self.motor_y_initial = data.data[0]


if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('expressivenes_node')       # Init ROS node

        exp_object = expressivenes()
        rospy.on_shutdown(exp_object.stopping_node)   #When ROS is closed, this void is executed

        exp_object.run_loop()

    except rospy.ROSInterruptException:
        pass
