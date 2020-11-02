#!/usr/bin/python3

# import the necessary packages
import rospy

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

class nose_listening():
    """ Class class_name.

    Info about the class
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        """

        #Subscribe to ROS topics
        self.listen_sub = rospy.Subscriber("listening", Bool, self.callback)

        #Define the ROS publishers
        self.nose_pub = rospy.Publisher("topic_pub", UInt8, queue_size=0)

        #Define object as msg type
        self.nose_msg = UInt8()
        self.nose_msg.data = 1      # Data as 1 sets nose color to blue

        print("[INFO] Node started")


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

        #Example to publish msg
        if data.data == True:
            self.nose_pub.publish(self.nose_msg)
        else:
            self.nose_pub.publish(0)


if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('nose_listening_node')       # Init ROS node

        nose = nose_listening()
        rospy.on_shutdown(nose.stopping_node)   #When ROS is closed, this void is executed

        nose.run_loop()

    except rospy.ROSInterruptException:
        pass
