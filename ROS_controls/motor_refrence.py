import rospy
from std_msgs.msg import String

# Define a global variable to store the received message
my_string = ""

# Define a callback function to be called when a message is received
def pre_ik_instruction_callback(data):
    global my_string
    my_string = data.data

# Define a function to initialize the node, subscribe to the topic, and spin the node
def listener():
    rospy.init_node('arm_control', anonymous=False)  # Initialize the node with a name 'arm_control' and make it non-anonymous 
    rospy.Subscriber("pre_ik_instructions", String, callback)  # Subscribe to the topic "pre_ik_instructions" and call the callback function
    rospy.spin()  # Spin the node to keep it running until it is stopped


if __name__ == '__main__':
    listener()  # Call the listener function when the script is run
