class Motor:
    def __init__(self, motor_number):
        self.motor_number = motor_number
        self.speed = 0
    
    def set_speed(self, speed):
        self.speed = speed
        print(f"Motor {self.motor_number} set to speed {self.speed}")








# Import the necessary ROS libraries
import rospy
from std_msgs.msg import String



# Initialize the ROS node with a name
rospy.init_node('my_node')

# Create a publisher object that will publish messages to the 'my_topic' topic
# The messages will be of type 'String' and the maximum number of queued messages is 10
pub = rospy.Publisher('my_topic', String, queue_size=10)

# Set the publishing rate to 10Hz
rate = rospy.Rate(10) # 10hz

# Loop until the ROS node is shutdown
while not rospy.is_shutdown():
   
   
    user_input_motor = input("Enter motor number: ")
    user_input_parameter = input("Enter parameter: ")


   
   
   
   
   
   
    # Create a message to be published
    message = "Hello, world!"
    
    # Publish the message to the 'my_topic' topic
    pub.publish(message)
    
    # Sleep for the remaining time to maintain the 10Hz publishing rate
    rate.sleep()
