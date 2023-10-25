import rospy
from std_msgs.msg import String
from Inverse_library import inverse_kinematics


rospy.init_node('Robotic_Arm', anonymous=False)  # Initialize the ROS node with a name 'motor_Control'

# rospy.init_node('arm_control', anonymous=False)  # Initialize the node with a name 'arm_control' and make it non-anonymous (so that it can be seen by rostopic list)
# rospy.init_node('motormove')  # Initialize the ROS node with a name 'motormove'


pub_motor_setup = rospy.Publisher('motor_setup', String, queue_size=10)  # Create a publisher for the topic 'motor_setup'
pub_motormove = rospy.Publisher('motormove', String, queue_size=10)  # Create a publisher for the topic 'motormove'

rospy.Subscriber("pre_ik_instructions", String, pre_ik_instruction_callback)  # Subscribe to the topic "pre_ik_instructions" and call the callback function

# Define a global variable to store the received message
Instructions = ""


# Define a callback function to be called when a message is received
def pre_ik_instruction_callback(data):
    """
    store the received message in the global variable Instructions

    Standardise message as : "<x>*<y>*<z>*<t1>*<t2>*<t3>*<gripper>"
    """
    global Instructions
    Instructions = data.data

def motor_setup(motor_id, encA, encB, pwm, in1, in2) -> None:
    """
    Publishes a message to the topic 'motor_setup'
    standardise message as :standardise message as : "<motor_id>*<encA>*<encB>*<pwm>*<in1>*<in2>"


    """
    msg = String()  # Create a String message object
    msg.data = f"{motor_id}*{encA}*{encB}*{pwm}*{in1}*{in2}"  # Set the message data
    pub_motor_setup.publish(msg)  # Publish the message to the topic 'motor_setup'/

def motormove(motor_id , target, position, kp, ki, kd) -> None:
    """
    Publishes a message to the topic 'motormove'
    standardise message as : "<motor_id>*<target position>*<kp>*<ki>*<kd>"

    """


    msg = String()  # Create a String message object
    msg.data = f"{motor_id}*{target}*{position}*{kp}*{ki}*{kd}" # Set the message data
    pub_motormove.publish(msg)  # Publish the message to the topic 'my_topic'

prev_Instructions = ""
while True :
    rospy.spin()
    if Instructions == prev_Instructions:
        continue
    if Instructions != prev_Instructions:
        # Standardise message as : "<x>*<y>*<z>*<t1>*<t2>*<t3>*<gripper>"
        prev_Instructions = Instructions
        Instructions = Instructions.split("*")
        # x = Instructions[0]
        # y = Instructions[1]
        # z = Instructions[2]
        # t1 = Instructions[3]
        # t2 = Instructions[4]
        # t3 = Instructions[5]
        # gripper = Instructions[6]
        # Do something with the Instructions

        out_message = inverse_kinematics(Instructions)  # output of form : standardise message as : "<motor_id>*<target position>*<kp>*<ki>*<kd>"
        #split out_message into motor_id, target, position, kp, ki, kd

        motor_id = out_message[0]
        target = out_message[1]
        kp = out_message[2]
        ki = out_message[3]
        kd = out_message[4]

        motormove(motor_id , target, kp, ki, kd)

    
        
        
        
    


