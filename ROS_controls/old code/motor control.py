#!/usr/bin/env python

import rospy
from your_package_name.stepper_motor import StepperMotor
from std_msgs.msg import Int32

def callback(data):
    # Callback function to move the stepper motor based on received data
    motor.move_degrees(data.data)

if __name__ == '__main__':
    rospy.init_node('stepper_motor_node')

    # Initialize the stepper motor with appropriate pins and steps per revolution
    motor = StepperMotor(step_pin=17, direction_pin=18, steps_per_revolution=200)

    # Create a subscriber to receive commands
    rospy.Subscriber('stepper_motor_command', Int32, callback)

    # Keep the node running
    rospy.spin()


'''
To run a stepper motor using ROS (Robot Operating System) from the terminal, you'll need to set up a ROS package and create a ROS node to control the motor. Here are the general steps to achieve this:

1. Install ROS:
   If you haven't already, you'll need to install ROS on your system. You can find installation instructions for your specific ROS distribution on the ROS website (http://wiki.ros.org/ROS/Installation).

2. Create a ROS Package:
   You should create a new ROS package or use an existing one for your project. To create a new package, you can use the `catkin_create_pkg` command. Replace `your_package_name` with the desired package name and specify any dependencies your package might have.
   
   ```bash
   catkin_create_pkg your_package_name rospy std_msgs
   ```

3. Write a ROS Node:
   Create a Python script for your ROS node to control the stepper motor. This script will use the `rospy` library to communicate with the ROS ecosystem. You can use the `StepperMotor` class we discussed earlier in this script. Make sure to import `rospy` and initialize it as well.

   Here's a basic example of a ROS node that uses the `StepperMotor` class:

   ```python
   #!/usr/bin/env python

   import rospy
   from your_package_name.stepper_motor import StepperMotor
   from std_msgs.msg import Int32

   def callback(data):
       # Callback function to move the stepper motor based on received data
       motor.move_degrees(data.data)

   if __name__ == '__main__':
       rospy.init_node('stepper_motor_node')

       # Initialize the stepper motor with appropriate pins and steps per revolution
       motor = StepperMotor(step_pin=17, direction_pin=18, steps_per_revolution=200)

       # Create a subscriber to receive commands
       rospy.Subscriber('stepper_motor_command', Int32, callback)

       # Keep the node running
       rospy.spin()
   ```

4. Build Your ROS Package:
   You need to build your ROS package using `catkin_make`. Navigate to your catkin workspace and run the following command:

   ```bash
   catkin_make
   ```

5. Run ROS Core:
   Before running your node, you should start the ROS core using the following command:

   ```bash
   roscore
   ```

6. Launch Your ROS Node:
   You can launch your ROS node using the `rosrun` command. Replace `your_package_name` with the actual name of your package and `your_node_name` with your node's name.

   ```bash
   rosrun your_package_name your_node_name.py
   ```

7. Control the Motor:
   To control the motor, you can use ROS topics or services to send commands to your node. For example, you can publish to a topic named `stepper_motor_command` with the desired step count using the `rostopic` command:

   ```bash
   rostopic pub stepper_motor_command std_msgs/Int32 "data: 360"
   ```

This is a basic example of how to control a stepper motor with ROS from the terminal. You can further customize your node to suit your specific application and add error handling and safety features as needed.

'''