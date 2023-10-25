# Path: Inverse_Kinematics/Robot_Arm_vicharika.py

import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

my_chain = ikpy.chain.Chain.from_urdf_file("Robot_Arm_vicharika.urdf")

def inverse_kinematics(target_position):
    motor_instruction = my_chain.inverse_kinematics(target_position)
    return motor_instruction

# target_vector = [0.3, 0.3, 0.3]
# target_frame = np.eye(4)
# target_frame[:3, 3] = target_vector

# print(my_chain.inverse_kinematics(target_frame))



