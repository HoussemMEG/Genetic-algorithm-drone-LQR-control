from drone import Drone
import numpy as np


# Creating a drone
init_state = [-20, 0, 0, 0, 0, 0, 0, 0]
ref_state = [0, 0, 0, 0, 0, 0, -5, 0]
drone = Drone(init_state=init_state, ref_state=ref_state)
drone.simulate()
#drone.angle_show()
drone.position_show()
drone.control_show()