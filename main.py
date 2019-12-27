from drone import Drone
from genetic_algorithm import *
import numpy as np

# Initialisation
population_number = 5

ga = GA(population_number)

# Creating a drone
init_state = [0, 0, 0, 0, 0, 0, -1, 0]
ref_state = [0, 0, 0, 0, 0, 0, 0, 0]
drones = [Drone(init_state=init_state, ref_state=ref_state) for _ in range(population_number)]


all_state_memories = []
all_control_memories = []
for index, drone in enumerate(drones):
    drone.simulate(ga.population[index])
    all_state_memories.append(drone.state_memory)
    all_control_memories.append(drone.control_memory)
    #drone.angle_show()
    #drone.position_show()
    #drone.control_show()


# problème qui peut sortir c'est scale entre les z et U dans le GA