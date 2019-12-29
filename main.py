from drone import Drone
from genetic_algorithm import *
from tqdm import tqdm
import numpy as np

# Initialisation
population_number = 10

ga = GA(population_number)

# Creating a drone
init_state = [0, 0, 0, 0, 0, 0, -1, 0]
ref_state = [0, 0, 0, 0, 0, 0, 0, 0]
drone = Drone(init_state=init_state, ref_state=ref_state)
# un resultat [-0.1992, 0.0156, -0.1297, -0.01, 0.0009, -0.0065, -95.43, -17.12]
number = 10
for hello in range(number):
    all_state_memories = []
    all_control_memories = []
    for index in range(population_number):
        drone.simulate(ga.population[index])
        all_state_memories.append(drone.state_memory)
        all_control_memories.append(drone.control_memory)
        if (index == population_number - 1) and hello == number-1:
            print('\n\n')
            print('gain :', ga.population[index])
            drone.position_show()
            drone.angle_show()
            drone.control_show()
        drone.reset_memory()

    selection = ga.selection(all_state_memories, all_control_memories)
    ga.mating(selection)
    ga.mutation()

        #drone.angle_show()
        #drone.position_show()
        #drone.control_show()

#drone.position_show()

#ga.calculate_fitness(all_state_memories, all_control_memories)
#print(ga.mating(ga.selection(all_state_memories, all_control_memories)))
#ga.mutation()
#print(ga.population)


# problème qui peut sortir c'est scale entre les z et U dans le GA