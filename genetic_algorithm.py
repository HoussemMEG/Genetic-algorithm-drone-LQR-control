import numpy as np
from numpy.random import randint
from random import random, choice, gauss
from drone import dt
import pandas as pd
from random import gauss, randrange
from operator import add

# fitness variables
R = [0.36]
Q = [3400, 50]

class GA:
    def __init__(self, pop_size, genes_nb=8, genes_upper_lim=2, genes_lower_lim=-2, z_enable=True):
        self.pop_size = pop_size
        self.genes_nb = genes_nb
        self.genes_upper_lim = genes_upper_lim
        self.genes_lower_lim = genes_lower_lim
        self.z_enable = z_enable
        self.population = []
        self.init_pop()

    def init_pop(self):
        for _ in range(self.pop_size):
            self.population.append(self.individual())

    def individual(self):
        if self.z_enable:
            individual = [-0.1992, 0.0156, -0.1297, -0.0100, 0.0009, -0.0065]
            temp = [round(random() * -100, 2) for _ in range(2)]
            individual += temp
        else:
            individual = [round(random() * (self.genes_upper_lim - self.genes_lower_lim) +
                                self.genes_lower_lim, 2) for _ in range(self.genes_nb)]
        return individual

    def individual_fitness(self, response, control_signal):
        z_squared = [Q[0]*element[6]**2 for element in response]
        z_dot_squared = [Q[1]*element[7]**2 for element in response]
        thrust_squared = [R[0]*element[3]**2 for element in control_signal]
        z_total = [z_squared[i]+z_dot_squared[i]+thrust_squared[i] for i in range(len(response))]
        z_sum = dt*((z_total[0]+z_total[-1])/2+sum(z_total[1:-1]))
        return z_sum

    def calculate_fitness(self, state_memories, control_memories):
        fitness = [self.individual_fitness(state_memories[i], control_memories[i]) for i in range(len(state_memories))]
        return fitness

    def selection(self, fitness):
        chance = gauss(0.5, 0.11)
        while chance > 1 or chance < 0:
            chance = gauss(0.5, 0.11)
        fitness_copy = fitness[:]
        fitness = [abs(fitness[i]-1.1*max(fitness)) for i in range(len(fitness))]
        sum_fitness = sum(fitness)
        fitness_normalised = [[fitness[i]/sum_fitness, i] for i in range(len(fitness))]
        fitness_sorted, sorted_index = zip(*sorted(fitness_normalised))
        fitness_cumsum = list(np.cumsum(fitness_sorted))
        fitness_cumsum.append(chance)
        selection_index = sorted(fitness_cumsum).index(chance)
        selection = sorted_index[selection_index:]
        while len(selection) < 2:
            selection = self.selection(fitness_copy)
        else:
            if len(selection) % 2 == 1:
                return selection[1:]
            else:
                return selection


ga = GA(10)
print(ga.selection([1406.8072633858162, 2082.4334610145684, 827.3850702731806, 12756.016418374971, 746.2045867628776, 1406.8072633858162, 2082.4334610145684, 827.3850702731806, 12756.016418374971, 746.2045867628776]))
