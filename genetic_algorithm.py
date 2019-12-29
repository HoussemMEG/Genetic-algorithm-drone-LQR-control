import numpy as np
from numpy.random import randint
from random import random, gauss, uniform
from drone import dt

from operator import add

# fitness variables
R = [0.36]
#R = [100000]
Q = [3400, 50]

class GA:
    def __init__(self, pop_size, genes_nb=8, genes_upper_lim=2, genes_lower_lim=-2, z_enable=True):
        self.pop_size = pop_size
        self.genes_nb = genes_nb
        self.genes_upper_lim = genes_upper_lim
        self.genes_lower_lim = genes_lower_lim
        self.z_enable = z_enable
        self.population = []
        self.fitness = []
        self.init_pop()
        self.mutation_variance = 0.5

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
        fitness = [[round(self.individual_fitness(state_memories[i], control_memories[i]), 2), i]
                   for i in range(len(state_memories))]
        fitness_sorted, sorted_index = zip(*sorted(fitness, reverse=True))
        self.population = [self.population[i] for i in sorted_index]
        print(fitness_sorted, " = ", round(sum(fitness_sorted)))
        self.fitness = fitness_sorted

    def selection(self, state_memories, control_memories):
        self.calculate_fitness(state_memories, control_memories)
        chance = gauss(0.5, 0.11)
        while chance > 1 or chance < 0:
            chance = gauss(0.5, 0.11)
        fitnesses = [round(abs(self.fitness[i]-1.1*max(self.fitness)), 2) for i in range(len(self.fitness))]
        sum_fitness = sum(fitnesses)
        fitness_normalised = [fitness/sum_fitness for fitness in fitnesses]
        fitness_cumsum = list(np.cumsum(fitness_normalised))
        fitness_cumsum.append(chance)
        selection_index = sorted(fitness_cumsum).index(chance)
        selection = list(range(selection_index, self.pop_size))
        while len(selection) < 2:
            selection = self.selection(state_memories, control_memories)
        else:
            if len(selection) % 2 == 1:
                return selection[1:]
            else:
                return selection

    def mating(self, selection):
        for i in range(0, len(selection), 2):
            if self.z_enable:
                pivot_point = 7
            else:
                pivot_point = randint(1, self.genes_nb)
            child_one = self.population[selection[i]][0:pivot_point] + self.population[selection[i+1]][pivot_point:]
            child_two = self.population[selection[i+1]][0:pivot_point] + self.population[selection[i]][pivot_point:]
            self.population[i] = child_one
            self.population[i+1] = child_two

    def mutation(self):
        if self.z_enable:
            for j, person in enumerate(self.population):
                for i, gene in enumerate(person):
                    if uniform(0, 1) <= 0.3 and (i == 6 or i == 7):
                        self.population[j][i] = round(gauss(gene, self.mutation_variance), 2)
