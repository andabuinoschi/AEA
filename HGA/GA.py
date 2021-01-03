import random
import time

import numpy as np

from HGA.endurance import give_endurance


class GeneticAlgorithm:
    def __init__(self, nodes, vehicles, travel_time, output_file_solution, output_file_score, number_of_iterations=20,
                 logging=True, population_size=100, crossover_probability=0.2, mutation_probability=0.05,
                 chromosome_mutation_probability=0.01, tour_mutation_probability=0.01, drone_mutation_probability=0.05):
        # GA configuration
        self.population_size = population_size
        self.number_of_iterations = number_of_iterations
        self.crossover_probability = crossover_probability
        self.mutation_probability = mutation_probability
        self.chromosome_mutation_probability = chromosome_mutation_probability
        self.tour_mutation_probability = tour_mutation_probability
        self.drone_mutation_probability = drone_mutation_probability

        self.output_file_solution = output_file_solution
        self.output_file_score = output_file_score

        # TSP data
        self.nodes = nodes
        self.vehicles = vehicles
        self.travel_time = travel_time
        self.number_of_vehicles = len(self.vehicles)
        self.number_of_nodes = len(self.nodes)
        self.populations = []
        self.best_solution_score = 1e9
        self.best_solution = []
        self.penalty_cost = 50000
        self.best_per_iteration = []
        self.logging = logging
        self.initialize()

    def initialize(self):
        # mark node i as truck only if there is no drone able to deliver the parcel
        for _, node in self.nodes.items():
            for _, vehicle in self.vehicles.items():
                if vehicle.vehicleType == 2:
                    if node.parcelWtLbs <= vehicle.capacityLbs:
                        node.truck_only = False
                    else:
                        node.truck_only = True

    def print(self, *args):
        if self.logging:
            print(*args)

    def initialize_population(self):
        self.populations = []
        for p in range(self.population_size):
            self.populations.append(self.generate_population())

    def generate_population(self):
        """
        Generate population by randomly assign a node to a vehicle.
        It makes sure that a drone has a rendezvous node.
        It doesn't take the endurance restrictions into account.
        :return:
        """
        population = [0]
        vehicle_rendezvous = [True for _ in range(len(self.vehicles))]

        tour_permutation = [0] + list(
            np.random.permutation(self.number_of_nodes - 1) + 1
        )
        for n in range(len(self.nodes) - 1):
            tn = tour_permutation[n]
            if self.nodes[tn].truck_only:
                vehicle = 0
            else:
                vehicle = random.randint(0, self.number_of_vehicles - 1)
                while vehicle_rendezvous[vehicle] is False:
                    vehicle = random.randint(0, self.number_of_vehicles - 1)

            if vehicle == 0:
                vehicle_rendezvous = [True for _ in range(len(self.vehicles))]
            else:
                vehicle_rendezvous[vehicle] = False
            population.append(vehicle)

        return tour_permutation, population

    def select(self, q, pick):
        for i in range(0, len(q) - 1):
            if q[i] <= pick < q[i + 1]:
                return i

    def roulette_selection(self, population, fitness):
        _max = sum(fitness)
        new_pop = []
        p = []
        q = [0]

        for i in range(self.population_size):
            p.append(fitness[i] / _max)

        for i in range(1, self.population_size + 1):
            q.append(q[i - 1] + p[i - 1])
        q.append(1.1)
        # plt.plot(q)
        # plt.show()

        for i in range(self.population_size):
            pos = random.uniform(0, 1)
            new_pop.append(population[self.select(q, pos)])
        return new_pop

    def evaluate(self, nodes, drone_nodes):
        c = 0
        last_truck_node = 0
        drone_costs = {}
        for n, d in zip(nodes, drone_nodes):
            if d == 0:  # if truck
                truck_time = self.travel_time[1][last_truck_node][n].totalTime
                max_drone_cost = self.get_max_drone_cost(drone_costs, rendezvous_node=n)
                last_truck_node = n
                c += max(truck_time, max_drone_cost)
                drone_costs = {}
            else:  # if drone
                infesability_cost = 0
                if (d + 1) in drone_costs:
                    infesability_cost += self.penalty_cost

                if self.nodes[n].truck_only:
                    infesability_cost += self.penalty_cost

                drone_costs[d + 1] = (self.travel_time[d + 1][last_truck_node][
                                          n
                                      ].totalTime + infesability_cost, n, last_truck_node)

        return c

    def get_max_drone_cost(self, drone_costs, rendezvous_node):
        """
        Get maximum time for drone delivery from node i(launch node) to node k(rendezvous node)
        :return:
        """

        def penalize_endurance(drone, i, j):
            endurance = give_endurance(self.nodes, self.vehicles, self.travel_time, drone, i, j, rendezvous_node, 1)
            if endurance == -1:
                return 0

            return 0

        max_cost = max(
            [0]
            + [
                dc[0] + self.travel_time[d][dc[1]][rendezvous_node].totalTime + penalize_endurance(d, dc[2], dc[1])
                for d, dc in drone_costs.items()
            ]
        )

        return max_cost

    def get_parents_pairs(self, parents):
        _parents = parents[:]
        parents_pairs = []
        parents_pairs_size = len(parents) / 2

        while len(parents_pairs) < parents_pairs_size:
            select_p1 = random.randint(0, len(_parents) - 1)
            select_p2 = random.randint(0, len(_parents) - 1)
            if select_p1 != select_p2:
                parents_pairs.append((_parents[select_p1], _parents[select_p2]))
                if select_p1 > select_p2:
                    _parents.pop(select_p1)
                    _parents.pop(select_p2)
                else:
                    _parents.pop(select_p2)
                    _parents.pop(select_p1)
        return parents_pairs

    def fill_tour(self, offspring, parent):
        offspring_copy = np.copy(offspring)
        index = 1
        for n in parent:
            if n not in offspring:
                while offspring[index] != 0:
                    offspring_copy[index] = offspring[index]
                    index += 1
                offspring_copy[index] = n
                index += 1

        return offspring_copy

    def keep_drone_assignments(self, nodes, vehicles):
        new_assignments = np.zeros((len(nodes)))
        for i, v in enumerate(vehicles):
            new_assignments[np.argwhere(nodes == i)] = v

        return new_assignments

    def cross_over_tour_pmx(self, parents):
        chromosome_length = len(parents[0][0])
        offspring = np.zeros((self.population_size, 2, chromosome_length), dtype=np.int)

        parents_pairs = self.get_parents_pairs(parents)
        parents_probabilities = [random.uniform(0, 1) for _ in range(len(parents_pairs))]

        parent_pairs_probabilities = zip(parents_pairs, parents_probabilities)

        index = 0
        for parent_pair, probability in parent_pairs_probabilities:
            if probability > self.crossover_probability:
                crossover_point1 = random.randint(2, chromosome_length - 2)
                crossover_point2 = random.randint(crossover_point1 + 1, chromosome_length)

                offspring[index, 0, crossover_point1:crossover_point2] = parent_pair[0][0][
                                                                         crossover_point1:crossover_point2]
                offspring[index][0] = self.fill_tour(offspring[index][0], parent_pair[0][0])
                offspring[index][1] = self.keep_drone_assignments(offspring[index][0], parent_pair[0][1])
                index += 1
                offspring[index, 0, crossover_point1:crossover_point2] = parent_pair[1][0][
                                                                         crossover_point1:crossover_point2]
                offspring[index][0] = self.fill_tour(offspring[index][0], parent_pair[1][0])
                offspring[index][1] = self.keep_drone_assignments(offspring[index][0], parent_pair[0][1])
            else:
                offspring[index] = parent_pair[0]
                index += 1
                offspring[index] = parent_pair[1]
            index += 1

        return offspring

    def cross_over_two_point_cut_vehicles(self, parents):
        chromosome_length = len(parents[0][0])
        offspring = np.zeros((self.population_size, 2, chromosome_length), dtype=np.int)

        parents_pairs = self.get_parents_pairs(list(parents))
        parents_probabilities = [random.uniform(0, 1) for _ in range(len(parents_pairs))]

        parent_pairs_probabilities = zip(parents_pairs, parents_probabilities)

        index = 0
        for parent_pair, probability in parent_pairs_probabilities:
            if probability > self.crossover_probability:
                crossover_point1 = random.randint(2, chromosome_length - 2)
                crossover_point2 = random.randint(crossover_point1 + 1, chromosome_length)

                offspring[index][1][crossover_point1:crossover_point2] = parent_pair[1][1][
                                                                         crossover_point1:crossover_point2]
                offspring[index][1][0:crossover_point1] = parent_pair[0][1][0:crossover_point1]
                offspring[index][1][crossover_point2:] = parent_pair[0][1][crossover_point2:]
                offspring[index][0] = parent_pair[0][0]
                index += 1
                offspring[index][1][crossover_point1:crossover_point2] = parent_pair[0][1][
                                                                         crossover_point1:crossover_point2]
                offspring[index][1][0:crossover_point1] = parent_pair[1][1][0:crossover_point1]
                offspring[index][1][crossover_point2:] = parent_pair[1][1][crossover_point2:]
                offspring[index][0] = parent_pair[1][0]
            else:
                offspring[index] = parent_pair[0]
                index += 1
                offspring[index] = parent_pair[1]
            index += 1

        return offspring

    def mutation(self, offsprings):
        for offspring in offsprings:
            if random.uniform(0, 1) < self.mutation_probability:
                if random.uniform(0, 1) < self.tour_mutation_probability:
                    point1 = random.randint(1, self.number_of_nodes - 2)
                    point2 = random.randint(point1, self.number_of_nodes - 1)

                    _tmp = offspring[0][point2]
                    offspring[0][point1] = offspring[0][point2]
                    offspring[0][point2] = _tmp

                    _tmp = offspring[1][point2]
                    offspring[1][point1] = offspring[1][point2]
                    offspring[1][point2] = _tmp

                for i, _ in enumerate(offspring[1]):
                    if random.uniform(0, 1) < self.drone_mutation_probability:
                        current_value = offspring[1][i]
                        if current_value == 0:  # choose a random drone
                            offspring[1][i] = random.randint(1, self.number_of_vehicles - 1)
                        else:
                            _tmp = random.randint(0, self.number_of_vehicles - 1)
                            while _tmp == current_value:
                                _tmp = random.randint(0, self.number_of_vehicles - 1)

                            offspring[1][i] = _tmp

        return offsprings

    def select_neighbours(self, offspring):
        neighbours = []
        vehicle_ids = []
        for i, vehicle in enumerate(offspring[1]):
            if (
                    i != 0
                    and i != (len(offspring[1]) - 1)
            ):
                node = offspring[0][i]
                if self.nodes[node].truck_only:
                    tmp_state = np.copy(offspring)
                    tmp_state[1][node] = 0
                    neighbours.append(tmp_state)
                else:
                    if vehicle != 0:
                        for i in range(self.number_of_vehicles):
                            if i not in vehicle_ids:
                                tmp_state = np.copy(offspring)
                                tmp_state[1][node] = i
                                neighbours.append(tmp_state)
                        vehicle_ids.append(vehicle)
                    else:
                        vehicle_ids = []

        return neighbours

    def educate(self, offsprings, logging=False):
        MAX_ITER = 10
        new_offsprings = []
        for offspring_no, offspring in enumerate(offsprings):
            if logging:
                print(f"Hill climbing for {offspring_no}")
            it = 0
            local = False
            current_state = np.copy(offspring)
            best_solution = np.copy(offspring)
            best_solution_cost = self.evaluate(list(current_state[0]) + [0], list(current_state[1]) + [0])
            while not local and it <= MAX_ITER:
                last_iter_cost = best_solution_cost
                it += 1
                neighbours = self.select_neighbours(current_state)
                for n_sol in neighbours:
                    tmp_eval = self.evaluate(list(n_sol[0]) + [0], list(n_sol[1]) + [0])
                    if tmp_eval < best_solution_cost:
                        best_solution_cost = tmp_eval
                        best_solution = n_sol

                current_state = np.copy(best_solution)

                if logging:
                    print("Hill climbing:  ", best_solution_cost)
                if last_iter_cost - best_solution_cost <= 1e-8:
                    local = True

            # print(best_solution)
            new_offsprings.append(best_solution)
            best_solution = None
        return new_offsprings

    def run(self):
        print(
            f"Starting Genetic Algorithm for {len(self.nodes)} nodes and {self.number_of_vehicles - 1} drones"
        )
        start_time = time.time()

        current_iteration = 0

        while current_iteration < self.number_of_iterations:
            self.initialize_population()
            current_population = self.populations[:]
            self.print(f"------------------Iteration {current_iteration}------------------")

            no_improvement = 0
            best_iter_score = 1e9

            while no_improvement < 5:
                # Select parents P1 and P2
                eval = np.array([_ for _ in range(self.population_size)], dtype=np.float)

                min_f = 1e9
                max_f = -1e9

                for i, (tour, vehicle) in enumerate(current_population):
                    f = self.evaluate(list(tour) + [0], list(vehicle) + [0])
                    eval[i] = f
                    if f < min_f:
                        min_f = f
                    if f > max_f:
                        max_f = f

                fitness = 1.1 * max_f - eval

                parents = self.roulette_selection(current_population, fitness)
                min_eval = min(eval)
                self.print(f"GA min eval {min_eval} iteration {current_iteration}")
                if min_eval < self.best_solution_score:
                    self.best_solution_score = min_eval
                    best_sol_idx = np.argmin(eval)
                    self.best_solution = current_population[best_sol_idx]

                if min_eval < best_iter_score:
                    best_iter_score = min_eval
                    no_improvement = 0
                else:
                    no_improvement += 1

                offsprings = self.cross_over_tour_pmx(
                    parents,
                )

                offsprings = self.cross_over_two_point_cut_vehicles(offsprings)

                offsprings = self.mutation(
                    offsprings
                )
                offsprings = self.educate(offsprings, logging=False)
                current_population = np.copy(offsprings)
            current_iteration += 1
            self.best_per_iteration.append(best_iter_score)

        # print(self.best_solution)
        print("Best score: ", self.best_solution_score)
        self.print("Average score: ", sum(self.best_per_iteration) / self.number_of_iterations)
        self.print("Time: ", time.time() - start_time)
        np.savetxt(self.output_file_solution, self.best_solution, fmt="%i", delimiter=",")
        np.savetxt(self.output_file_score, np.array([self.best_solution_score]), delimiter=",")
        return self.best_solution_score
