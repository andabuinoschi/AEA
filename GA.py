import random


class GeneticAlgorithm:
    def __init__(self, nodes, vehicles, travel_time, number_of_iterations=10):
        # GA configuration
        self.population_size = 20
        self.number_of_iterations = number_of_iterations

        # TSP data
        self.nodes = nodes
        self.vehicles = vehicles
        self.travel_time = travel_time
        self.number_of_vehicles = len(self.vehicles)
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

        self.populations = []

        for p in range(self.population_size):
            self.populations.append(self.generate_population())

    def generate_population(self):
        population = [0]
        vehicle_rendezvous = [True for _ in range(len(self.vehicles))]

        for _ in range(len(self.nodes) - 1):
            vehicle = random.randint(0, self.number_of_vehicles - 1)
            while vehicle_rendezvous[vehicle] is False:
                vehicle = random.randint(0, self.number_of_vehicles - 1)

            if vehicle == 0:
                vehicle_rendezvous = [True for _ in range(len(self.vehicles))]
            else:
                vehicle_rendezvous[vehicle] = False
            population.append(vehicle)

        return population

    def select(self):
        pass

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
            else:  # if drone
                drone_costs[(d + 1, n)] = self.travel_time[d + 1][last_truck_node][n].totalTime

        print(c)

    def get_max_drone_cost(self, drone_costs, rendezvous_node):
        """
        Get maximum time for drone delivery from node i(launch node) to node k(rendezvous node)
        :return:
        """
        return max([0] + [dc + self.travel_time[d[0]][d[1]][rendezvous_node].totalTime for d, dc in drone_costs.items()])

    def run(self):
        print(f"Starting Genetic Algorithm for {len(self.nodes)} nodes and {self.number_of_vehicles - 1} drones")

        current_iteration = 0
        feasible_populations = []
        infeasible_populations = []  # truck_time[i, k] + recover_truck > endurance
        # same constraint violation for drone

        while current_iteration < self.number_of_iterations:
            print(f"------------------Iterations {current_iteration}------------------")
            # Select parents P1 and P2
            self.select()
            # Generate offspring individual C from P1 and P2

            # Apply split on C

            # Educate C using local search - optional

            # Call restore method to update the giant-tour chromosome in C
            current_iteration += 1
