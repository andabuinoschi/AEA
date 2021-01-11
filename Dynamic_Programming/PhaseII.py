import itertools
from Dynamic_Programming.PhaseI import PhaseI


class PhaseII:
    """
    In the second pass, we combine the truck paths with drone nodes to obtain efficient
    operations, that is operations that represent the least costly way to cover a set of nodes S
    with an operation for given start node start and end node k. An optimal TSP-D tour can be
    constructed using efficient operations only as a TSP-D tour will not become longer when
    an inefficient operation is replaced by an efficient operation.
    To find efficient operations associated with every triplet (S, start, k), where the operation
    starts at location start, ends at location k and visits all locations in S, we expand the table of
    truck paths to create a table of operations, by adding the drone movement on top of the
    truck paths. The problem of finding an efficient operation that starts in start, ends in k and
    visits all locations in S can be broken down into the problem of selecting a single drone
    node d from S \ {start, k} and combining the flight of the drone via this location with the
    shortest truck path that starts in start, ends in k and visits all locations in S \ {d}.

    Implementation of Phase II for solving TSP-D using dynamic programming.
    The phase II consists in computing the state value by adding drone
    operations. This can not necessarily improve the final cost of the route
    but most of the time, it does. This phase II consists in computing the
    new state value in the following manner:
                        + /infinity if k is not in S
    D(S, start, k) = {  D_t(S, start, k) if S = {k}
                       min_{d in S / {start, k}} (max {c_d(start, d) + c_d(d, k), D_t(S / {d}, m, k)}) otherwise
    where d is a dronable node in the entire set of cities, start is the depot
    node, c_d(x, y) represents the cost function of the drone between cities x and y
    and D_t represents the subproblems value from Phase I (the costs of the truck operations).

    We have a minimization of
    maximization problem because either the drone waits for the truck at the meeting point k
    (if the drone is faster), either the truck waits for the drone, so the value of the state
    will be the maximum of these costs.
    The paper on which the implementation is based can be found at
    https://www.researchgate.net/publication/320075655_Dynamic_Programming_Approaches_for_the_Traveling_Salesman_Problem_with_Drone
    """
    def __init__(self, path_costs, drone_costs, droneable_nodes):
        self.path_costs = path_costs
        self.n = len(self.path_costs)
        self.droneable_nodes = droneable_nodes
        self.drone_costs_paths = drone_costs
        phaseI = PhaseI(path_costs)
        phaseI.solve_subproblems_truck()
        self.truck_and_drone_subproblems_costs = {}
        self.truck_subproblems_cost = phaseI.subproblem_truck_costs
        for key, value in self.truck_subproblems_cost.items():
            self.truck_subproblems_cost[key] = value + (value[1],)

    def solve_subproblems_truck_and_drone_for_given_depot(self, depot_point=0):
        # Set transition cost from initial state
        truck_and_drone_subproblems_costs = {}
        for k in range(0, self.n):
            if k != depot_point:
                truck_and_drone_subproblems_costs[(1 << k, depot_point, k)] = self.truck_subproblems_cost[
                    (1 << k, depot_point, k)]

        # Iterate subsets of increasing length and store intermediate results
        # in classic dynamic programming manner
        for subset_size in range(2, self.n):
            for subset in itertools.combinations([node for node in range(0, self.n) if node != depot_point],
                                                 subset_size):
                # Set bits for all nodes in this subset
                bits = 0
                for bit in subset:
                    bits |= 1 << bit

                # Find the lowest cost to get to this subset
                for k in subset:
                    prev = bits & ~(1 << k)
                    res = []
                    for d in subset:
                        if d == depot_point or d == k:
                            continue
                        if d not in self.droneable_nodes:
                            res.append((self.truck_subproblems_cost[(prev, depot_point, d)][0] + self.path_costs[d][k], d) + (d,))
                        else:
                            state_without_drone_node = bits & ~(1 << d)
                            all_previous_nodes = list(truck_and_drone_subproblems_costs[(prev, depot_point, d)][1:])
                            # truck_previous_node = all_previous_nodes[0]
                            # if all_previous_nodes[1] == truck_previous_node:
                            all_previous_nodes[1] = d
                            new_value = list(max((self.drone_costs_paths[depot_point][d] + self.drone_costs_paths[d][k], k) + (k,),
                                           self.truck_subproblems_cost[(state_without_drone_node, depot_point, k)]))
                            new_value[1:] = all_previous_nodes
                            new_value = tuple(new_value)
                            res.append(new_value)
                    if res:
                        minimum_value_for_the_state = min(res)
                        truck_and_drone_subproblems_costs[(bits, depot_point, k)] = minimum_value_for_the_state
        return truck_and_drone_subproblems_costs

    def solve_subproblems_truck_and_drone(self):
        self.truck_and_drone_subproblems_costs = self.solve_subproblems_truck_and_drone_for_given_depot(depot_point=0)
        for depot_point in range(1, self.n):
            self.truck_and_drone_subproblems_costs.update(self.solve_subproblems_truck_and_drone_for_given_depot(depot_point=depot_point))


if __name__ == '__main__':
    # distances = [[0, 2, 9, 10], [1, 0, 6, 4], [15, 7, 0, 8], [6, 3, 12, 0]]
    distances = [[0, 2, 9, 10, 5], [1, 0, 6, 4, 2], [15, 7, 0, 8, 6], [6, 3, 12, 0, 7], [4, 9, 8, 3, 0]]
    # drone_costs = [[0, 1, 2, 3], [1, 0, 2, 3], [7, 3, 0, 3], [2, 1, 5, 0]]
    drone_costs = [[0, 1, 2, 3, 2], [1, 0, 2, 3, 4], [7, 3, 0, 3, 1], [2, 1, 5, 0, 2], [4, 2, 1, 3, 0]]
    droneable_nodes = [3, 1]
    phaseII = PhaseII(distances, drone_costs=drone_costs, droneable_nodes=droneable_nodes)
    phaseII.solve_subproblems_truck_and_drone()
    print(phaseII.solve_subproblems_truck_and_drone_for_given_depot(depot_point=1))
    print(phaseII.truck_and_drone_subproblems_costs)
    # still must work on backtrack
    # print(phaseII.backtrack_truck_and_drone_path(depot_point=0))
    # print(phaseII.backtrack_truck_and_drone_path(depot_point=1))
    # print(phaseII.backtrack_truck_and_drone_path(depot_point=2))
    # print(phaseII.backtrack_truck_and_drone_path(depot_point=3))
