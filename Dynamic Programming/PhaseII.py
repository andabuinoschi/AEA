import itertools
from PhaseI import PhaseI


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
    def __init__(self, path_costs, depot=0):
        self.path_costs = path_costs
        self.depot = depot
        self.n = len(self.path_costs)
        self.truck_and_drone_subproblems_costs = {}
        self.droneable_nodes = [3]
        self.cities_visited_by_drone = [0 for node in range(self.n)]
        self.drone_costs_paths = [[0, 1, 2, 3], [1, 0, 2, 3], [7, 3, 0, 3], [2, 1, 5, 0]]
        phaseI = PhaseI(path_costs)
        phaseI.solve_subproblems_truck()
        self.truck_subproblems_cost = phaseI.subproblem_truck_costs

    def solve_subproblems_truck_and_drone(self):
        v = 0

        # Set transition cost from initial state
        for k in range(0, self.n):
            if k != self.depot:
                self.truck_and_drone_subproblems_costs[(1 << k, self.depot, k)] = self.truck_subproblems_cost[
                    (1 << k, self.depot, k)]

        # Iterate subsets of increasing length and store intermediate results
        # in classic dynamic programming manner
        for subset_size in range(2, self.n):
            for subset in itertools.combinations([node for node in range(0, self.n) if node != self.depot],
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
                        if d == self.depot or d == k:
                            continue
                        if d not in self.droneable_nodes:
                            res.append((self.truck_subproblems_cost[(prev, self.depot, d)][0] + self.path_costs[d][k], d))
                        else:
                            state_without_drone_node = bits & ~(1 << d)
                            res.append(max((self.drone_costs_paths[self.depot][d] + self.drone_costs_paths[d][k], k),
                                           self.truck_subproblems_cost[(state_without_drone_node, self.depot, k)]))
                    self.truck_and_drone_subproblems_costs[(bits, self.depot, k)] = min(res)

    def backtrack_truck_and_drone_path(self):
        # We're interested in all bits but the least significant (the start state)
        bits = (2**self.n - 1) - 1

        # Calculate optimal cost
        res = []
        for k in range(1, self.n):
            res.append((self.truck_and_drone_subproblems_costs[(bits, self.depot, k)][0] + self.path_costs[k][self.depot], k))
        opt, parent = min(res)

        # Backtrack to find full path
        path = []
        for i in range(self.n - len(self.droneable_nodes) - 1):
            path.append(parent)
            new_bits = bits & ~(1 << parent)
            _, parent = self.truck_and_drone_subproblems_costs[(bits, self.depot, parent)]
            bits = new_bits

        # Add implicit start state
        path.append(self.depot)

        return opt, list(reversed(path))


if __name__ == '__main__':
    distances = [[0, 2, 9, 10], [1, 0, 6, 4], [15, 7, 0, 8], [6, 3, 12, 0]]
    phaseII = PhaseII(distances, depot=0)
    phaseII.solve_subproblems_truck_and_drone()
    print(phaseII.truck_and_drone_subproblems_costs)
    print(phaseII.truck_subproblems_cost)
    print(phaseII.backtrack_truck_and_drone_path())
