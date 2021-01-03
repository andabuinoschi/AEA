from PhaseII import PhaseII
import itertools


class PhaseIII:
    """
    We compute the costs of all states using a table D. To compute the costs of a state
    (subproblem) D(S, k) we look for a minimum cost sequence of efficient operations that
    starts at depot point, ends at k and visits all locations in S. This is specified in
    the recurrent definition below, where we compute the cost of a state as the minimum over
    the cost of possible preceding states plus the arc costs of the connecting arc connecting
    the two states (which is the cost of the corresponding efficient operation).
                 + infinity, if k is not in S
    D(S, k) = {  c(depot_point, k) if S = {k}
                 min_{T set in S)(min_{m in S} D(S - (T - {m}), m) + D_{phaseII}(T, m, k))
    where k is the last point visited in the tour contained in S, T is a subset of S,
    m is a middle point, c(x,y) represents the truck distance from x to y and D_(phaseII)
    represents the subproblems' costs from Phase II algorithm.
    """

    def __init__(self, truck_costs, drone_costs, droneable_nodes, depot_point):
        self.truck_costs = truck_costs
        self.n = len(self.truck_costs)
        self.depot_point = depot_point
        self.droneable_nodes = droneable_nodes
        self.drone_costs_paths = drone_costs
        phaseII = PhaseII(truck_costs, drone_costs, droneable_nodes)
        phaseII.solve_subproblems_truck_and_drone()
        self.truck_and_drone_subproblems_costs = phaseII.truck_and_drone_subproblems_costs
        self.truck_subproblems_cost = phaseII.truck_subproblems_cost
        self.optimal_subproblem_cost = {}

    def solve_entire_problem(self):
        # Create initial states of one element as being a truck delivery to node k
        v_bits = 2 ** self.n - 1 - 2 ** self.depot_point
        subset_v = [node for node in range(0, self.n) if node != self.depot_point]
        z = {}
        for k in subset_v:
            self.optimal_subproblem_cost[(1 << k, k)] = (self.truck_costs[self.depot_point][k], k, k)
        for subset_size_u in range(1, self.n):
            for subset_u in itertools.combinations([node for node in range(0, self.n) if node != self.depot_point],
                                                   subset_size_u):
                # Set bits for all nodes in this subset
                u_bits = 0
                for bit in subset_u:
                    u_bits |= 1 << bit
                v_minus_u = [node for node in subset_v if node not in subset_u]
                for subset_size_t in range(1, len(v_minus_u) + 1):
                    for subset_t in itertools.combinations(v_minus_u, subset_size_t):
                        # Set bits for all nodes in subset T
                        t_bits = 0
                        for bit in subset_t:
                            t_bits |= 1 << bit
                        for u, w in itertools.product(subset_u, subset_v):
                            if u != w:
                                if not (u_bits | (1 << u) | (1 << w) | t_bits, w) in self.optimal_subproblem_cost:
                                    self.optimal_subproblem_cost[(u_bits | (1 << u) | (1 << w) | t_bits, w)] = (
                                    float('inf'),)
                                result_z = self.optimal_subproblem_cost[(u_bits, u)][0] + \
                                           self.truck_and_drone_subproblems_costs[(t_bits | (1 << w), u, w)][0]
                                new_state = (u_bits | (1 << u) | (1 << w) | t_bits, w)
                                if result_z < self.optimal_subproblem_cost[new_state][0]:
                                    truck_and_drone_solution = self.truck_and_drone_subproblems_costs[(t_bits | (1 << w), u, w)]
                                    parent_truck = truck_and_drone_solution[1]
                                    parent_drone = truck_and_drone_solution[2]
                                    self.optimal_subproblem_cost[new_state] = (result_z, u, t_bits | (1 << w), parent_truck, parent_drone)

    def backtrack_solution(self):
        final_state = 2 ** self.n - 1 - 2 ** self.depot_point
        res = []

        for k in range(0, self.n):
            if k != self.depot_point:
                res.append(
                    (self.optimal_subproblem_cost[(final_state, k)][0] + self.truck_costs[k][self.depot_point], k))

        optimal_cost, parent = min(res)
        bits = final_state & ~parent
        truck_path = []
        drone_path = []
        truck_path.append(parent)
        drone_path.append(parent)
        print(optimal_cost, parent)
        return res


if __name__ == '__main__':
    distances = [[0, 2, 9, 10], [1, 0, 6, 4], [15, 7, 0, 8], [6, 3, 12, 0]]
    drone_costs = [[0, 1, 2, 3], [1, 0, 2, 3], [7, 3, 0, 3], [2, 1, 5, 0]]
    droneable_nodes = [3, 1]
    phaseIII = PhaseIII(distances, drone_costs, droneable_nodes, 0)
    phaseIII.solve_entire_problem()
    print(phaseIII.truck_and_drone_subproblems_costs)
    print(phaseIII.optimal_subproblem_cost)
    print(phaseIII.backtrack_solution())
