import itertools
from Dynamic_Programming.HeldKarp import HeldKarp


class PhaseI:
    """
    Implementation of Phase I for solving TSP-D using dynamic programming.
    The phase I consists in finding the best route path for the truck based
    on computations of subproblems. The idea of implementation is based on
    Held-Karp https://en.wikipedia.org/wiki/Held%E2%80%93Karp_algorithm which
    states:
    "Every subpath of a path of minimum distance is itself of minimum distance."
    meaning that we can compute the solutions of subproblems starting with the
    smallest.
    The recurrence definition is:
                          + /infinity if k is not in S
    C(S, start, k) =  {   c(start, k) if S = {k}
                          min_{m in S} (C(S - {k}, start, m) + c(m, k)) otherwise
    where S is the binary set of visited nodes in the current subproblem, start is
    the depot node, k is the last visited node in the subproblem and m is the
    intermediary node.
    We adapt the Held Karp implementation for TSP to compute subproblems' costs for
    each start node.
    The paper on which the implementation is based can be found at
    https://www.researchgate.net/publication/320075655_Dynamic_Programming_Approaches_for_the_Traveling_Salesman_Problem_with_Drone
    """

    def __init__(self, path_costs):
        self.path_costs = path_costs
        self.n = len(path_costs)
        self.subproblem_truck_costs = {}

    def solve_subproblems_truck(self):
        for depot in range(self.n):
            hk = HeldKarp(self.path_costs, depot=depot)
            hk.solve_subproblems()
            self.subproblem_truck_costs.update(hk.subproblem_costs)


if __name__ == '__main__':
    distances = [[0, 2, 9, 10], [1, 0, 6, 4], [15, 7, 0, 8], [6, 3, 12, 0]]
    phasei = PhaseI(distances)
    phasei.solve_subproblems_truck()
    print(phasei.subproblem_truck_costs)
