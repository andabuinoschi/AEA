import itertools


class HeldKarp:
    """
    This is the adjusted implementation for Held Karp dynamic programming approach for TSP
    with the difference that we give the depot node to the constructor of this solver.
    In self.truck_costs we will have as keys tuples of (S, depot, k) where S is coded as binary set
    meaning that if we have visited, for example, cities 1, 2, and 3 then S will be the decimal of
    [1,1,1] shifted to left (in this case [1,1,1,0]), S = 14 and k represents the last visited node
    from S. The values of the dictionary will be tuples of (cost, m) where cost represents
    the computed solution of the current subproblem and m is the intermediary node or the previous node of k.
    The appearance of m in the state definition also helps at backtracking the best solution in the
    end.
    The paper on which the implementation is based can be found at
    https://www.researchgate.net/publication/320075655_Dynamic_Programming_Approaches_for_the_Traveling_Salesman_Problem_with_Drone
    """
    def __init__(self, path_costs, depot=0):
        self.subproblem_costs = {}
        self.path_costs = path_costs
        self.n = len(self.path_costs)
        self.depot = depot

    def solve_subproblems(self):
        """
        Implementation of Held-Karp, an algorithm that solves the Traveling
        Salesman Problem using dynamic programming with memoization for a given depot node.
        """
        # Maps each subset of the nodes to the cost to reach that subset, as well
        # as what node it passed before reaching this subset.
        # Node subsets are represented as set bits.
        self.subproblem_costs = {}
        # Set transition cost from initial state
        for k in range(0, self.n):
            if k != self.depot:
                self.subproblem_costs[(1 << k, self.depot, k)] = (self.path_costs[self.depot][k], self.depot)

        # Iterate subsets of increasing length and store intermediate results
        # in classic dynamic programming manner
        for subset_size in range(2, self.n):
            for subset in itertools.combinations([node for node in range(0, self.n) if node != self.depot], subset_size):
                # Set bits for all nodes in this subset
                bits = 0
                for bit in subset:
                    bits |= 1 << bit

                # Find the lowest cost to get to this subset
                for k in subset:
                    prev = bits & ~(1 << k)

                    res = []
                    for m in subset:
                        if m == self.depot or m == k:
                            continue
                        res.append((self.subproblem_costs[(prev, self.depot, m)][0] + self.path_costs[m][k], m))
                    self.subproblem_costs[(bits, self.depot, k)] = min(res)

    def backtrack_to_solution(self):
        # We're interested in all bits but the least significant (the start state)
        bits = (2**self.n - 1) - 2**self.depot

        # Calculate optimal cost
        res = []
        for k in range(0, self.n):
            if k != self.depot:
                res.append((self.subproblem_costs[(bits, self.depot, k)][0] + self.path_costs[k][self.depot], k))
        opt, parent = min(res)

        # Backtrack to find full path
        path = []
        for i in range(self.n - 1):
            path.append(parent)
            new_bits = bits & ~(1 << parent)
            _, parent = self.subproblem_costs[(bits, self.depot, parent)]
            bits = new_bits

        # Add implicit start state
        path.append(self.depot)

        return opt, list(reversed(path))


if __name__ == '__main__':
    distances = [[0, 2, 9, 10], [1, 0, 6, 4], [15, 7, 0, 8], [6, 3, 12, 0]]
    hk = HeldKarp(distances, depot=2)
    hk.solve_subproblems()
    print(hk.backtrack_to_solution())
    print(hk.subproblem_costs)
