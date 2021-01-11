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
    # distances = [[0.0, 150.806929, 161.810138, 81.602679, 186.659736, 120.947596, 178.196448, 75.684636, 193.217411, 276.154853, 169.276784], [168.349578, 0.0, 313.699734, 224.378844, 294.24846, 248.454972, 221.351156, 203.192012, 317.751213, 328.514655, 321.166379], [158.728183, 288.244772, 0.0, 97.306535, 241.02485, 164.112402, 271.167614, 173.721876, 33.968022, 339.150077, 24.070262], [81.602679, 202.005838, 97.71268, 0.0, 235.867419, 158.954971, 250.751354, 147.689904, 130.075825, 333.992646, 105.072968], [186.659736, 296.624464, 245.210808, 240.053377, 0.0, 100.106501, 163.894715, 112.322394, 262.823868, 97.892502, 252.926108], [120.947596, 230.912323, 164.112402, 158.954971, 100.106501, 0.0, 146.728487, 64.459907, 181.725462, 198.231729, 171.827703], [178.196448, 248.618033, 270.743969, 250.751354, 166.608851, 146.728487, 0.0, 117.382984, 288.35703, 118.076588, 278.45927], [75.684636, 185.649363, 173.721876, 147.689904, 112.322394, 64.459907, 117.877647, 0.0, 191.334937, 210.386151, 181.437177], [190.926453, 303.596596, 33.933802, 129.504805, 258.63791, 181.725462, 288.780675, 191.334937, 0.0, 356.763138, 28.34202], [276.154853, 361.637964, 343.336035, 338.178604, 97.892502, 198.231729, 118.076588, 210.447621, 360.949096, 0.0, 351.051336], [162.409699, 291.926288, 5.417048, 100.988051, 246.662551, 169.750104, 276.805316, 179.359578, 39.605724, 344.787779, 0.0]]
    hk = HeldKarp(distances, depot=0)
    hk.solve_subproblems()
    print(hk.backtrack_to_solution())
    print(hk.subproblem_costs)
