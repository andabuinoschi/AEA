from Dynamic_Programming.PhaseIII import PhaseIII


class SolverDP:
    def __init__(self, truck_time_costs, drones_time_costs, droneable_nodes, depot_point):
        phaseIII = PhaseIII(truck_time_costs, drones_time_costs, droneable_nodes, depot_point)
        phaseIII.solve_entire_problem()
        print(phaseIII.optimal_subproblem_cost)
        self.optimal_cost, self.truck_path, self.drone_legs = phaseIII.backtrack_solution()
