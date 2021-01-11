from Dynamic_Programming.PhaseIII import PhaseIII


class SolverDP:
    def __init__(self, truck_time_costs, drones_time_costs, droneable_nodes, depot_point):
        phaseIII = PhaseIII(truck_time_costs, drones_time_costs, droneable_nodes, depot_point)
        phaseIII.solve_entire_problem()
        self.optimal_cost, self.truck_path, self.drone_legs = phaseIII.backtrack_solution()
        self.optimal_cost, self.truck_drone_path = phaseIII.backtrack_truck_drone_path()
