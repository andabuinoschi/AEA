import matplotlib.pyplot as plt
import numpy as np

from HGA.GA import GeneticAlgorithm
from HGA.tsp_model import MTSP

from Dynamic_Programming.DPParser import DPParser
from Dynamic_Programming.SolverDP import SolverDP


# 05 - 20170608T131306913055
# 04 - 20170608T121651164057

def test_evaluate(ga):
    nodes = [0, 8, 13, 22, 14, 19, 10, 21, 16, 7, 12, 5, 15, 4, 23, 18, 11, 25, 9, 6, 3, 20, 24, 1, 17, 2, 0]
    drone_nodes = [0, 2, 1, 0, 1, 2, 0, 3, 0, 1, 0, 0, 0, 0, 4, 3, 0, 0, 1, 0, 3, 0, 3, 4, 1, 0, 0]
    print(ga.evaluate(nodes, drone_nodes))


def draw_map(map_file, solution_file, mtsp_model):
    solution = np.loadtxt(solution_file, delimiter=',')
    colors = ["r", "g", "b", "black", "gray"]

    fig, ax = plt.subplots()
    for i in reversed(range(5)):

        if i != 0:
            try:
                tmp_xi = np.array(
                    [[mtsp_model.node[solution[0][idx]].lonDeg, mtsp_model.node[solution[0][idx]].latDeg] for idx, v in
                     enumerate(solution[1]) if v == i or v == 0])

                annotations = [solution[0][idx] for idx, v in enumerate(solution[1]) if v == i or v == 0]

                plt.plot(tmp_xi[:, 0], tmp_xi[:, 1], 'r.')
                plt.plot(tmp_xi[:, 0], tmp_xi[:, 1], colors[i])
                for i in range(len(annotations)):
                    ax.annotate(annotations[i], (tmp_xi[i, 0], tmp_xi[i, 1]))
                # plt.show()
            except:
                pass  # no nodes for drone

        elif i == 0:

            tmp_xi = np.array(
                [[mtsp_model.node[solution[0][idx]].lonDeg, mtsp_model.node[solution[0][idx]].latDeg] for idx, v in
                 enumerate(solution[1]) if v == i] + [
                    [mtsp_model.node[solution[0][0]].lonDeg, mtsp_model.node[solution[0][0]].latDeg]])

            annotations = [solution[0][idx] for idx, v in enumerate(solution[1]) if v == i]

            plt.plot(tmp_xi[1:, 0], tmp_xi[1:, 1], 'r.')
            plt.plot(tmp_xi[:1, 0], tmp_xi[:1, 1], 'ko', markersize=8)
            plt.plot(tmp_xi[:, 0], tmp_xi[:, 1], colors[i])
            for i in range(len(annotations)):
                ax.annotate(annotations[i], (tmp_xi[i, 0], tmp_xi[i, 1]))
    plt.show()

    # mplleaflet.show(path=map_file)


if __name__ == "__main__":
    try:
        # 20170606T115303341654
        # problem "02" has 10 nodes
        # problem "01" has 25 nodes
        # problem "03" has 50 nodes
        problem = "01"
        output_file_solution_name = f"data/problems/{problem}/solutions/01_solution.csv"
        output_file_score_name = f"data/problems/{problem}/solutions/01_score.csv"
        mtsp_model = MTSP(
            f"data/problems/{problem}/locations/tbl_locations_01.csv",
            f"data/problems/{problem}/locations/tbl_truck_travel_data_01.csv",
            "data/vehicles/tbl_vehicles_01.csv",
            4,
        )

        # ga = GeneticAlgorithm(mtsp_model.node, mtsp_model.vehicle, mtsp_model.travel, output_file_solution_name,
        #                       output_file_score_name, number_of_iterations=20)
        # ga.run()
        # draw_map(f"data/problems/{problem}/solutions/01_solution.html", output_file_solution_name, mtsp_model)

        mtsp_model_dp = MTSP(
            f"data/problems/{problem}/locations/tbl_locations_01.csv",
            f"data/problems/{problem}/locations/tbl_truck_travel_data_01.csv",
            "data/vehicles/tbl_vehicles_01.csv",
            4,
        )

        dpParser = DPParser(mtsp_model_dp.node, mtsp_model_dp.vehicle, mtsp_model_dp.travel)
        truck_time_costs = dpParser.truck_time_costs
        drones_time_costs = dpParser.drone_time_costs
        droneable_nodes = dpParser.droneable_nodes
        depot_point = dpParser.depot_point

        dpSolver = SolverDP(truck_time_costs, drones_time_costs, droneable_nodes, depot_point)
        print(dpSolver.optimal_cost, dpSolver.truck_path, dpSolver.drone_legs)

    except:
        print("There was a problem.  Sorry things didn't work out.  Bye.")
        raise
