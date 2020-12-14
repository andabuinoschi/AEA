import matplotlib.pyplot as plt
import mplleaflet
import numpy as np

from HGA.GA import GeneticAlgorithm
from HGA.tsp_model import MTSP


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
        problem = "03"
        output_file_solution_name = f"data/problems/{problem}/solutions/01_solution.csv"
        output_file_score_name = f"data/problems/{problem}/solutions/01_score.csv"
        mtsp_model = MTSP(
            f"data/problems/{problem}/locations/tbl_locations_01.csv",
            f"data/problems/{problem}/locations/tbl_truck_travel_data_01.csv",
            "data/vehicles/tbl_vehicles_01.csv",
            4,
        )

        ga = GeneticAlgorithm(mtsp_model.node, mtsp_model.vehicle, mtsp_model.travel, output_file_solution_name,
                              output_file_score_name, number_of_iterations=20)
        # test_evaluate(ga)
        ga.run()
        draw_map(f"data/problems/{problem}/solutions/01_solution.html", output_file_solution_name, mtsp_model)
    except:
        print("There was a problem.  Sorry things didn't work out.  Bye.")
        raise
