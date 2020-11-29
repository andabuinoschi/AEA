from HGA.GA import GeneticAlgorithm
from HGA.tsp_model import MTSP


def test_evaluate(ga):
    nodes = [0, 3, 25, 23, 7, 4, 13, 8, 2, 9, 22, 15, 20, 1, 12, 16, 11, 6, 5, 17, 24, 19, 21, 14, 10, 18, 0]
    drone_nodes = [0, 2, 0, 3, 1, 4, 0, 4, 0, 0, 2, 0, 3, 4, 0, 1, 0, 0, 1, 3, 2, 0, 0, 0, 2, 1, 0]
    ga.evaluate(nodes, drone_nodes)


if __name__ == "__main__":
    try:
        mtsp_model = MTSP(
            "data/locations/tbl_locations_01.csv",
            "data/locations/tbl_truck_travel_data_01.csv",
            "data/vehicles/tbl_vehicles_01.csv",
            4,
        )

        ga = GeneticAlgorithm(mtsp_model.node, mtsp_model.vehicle, mtsp_model.travel)
        test_evaluate(ga)

    except:
        print("There was a problem.  Sorry things didn't work out.  Bye.")
        raise
