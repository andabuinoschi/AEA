from HGA.GA import GeneticAlgorithm
from HGA.tsp_model import MTSP


def test_evaluate(ga):
    nodes = [0, 8, 13, 22, 14, 19, 10, 21, 16, 7, 12, 5, 15, 4, 23, 18, 11, 25, 9, 6, 3, 20, 24, 1, 17, 2, 0]
    drone_nodes = [0, 2, 1, 0, 1, 2, 0, 3, 0, 1, 0, 0, 0, 0, 4, 3, 0, 0, 1, 0, 3, 0, 3, 4, 1, 0, 0]
    print(ga.evaluate(nodes, drone_nodes))


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
        ga.run()

    except:
        print("There was a problem.  Sorry things didn't work out.  Bye.")
        raise
