
class DPParser:
    def __init__(self, node, vehicle, travel):
        self.nodes = node
        self.vehicles = vehicle
        self.travel_time = travel

        self.truck_time_costs = []
        self.drone_time_costs = []
        self.droneable_nodes = []
        self.depot_point = None

        self.initialize()

    def initialize(self):
        for _, node in self.nodes.items():
            for _, vehicle in self.vehicles.items():
                if vehicle.vehicleType == 2:
                    if node.parcelWtLbs <= vehicle.capacityLbs:
                        node.truck_only = False
                    else:
                        node.truck_only = True

        truck_travel_data = self.travel_time[1]
        for key_node_i in truck_travel_data.keys():
            row = []
            for key_node_j in truck_travel_data[key_node_i].keys():
                row.append(truck_travel_data[key_node_i][key_node_j].totalTime)
            self.truck_time_costs.append(row)

        drones_travel_data = self.travel_time[2]
        for key_node_i in drones_travel_data.keys():
            row = []
            for key_node_j in drones_travel_data[key_node_i].keys():
                row.append(drones_travel_data[key_node_i][key_node_j].totalTime)
            self.drone_time_costs.append(row)

        for node_tag, node_traits in self.nodes.items():
            if node_traits.nodeType == 0:
                self.depot_point = node_tag
            if node_traits.truck_only:
                continue
            else:
                self.droneable_nodes.append(node_tag)
