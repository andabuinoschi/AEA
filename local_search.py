def get_initial_sol(nodes, distances):
    sol_nodes_visited_by_vehicle = [(1, 0)]

    sol_time = [0]
    current_node = 0
    number_of_nodes = len(nodes)

    while len(sol_nodes_visited_by_vehicle) < number_of_nodes:
        tmp_time = 1e13
        tmp_node = -1

        for j, _ in enumerate(nodes):
            if current_node != j and (1, j) not in sol_nodes_visited_by_vehicle:
                t = distances[1][current_node][j].totalTime
                if t < tmp_time:
                    tmp_time = t
                    tmp_node = j

        current_node = tmp_node
        sol_nodes_visited_by_vehicle.append((1, tmp_node))
        sol_time.append(tmp_time)

    sol_nodes_visited_by_vehicle.append((1, number_of_nodes + 1))  # return to depot
    sol_time.append(distances[1][current_node][0].totalTime)
    return sol_nodes_visited_by_vehicle, sol_time


def select_neighbours(current_state, drone_id):
    neighbours = []
    for i, node in enumerate(current_state):
        if (
            i != 0
            and i != (len(current_state) - 1)
            and current_state[i - 1][0] == 1
            and current_state[i + 1][0] == 1
        ):
            tmp_state = current_state[:]
            tmp_state[i] = (drone_id, node[1])
            neighbours.append(tmp_state)

    return neighbours


def evaluate(sol, distances):
    s = 0
    last_truck_node = 0
    intermediary_drone_cost = 0
    last_drone_node = -1

    for i, node in enumerate(sol):
        if i != 0:
            if node[0] == 1:  # truck
                if i == len(sol) - 1:
                    s += 0
                else:
                    tmp_cost = distances[node[0]][last_truck_node][node[1]].totalTime
                    if last_drone_node != -1:
                        intermediary_drone_cost += distances[2][last_drone_node][
                            node[1]
                        ].totalTime

                    s += max(intermediary_drone_cost, tmp_cost)
                    intermediary_drone_cost = 0
                    last_drone_node = -1
                    last_truck_node = node[1]

            else:  # drone
                intermediary_drone_cost += distances[node[0]][last_truck_node][
                    node[1]
                ].totalTime
                last_drone_node = node[1]
    return s


MAX_ITER = 50


def hill_climbing(vehicles, distances, initial_state, initial_sol_eval):
    print("Initial total time", initial_sol_eval)
    local = False
    current_state = initial_state[:]
    best_solution_cost = initial_sol_eval
    best_solution = None

    it = 0
    while not local and it <= MAX_ITER:
        last_iter_cost = best_solution_cost
        it += 1
        neighbours = select_neighbours(current_state, 2)
        for n_sol in neighbours:
            tmp_eval = evaluate(n_sol, distances)
            if tmp_eval < best_solution_cost:
                best_solution_cost = tmp_eval
                best_solution = n_sol

        current_state = best_solution[:]
        print(best_solution_cost)
        if last_iter_cost - best_solution_cost <= 1e-8:
            local = True

    print(best_solution)


def search(nodes, vehicles, distances):
    initial_sol_nodes, initial_sol_time = get_initial_sol(nodes, distances)
    initial_sol_eval = sum(initial_sol_time)
    hill_climbing(vehicles, distances, initial_sol_nodes, initial_sol_eval)
