import random

configurations = {
    "population_size": [50, 70, 100],
    "crossover_probability": [0.1, 0.2, 0.3],
    "mutation_probability": [0.05, 0.1, 0.2],
    "chromosome_mutation_probability": [0.01, 0.05, 0.1],
    "tour_mutation_probability": [0.01, 0.05, 0.1],
    "drone_mutation_probability": [0.05, 0.1, 0.2]
}


def get_all_algorithm_configurations():
    algorithm_configurations = []
    for conf_key, conf_values in configurations.items():
        if len(algorithm_configurations) == 0:
            for v in conf_values:
                _conf = {conf_key: v}
                algorithm_configurations.append(_conf)
        else:
            for c in algorithm_configurations[:]:
                first_conf = True
                for v in conf_values:
                    if first_conf:
                        c[conf_key] = v
                        first_conf = False
                    else:
                        _conf = c.copy()
                        _conf[conf_key] = v
                        algorithm_configurations.append(_conf)

    return algorithm_configurations


def get_algorithm_configurations(limit=10):
    algorithm_configurations = {}
    while limit > 0:
        limit -= 1
        _conf = {}
        for conf_key, conf_values in configurations.items():
            _conf[conf_key] = random.choice(conf_values)
        conf_hash = hash(frozenset(_conf.items()))
        algorithm_configurations[conf_hash] = _conf

    return algorithm_configurations
