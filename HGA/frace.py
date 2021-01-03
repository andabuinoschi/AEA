import random

import pandas as pd
from scikit_posthocs import posthoc_nemenyi_friedman
from scipy.stats import chi2

from HGA.GA import GeneticAlgorithm
from HGA.algorithm_configurations import get_algorithm_configurations
from HGA.tsp_model import MTSP


def friedman_score(n, total_ranks):
    number_of_configurations = len(total_ranks)
    return ((12 * n) / (number_of_configurations * (number_of_configurations + 1))) * sum(
        [(total_rank / n - (number_of_configurations + 1) / 2) ** 2 for total_rank in total_ranks])


def get_total_ranks(scores):
    ranks = {}
    for instance, configurations in scores.items():
        cscores = {}
        for c, scores in configurations.items():
            cscores[c] = sum(scores)
        cscores = {k: v for k, v in sorted(cscores.items(), key=lambda item: item[1])}

        rank = 1
        for conf, _ in cscores.items():
            if conf not in ranks:
                ranks[conf] = rank
            else:
                ranks[conf] += rank
            rank += 1
    return ranks


def frace(algorithm, configurations, problems, min_number_of_instances=5):
    it = 0
    tmp_configurations = configurations.copy()
    scores = {}
    while len(tmp_configurations) > 1 or it < 50:
        instance = random.choice(problems)
        for key, c in tmp_configurations.items():
            conf_hash = hash(frozenset(c.items()))
            print("Running algorithm with conf", c)
            alg = algorithm(instance[0].node, instance[0].vehicle, instance[0].travel, instance[1], instance[2],
                            number_of_iterations=1, logging=False, **c)
            score = alg.run()
            if instance[1] not in scores:
                scores[instance[1]] = {conf_hash: [score]}
            else:
                if conf_hash not in scores[instance[1]]:
                    scores[instance[1]][conf_hash] = [score]
                else:
                    scores[instance[1]][conf_hash].append(score)

        it += 1

        total_ranks = get_total_ranks(scores)

        scores_per_configuration = {}
        for instance, configs in scores.items():
            for c, scores in configs.items():
                if c not in scores_per_configuration:
                    scores_per_configuration[c] = scores
                else:
                    scores_per_configuration[c] = scores_per_configuration[c] + scores

        scores_df = pd.DataFrame.from_dict(scores_per_configuration)
        if it >= min_number_of_instances:
            fm = friedman_score(it, [rank for conf, rank in total_ranks.items()])
            critical_value = chi2.ppf(1 - .05, df=it)
            print("Friedman score is", fm, "for ranks", total_ranks)
            if fm > critical_value:
                print("Friedman test value is", fm, "which is larger than the critical value", critical_value, "for",
                      it, "degrees of freedom")
                best_configuration_key = \
                    list({k: v for k, v in sorted(total_ranks.items(), key=lambda item: item[1])}.keys())[0]

                posthoc_friedman_test = posthoc_nemenyi_friedman(scores_df)
                for key in configurations.keys():
                    if posthoc_friedman_test[best_configuration_key][key] <= 0.05:
                        print("There is a major difference between", best_configuration_key, "and", key, ". Removing",
                              key, "from configuration list")
                        del tmp_configurations[key]


if __name__ == "__main__":
    try:
        problems = []

        # set of instances
        for problem in ["02", "04", "05"]:
            output_file_solution_name = f"../data/problems/{problem}/solutions/01_solution_frace.csv"
            output_file_score_name = f"../data/problems/{problem}/solutions/01_score_frace.csv"
            mtsp_model = MTSP(
                f"../data/problems/{problem}/locations/tbl_locations_01.csv",
                f"../data/problems/{problem}/locations/tbl_truck_travel_data_01.csv",
                "../data/vehicles/tbl_vehicles_01.csv",
                4,
            )

            problems.append((mtsp_model, output_file_solution_name, output_file_score_name))

        # set of algorithm configurations
        alg_configurations = get_algorithm_configurations(limit=10)

        frace(GeneticAlgorithm, alg_configurations, problems, min_number_of_instances=5)
    except:
        print("There was a problem.  Sorry things didn't work out.  Bye.")
        raise
