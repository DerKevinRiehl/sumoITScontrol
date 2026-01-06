# #############################################################################
# ####### FairSCOSCA: Fairness At Arterial Signals - Just Around The Corner
# #######   AUTHOR:       Kevin Riehl <kriehl@ethz.ch>, Justin Weiss <juweiss@ethz.ch> 
# #######                 Anastasios Kouvelas <kouvelas@ethz.ch>, Michail A. Makridis <mmakridis@ethz.ch>
# #######   YEAR :        2025
# #######   ORGANIZATION: Traffic Engineering Group (SVT), 
# #######                 Institute for Transportation Planning and Systems,
# #######                 ETH Zürich
# #############################################################################

"""
    This script contains optimizers used for finding optimal parametrization
    of traffic light controllers.
"""




# #############################################################################
# ###### IMPORTS ##############################################################
# #############################################################################
from bayes_opt import BayesianOptimization
from RunSimulation import Simulation
import multiprocessing
import os
import sys
import numpy as np
import csv
# Set SUMO path if available
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))




# #############################################################################
# ## DEFINE SEEDS AND MAIN FCT
# #############################################################################
"""
Run simulations across all seeds with given parameter set.
Aggregates results.
"""
SEEDS = [41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60]
def main(adaptation_cycle,adaptation_green,green_thresh,adaptation_offset,offset_thresh):
    alpha = 0.5518
    Changetime =2
    Thresholdtime = 5
    # Prepare partial simulation
    param_sets = [
        (seed, adaptation_cycle, adaptation_green, green_thresh, adaptation_offset, offset_thresh,alpha, Changetime, Thresholdtime)
        for seed in SEEDS
    ]
    # Parallel execution using multiprocessing
    with multiprocessing.Pool(min(len(SEEDS), os.cpu_count())) as pool:
        results = pool.map(Simulation, param_sets)
    results_array = np.array(results)
    mean_results = np.mean(results_array, axis=0)
    std_results = np.std(results_array, axis=0)
    # Print Aggregated Results
    print("\n=== Aggregated Results ===")
    print(f"Mean Total Throughput:     {mean_results[0]:.2f} ± {std_results[0]:.2f} veh")
    print(f"Mean Total Flow:           {mean_results[1]:.2f} ± {std_results[1]:.2f} veh/h")
    print(f"Mean Avg Speed:            {mean_results[2]:.2f} ± {std_results[2]:.2f} m/s")
    print(f"Mean Avg Density:          {mean_results[3]:.2f} ± {std_results[3]:.2f} veh")
    print(f"Mean Avg Delay:            {mean_results[4]:.2f} ± {std_results[4]:.2f} sec")
    print(f"Mean Avg Delay Sideroad:   {mean_results[5]:.2f} ± {std_results[5]:.2f} sec")
    print(f"Mean Avg Delay Mainroad:   {mean_results[6]:.2f} ± {std_results[6]:.2f} sec")
    print(f"Mean Max Delay:            {mean_results[7]:.2f} ± {std_results[7]:.2f} sec")
    print(f"Mean Total Travel Time:    {mean_results[8]:.2f} ± {std_results[8]:.2f} sec")
    print(f"Mean Gini Total:           {mean_results[9]:.3f} ± {std_results[9]:.3f}")
    print(f"Mean Gini Sideroad:        {mean_results[10]:.3f} ± {std_results[10]:.3f}")
    print(f"Mean Gini Mainroad:        {mean_results[11]:.3f} ± {std_results[11]:.3f}")
    # Use negative Metric as Cost for Optimization
    cost = -1 * mean_results[9]
    # Prepare row for CSV logging
    csv_row = [
     adaptation_cycle, adaptation_green, green_thresh, adaptation_offset, offset_thresh, alpha, Changetime, Thresholdtime
     ] + mean_results.tolist() + std_results.tolist() + [cost] # Add cost at end
    # Write or append to CSV
    csv_file = "bayes_opt_log.csv"
    file_exists = os.path.isfile(csv_file)
    with open(csv_file, mode='a', newline='') as file:
         writer = csv.writer(file)
         if not file_exists:
             writer.writerow([
                 'adaptation_cycle', 'adaptation_green', 'green_thresh', 'adaptation_offset', 'offset_thresh',
                 'alpha','Changetime','Thresholdtime',"Total Throughput", "Total Flow", "Avg Speed", "Avg Density", "Avg Delay",
                 "Avg Delay Sideroad", "Avg Delay Mainroad", "Max Delay", "Total Travel Time",
                 "Gini Total", "Gini Sideroad", "Gini Mainroad", "Cost"
             ])
         writer.writerow(csv_row)
    return cost




# #############################################################################
# ## ENTRY POINT AND BAYESIAN OPTIMIZER
# #############################################################################
if __name__ == "__main__":
    # Optional: Use Bayesian Optimization to find best parameter configuration
    optimizer = BayesianOptimization(
        f=main,
        pbounds={
            'adaptation_cycle': (10, 50),
            'adaptation_green': (5, 20),
            'green_thresh': (0, 5),
            'adaptation_offset': (0.1, 0.9),
            'offset_thresh': (0, 0.6)
            #'alpha': (0.2,0.98),
            #'Changetime': (2,8),
            #'Thresholdtime': (15,55)
        },
        random_state=42,
    )
    optimizer.maximize(init_points=15, n_iter=120)
    print("\n=== BEST PARAMETERS ===")
    print(f"Best Params: {optimizer.max['params']}")
    print(f"Best Score:  {optimizer.max['target']}")
"""
#Manual call
main(39.2797576724562, 13.979877262955549, 0.7799726016810132, 0.22481491235394924, 0.03485016730091967, 4.2472407130841745, 53.02857225639664)
"""