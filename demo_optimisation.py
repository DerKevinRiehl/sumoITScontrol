############## IMPORTS
import warnings
import sys
import subprocess
import numpy as np
warnings.filterwarnings("ignore")

from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm


############## OPTIMISATION
searchspace = [
    [5, 10, 15, 20, 25, 30, 35, 40, 45, 50], # K_P
    [0], # K_I
]
seeds = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
n_threads = 10

# searchspace[0] = [30]
n_thr = 10

def run_single(seed, param_1):
    cmd = [
        sys.executable,
        "demo_optimisation_execute_script.py",
        "--K_P", str(param_1),
        "--K_I", "0",
        "--SEED", str(seed)
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    return seed, param_1, float(result.stdout)

results = {}

# Create all jobs first
jobs = [(seed, param_1) for seed in seeds for param_1 in searchspace[0]]

with ThreadPoolExecutor(max_workers=n_thr) as executor:
    futures = [executor.submit(run_single, seed, param_1) for seed, param_1 in jobs]

    # tqdm progress bar over completed futures
    for future in tqdm(as_completed(futures), total=len(futures)):
        seed, param_1, value = future.result()

        if param_1 not in results:
            results[param_1] = []
        results[param_1].append(value)

# Aggregate mean/std
for param_1 in results:
    values = results[param_1]
    results[param_1] = [float(np.mean(values)), float(np.std(values)/10)]
for param_1 in results:
    print(param_1, results[param_1]/10)

# Test

""" 
5 [13.041207561889655, 24.384136099917487]
10 [12.481044009619161, 22.775428870985557]
15 [12.894482957963698, 23.681340957184524]
20 [12.636514045816126, 23.42537274130512]
25 [13.35457413165786, 25.804242493244146]
30 [13.319907248459165, 26.485499285459422]
35 [13.757885740292059, 26.943198053512983]
40 [13.838714296130345, 26.991941556358597]
45 [14.165949450774553, 26.9682343548254]
50 [13.982141129842612, 26.979434403798134]
"""