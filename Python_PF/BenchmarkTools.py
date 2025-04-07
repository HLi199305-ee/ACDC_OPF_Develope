import time
import os
import psutil
import numpy as np
import pandas as pd
from statistics import median
from opf_acdc import solve_opf

def get_memory_mb():
    """Returns memory usage in MB."""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / 1024 / 1024

def benchmark_power_flow(ac_cases, dc_case):
    print("\n====== AC/DC OPF Python Benchmark ======\n")
    print(f"Configuration:\n  DC Case: {dc_case}\n  Samples: 5\n")

    print("Performing warm-up runs...")
    for _ in range(3):
        solve_opf(ac_cases[0], dc_case)  # Warm-up with (acgrid_name, dcgrid_name)

    results = []

    for ac_case in ac_cases:
        print(f"\nBenchmarking case: {ac_case}")
        times = []
        mem_usages = []

        for i in range(5):
            # Check for DC file
            expected_file = f"{dc_case}_baseMW_dc.csv"
            if not os.path.exists(expected_file):
                print(f"Skipping: Missing required file: {expected_file}")
                times.append(0.0)
                mem_usages.append(0.0)
                continue

            mem_before = get_memory_mb()
            start = time.time()

            try:
                solve_opf(ac_case, dc_case)  # Call with (acgrid_name, dcgrid_name)
            except Exception as e:
                print("Exception during optimization:", e)
                continue

            elapsed = time.time() - start
            mem_after = get_memory_mb()
            mem_used = max(0.0, mem_after - mem_before)

            times.append(elapsed)
            mem_usages.append(mem_used)
            print(f"  Run {i+1}: {elapsed:.3f}s, {mem_used:.1f} MB")

        if times:
            med_time = median(times)
            min_time = min(times)
            max_time = max(times)
            peak_memory = max(mem_usages)
            relative_memory = (peak_memory / results[0]['peak_memory']) if results else 1.0
        else:
            med_time = min_time = max_time = peak_memory = relative_memory = float('nan')

        results.append({
            'case_name': ac_case,
            'median_time': med_time,
            'min_time': min_time,
            'max_time': max_time,
            'peak_memory': peak_memory,
            'relative_memory': relative_memory
        })

    return results

def print_results_table(results):
    print("\n====== Benchmark Results Summary ======")
    print("| Case       | Time (s)         | Memory Usage       |")
    print("|------------|------------------|--------------------|")
    print("|            | Median | Range   | Absolute | Relative |")
    print("|------------|--------|---------|----------|----------|")

    for r in results:
        rel_mem = f"{r['relative_memory']:.2f}x" if not np.isnan(r['relative_memory']) else "N/A"
        print(f"| {r['case_name']:<10} | {r['median_time']:.3f} | "
              f"{r['min_time']:.3f}-{r['max_time']:.3f} | "
              f"{r['peak_memory']:.1f} | {rel_mem:>8} |")

def save_results_csv(results, filename='benchmark_python.csv'):
    df = pd.DataFrame(results)
    df.to_csv(filename, index=False)
    print(f"\nResults saved to {filename}")

# === usage ===
if __name__ == "__main__":
    dc_case = "mtdc3slack_a"
    ac_cases = ["ac14ac57", "ac57ac118", "ac9ac14"]

    results = benchmark_power_flow(ac_cases, dc_case)
    print_results_table(results)
    save_results_csv(results)
