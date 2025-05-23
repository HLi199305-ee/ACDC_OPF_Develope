#ifndef BENCHMARKTOOLS_H
#define BENCHMARKTOOLS_H

#include <string>
#include <vector>

struct BenchmarkResult {
    std::string case_name;
    double median_time;
    double min_time;
    double max_time;
    double peak_memory;
    double relative_memory;
};

void run_benchmark(const std::vector<std::string>& test_cases, const std::string& dc_case, std::vector<BenchmarkResult>& results);
void print_summary(const std::vector<BenchmarkResult>& results);
void save_results_csv(const std::vector<BenchmarkResult>& results, const std::string& filename);
double get_peak_memory_mb();

#endif // BENCHMARKTOOLS_H
