#include "benchmarkTools.h"
#include "solve_opf.h"
int main() {
    solve_opf("mtdc3slack_a", "ac14ac57",
        /*vscControl*/ true,
        /*writeTxt  */ false,
        /*plotResult*/ false);

   /* std::vector<std::string> test_cases = { "ac9ac14" };
    std::string dc_case = "mtdc3slack_a";
    std::vector<BenchmarkResult> results;

    run_benchmark(test_cases, dc_case, results);
    print_summary(results);
    save_results_csv(results, "benchmark_cpp.csv");*/
}