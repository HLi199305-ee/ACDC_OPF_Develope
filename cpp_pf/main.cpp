// References:
// [1] J.Beerten, S.Cole and R.Belmans, "Generalized Steady-State VSC MTDC Model 
// for Sequential AC / DC Power Flow Algorithms, " in IEEE Transactions on Power Systems, 
// vol. 27, no. 2, pp. 821 - 829, May 2012, doi : 10.1109 / TPWRS.2011.2177867.
// [2]  Mauro Escobar, https ://github.com/me2533/acopf
// [3]  Matacdc1.0 unser's manual, https://www.esat.kuleuven.be/electa/teaching/matacdc/MatACDCManual

#include "solve_opf.h"
#include "BenchmarkTools.h"

int main() {
    solve_opf("mtdc3slack_a", "ac14ac57");
    
    std::vector<std::string> test_cases = { "ac14ac57", "ac57ac118", "ac9ac14" };
    std::string dc_case = "mtdc3slack_a";
    std::vector<BenchmarkResult> results;

    run_benchmark(test_cases, dc_case, results);
    print_summary(results);
    save_results_csv(results, "benchmark_cpp.csv");

    return 0;
}