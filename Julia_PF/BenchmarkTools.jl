using BenchmarkTools
using Statistics
using LinearAlgebra
using CUDA

# Include the same files as in main.jl
cd(dirname(@__FILE__))
include("create_ac.jl") # define interconnected ac grids
include("create_dc.jl") # define multi-terminal dc grid
include("makeYbus.jl") # calculate Bus Admittance Matrix
include("params_dc.jl") # obtain ac grid parameters 
include("params_ac.jl") # obtain dc grid parameters 
include("solve_opf.jl") # solve ac/dc OPF

# List of test cases
test_cases = ["ac14ac57", "ac57ac118", "ac9ac14"]

# Store results
results = []

println("\n====== AC/DC OPF Benchmark ======\n")

for case in test_cases
    println("\n Running test case: $case")
    
    # Warm-up run (to compile code and load data)
    solve_opf(case, "mtdc3slack_a")
    
    # Benchmark with parameters suitable for optimization problems
    bench = @benchmarkable solve_opf($case, "mtdc3slack_a") seconds=30 samples=10 evals=1
    
    # Run the benchmark
    b = run(bench)
    
    # Extract statistics
    time_median = median(b.times) / 1e9  # Convert nanoseconds to seconds
    time_mean = mean(b.times) / 1e9
    time_min = minimum(b.times) / 1e9
    time_max = maximum(b.times) / 1e9
    mem_median = median(b.memory) / 1e6  # Convert bytes to MB
    
    # Parallelization Check
    num_threads = Threads.nthreads()
    
    # GPU Utilization 
    gpu_enabled = CUDA.functional() ? "Yes" : "No"
    
    # Store results
    push!(results, (case, time_median, time_mean, time_min, time_max, mem_median, num_threads, gpu_enabled))
    
    # Print results
    println("Execution Time (median): ", round(time_median, digits=3), " seconds")
    println("Execution Time (mean): ", round(time_mean, digits=3), " seconds")
    println("Execution Time Range: ", round(time_min, digits=3), " - ", round(time_max, digits=3), " seconds")
    println("Memory Usage (median): ", round(mem_median, digits=2), " MB")
    println("Parallel Threads Used: ", num_threads)
    println("GPU Acceleration: ", gpu_enabled)
end

# Print final comparison table
println("\n====== Benchmark Results Summary ======\n")
println("| System     | Time Median (s) | Time Mean (s) | Time Min (s) | Time Max (s) | Memory (MB) | Threads | GPU |")
println("|------------|-----------------|---------------|--------------|--------------|-------------|---------|-----|")
for (case, tmed, tmean, tmin, tmax, mem, threads, gpu) in results
    println("| $(lpad(case, 10)) | $(lpad(round(tmed, digits=3), 15)) | $(lpad(round(tmean, digits=3), 13)) | $(lpad(round(tmin, digits=3), 12)) | $(lpad(round(tmax, digits=3), 12)) | $(lpad(round(mem, digits=2), 11)) | $(lpad(threads, 7)) | $(lpad(gpu, 3)) |")
end