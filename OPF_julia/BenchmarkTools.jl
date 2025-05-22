using BenchmarkTools
using Statistics
using DelimitedFiles

# Include required files
cd(dirname(@__FILE__))
include("create_ac.jl")
include("create_dc.jl")
include("makeYbus.jl")
include("params_dc.jl")
include("params_ac.jl")
include("solve_opf.jl")
include("viz_opf.jl")

# Robust memory measurement function
function get_peak_memory()
    try
        # Linux/macOS version
        return parse(Int, split(read(`ps -p $(getpid()) -o rss=`, String))[1]) / 1024.0
    catch
        try
            # Windows version
            cmd_output = read(`wmic process where processid=$(getpid()) get WorkingSetSize`, String)
            mem_bytes = parse(Int, split(cmd_output)[2])
            return mem_bytes / (1024.0 * 1024.0)
        catch e
            @warn "Memory measurement failed: $e"
            return 0.0
        end
    end
end

# Benchmark results structure
struct BenchmarkResult
    case_name::String
    median_time::Float64
    min_time::Float64
    max_time::Float64
    peak_memory::Float64
    relative_memory::Float64
end

# Test cases
test_cases = ["ac14ac57", "ac57ac118", "ac9ac14", "ac118ac300"]
dc_case = "mtdc3slack_a"

# Store results
results = BenchmarkResult[]

println("\n====== AC/DC OPF Julia Benchmark ======\n")
println("Configuration:")
println("  DC Case: $dc_case")
println("  Threads: $(Threads.nthreads())")
println("  Samples: 5\n")

# Warm-up phase (3 runs)
println("Performing warm-up runs...")
for i in 1:3
    solve_opf(dc_case, test_cases[1], plotResult = false)
    GC.gc() # Force garbage collection
end

for case in test_cases
    println("\nBenchmarking case: $case")
    
    run_times = Float64[]
    memory_usage = Float64[]
    
    # Measurement runs (5 iterations)
    for i in 1:5
        GC.gc()
        mem_before = get_peak_memory()
        
        # Time the actual execution
        t = @elapsed begin
            solve_opf(dc_case, test_cases[1], plotResult = false)
        end
        
        mem_after = get_peak_memory()
        
        push!(run_times, t)
        push!(memory_usage, max(0.0, mem_after - mem_before)) # Ensure non-negative
        
        println("  Run $i: ", round(t, digits=3), " s, ", 
               round(memory_usage[end], digits=1), " MB")
    end
    
    # Calculate statistics
    sort!(run_times)
    sort!(memory_usage)
    
    median_time = median(run_times)
    min_time = minimum(run_times)
    max_time = maximum(run_times)
    peak_mem = maximum(memory_usage)  # Use max observed memory
    
    # Calculate relative memory (compared to first case)
    if !isempty(results) && results[1].peak_memory > 0
        relative_mem = peak_mem / results[1].peak_memory
    else
        relative_mem = isempty(results) ? 1.0 : NaN
    end
    
    push!(results, BenchmarkResult(case, median_time, min_time, max_time, peak_mem, relative_mem))
end

# Print results table
println("\n====== Benchmark Results Summary ======")
println("| Case       | Time (s)         | Memory Usage       |")
println("|------------|------------------|--------------------|")
println("|            | Median | Range   | Absolute | Relative |")
println("|------------|--------|---------|----------|----------|")

for res in results
    println("| $(lpad(res.case_name, 10)) | ",
            "$(lpad(round(res.median_time, digits=3), 6)) | ",
            "$(lpad(round(res.min_time, digits=3), 5))-",
            "$(lpad(round(res.max_time, digits=3), 3)) | ",
            "$(lpad(round(res.peak_memory, digits=1), 8)) | ",
            "$(lpad(isnan(res.relative_memory) ? "N/A" : string(round(res.relative_memory, digits=2)), 8))x |")
end

# Save results to CSV
open("benchmark_julia.csv", "w") do io
    writedlm(io, [["case_name", "median_time", "min_time", "max_time", 
                   "peak_memory", "relative_memory"]], ',')
    for res in results
        writedlm(io, [[res.case_name, res.median_time, res.min_time, 
                      res.max_time, res.peak_memory, res.relative_memory]], ',')
    end
end

println("\nResults saved to benchmark_julia.csv")

