% function benchmark_power_flow()
%     tic;
%     solve_opf('mtdc3slack_a', 'ac14ac57');  % Call your power flow function
%     exec_time = toc;
%     
%     mem_info = memory;
%     fprintf('Execution Time: %.6f seconds\n', exec_time);
%     fprintf('Memory Usage: %.2f MB\n', mem_info.MemUsedMATLAB / 1e6);
% end
function benchmark_power_flow()
    % AC/DC OPF Benchmark for MATLAB
    fprintf('\n====== AC/DC OPF MATLAB Benchmark ======\n\n');

    % List of test cases
    test_cases = {'ac14ac57', 'ac57ac118', 'ac9ac14'};
    dc_case = 'mtdc3slack_a';  % Fixed DC case for all tests
    
    % Store results
    results = cell(length(test_cases), 1);
    
    fprintf('\n====== AC/DC OPF MATLAB Benchmark ======\n');
    
    for i = 1:length(test_cases)
        case_name = test_cases{i};
        fprintf('\n Running test case: %s\n', case_name);
        
        % to compile code and load data
        solve_opf(dc_case, case_name);
        
        % Benchmark timing
        num_runs = 5;  % Number of runs for more stable timing
        times = zeros(num_runs, 1);
        
        for j = 1:num_runs
            tstart = tic;
            solve_opf(dc_case, case_name);
            times(j) = toc(tstart);
        end
        
        % Calculate statistics
        time_median = median(times);
        time_mean = mean(times);
        time_min = min(times);
        time_max = max(times);
        
        % Memory measurement (approximate)
        mem_before = memory;
        solve_opf(dc_case, case_name);
        mem_after = memory;
        mem_usage = (mem_after.MemUsedMATLAB - mem_before.MemUsedMATLAB) / 1e6;  % MB
        
        % Parallelization info
        pool = gcp('nocreate');
        if isempty(pool)
            num_workers = 0;
        else
            num_workers = pool.NumWorkers;
        end
        
        % GPU info
        gpu_info = gpuDevice;
        gpu_enabled = gpu_info.DeviceSupported;
        
        % Store results
        results{i} = struct(...
            'case', case_name, ...
            'time_median', time_median, ...
            'time_mean', time_mean, ...
            'time_min', time_min, ...
            'time_max', time_max, ...
            'memory_mb', mem_usage, ...
            'workers', num_workers, ...
            'gpu', gpu_enabled);
        
        % Print results
        fprintf('Execution Time (median): %.3f seconds\n', time_median);
        fprintf('Execution Time (mean):   %.3f seconds\n', time_mean);
        fprintf('Execution Time Range:    %.3f - %.3f seconds\n', time_min, time_max);
        fprintf('Memory Usage:           %.2f MB\n', mem_usage);
        fprintf('Parallel Workers:       %d\n', num_workers);
        fprintf('GPU Acceleration:       %s\n', string(gpu_enabled));
    end
    
    % Print summary table
    fprintf('\n====== Benchmark Results Summary ======\n');
    fprintf('| System     | Time Median (s) | Time Mean (s) | Time Min (s) | Time Max (s) | Memory (MB) | Workers | GPU |\n');
    fprintf('|------------|-----------------|---------------|--------------|--------------|-------------|---------|-----|\n');
    
    for i = 1:length(results)
        r = results{i};
        fprintf('| %-10s | %15.3f | %13.3f | %12.3f | %12.3f | %11.2f | %7d | %3s |\n', ...
            r.case, r.time_median, r.time_mean, r.time_min, r.time_max, ...
            r.memory_mb, r.workers, string(r.gpu));
    end
    
    % Save results to file
    save('benchmark_results.mat', 'results');
end        