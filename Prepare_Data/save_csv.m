function save_csv(mpc_merged, folder_direct)
%% Purpose: save mpc_merged structure data to CSV files
%% Inputs: mpc_merged- structure data; folder_direct- storage directory, if empty
%%         saves to the current directory
%% Outputs: CVS files- including bus_file, gen_file, branch_file, gencost_file

    prefix = input('Enter a prefix for the CSV filenames: ', 's');

    if nargin < 2 || isempty(folder_direct)
        folder_direct = pwd; 
    end

    if ~isfolder(folder_direct)
        mkdir(folder_direct);
    end

    MVA_file = fullfile(folder_direct, [prefix '_MVA.csv']);
    bus_file = fullfile(folder_direct, [prefix '_bus.csv']);
    gen_file = fullfile(folder_direct, [prefix '_gen.csv']);
    branch_file = fullfile(folder_direct, [prefix '_branch.csv']);
    gencost_file = fullfile(folder_direct, [prefix '_gencost.csv']);

    writematrix(mpc_merged.baseMVA, MVA_file);
    writematrix(mpc_merged.bus, bus_file);
    writematrix(mpc_merged.gen, gen_file);
    writematrix(mpc_merged.branch, branch_file);
    writematrix(mpc_merged.gencost, gencost_file);
   
    fprintf('Merged AC grid data has already been saved to:\n');
    fprintf('  %s\n', MVA_file);
    fprintf('  %s\n', bus_file);
    fprintf('  %s\n', gen_file);
    fprintf('  %s\n', branch_file);
    fprintf('  %s\n', gencost_file);
end