function sorted_ac = mpc_sorted(mpc)

%% Purpose: sort the bus number of mpc files
%% Input: mpc (MATPOWER case struct)
%% Output: mpc file with sorted bus number

    if ~isstruct(mpc) 
        error('Input must be a valid MATPOWER case struct.');
    end
    
    % Get the original bus_i and sort them
    old_bus_ids = mpc.bus(:, 1);
    sorted_bus_ids = sort(old_bus_ids);
    
    % Creating a mapping: old_bus_id -> new_bus_id
    bus_map = containers.Map(sorted_bus_ids, 1:length(sorted_bus_ids));
    
    % Update bus_i in mpc.bus 
    sorted_ac = mpc;
    sorted_ac.bus(:, 1) = cell2mat(values(bus_map, num2cell(mpc.bus(:, 1))));
    
    % Update fbus and tbus in mpc.bus
    sorted_ac.branch(:, 1) = cell2mat(values(bus_map, num2cell(mpc.branch(:, 1))));
    sorted_ac.branch(:, 2) = cell2mat(values(bus_map, num2cell(mpc.branch(:, 2))));
    
    % Update bus in mpc.gen
    sorted_ac.gen(:, 1) = cell2mat(values(bus_map, num2cell(mpc.gen(:, 1))));
       
end