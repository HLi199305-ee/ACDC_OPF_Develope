function merged_ac = mpc_merged(varargin)

    %% Purpose: merge multiple ac grids from matower mpc files
    %% Input: names of mpc files from MATPOWER data sources
    %% Output: merged mpc file used for acdcopf

    %% Initialization
    if ischar(varargin{1}) || isstring(varargin{1})
        sys_name = varargin{1};
        mpc = feval(sys_name);
        baseMVA = mpc.baseMVA; 
    else
        error('Input must be a valid case name string, e.g., ''case14''.');
    end

    mpc = mpc_sorted(mpc); % preprocess mpc
    fprintf('Reordered bus IDs for %s\n', sys_name);
        
    merged_ac.version = '2';
    merged_ac.baseMVA = baseMVA;
    merged_ac.bus = [mpc.bus, ones(size(mpc.bus, 1), 1)];
    merged_ac.gen = [mpc.gen, ones(size(mpc.gen, 1), 1)];
    merged_ac.branch = [mpc.branch, ones(size(mpc.branch, 1), 1)];
    merged_ac.gencost = [mpc.gencost, ones(size(mpc.gencost, 1), 1)];

    %% Merge
    for i = 2:nargin

        if ischar(varargin{i}) || isstring(varargin{i})
            sys_name = varargin{i};
            mpc = feval(sys_name);
        end

        if mpc.baseMVA ~= merged_ac.baseMVA
            error('baseMVA of the input cases is inconsistent. CANNOT be merged');
        end

        mpc = mpc_sorted(mpc); % preprocess mpc
        fprintf('Reordered bus IDs for %s\n', sys_name);

        mpc.bus = [mpc.bus, i*ones(size(mpc.bus, 1), 1)];
        mpc.gen = [mpc.gen, i*ones(size(mpc.gen, 1), 1)];
        mpc.branch = [mpc.branch, i*ones(size(mpc.branch, 1), 1)];
        mpc.gencost = [mpc.gencost, i*ones(size(mpc.gencost, 1), 1)];

        merged_ac.bus = [merged_ac.bus; mpc.bus];
        merged_ac.gen = [merged_ac.gen; mpc.gen];
        merged_ac.branch = [merged_ac.branch; mpc.branch];
        merged_ac.gencost = [merged_ac.gencost; mpc.gencost];
    end

end






