function dcNet = create_dc(case_name)
% CREATE_DC Reads DC grid data from CSV files.
%
%   dcNet = CREATE_DC(case_name) reads the DC grid data from CSV files
%   with the given case_name prefix. The following CSV files are required:
%       case_name_baseMW_dc.csv
%       case_name_pol_dc.csv
%       case_name_bus_dc.csv
%       case_name_branch_dc.csv
%       case_name_conv_dc.csv
%
%   The function returns a structure dcNet with the fields:
%       dcNet.baseMW    - Scalar base MW value 
%       dcNet.pol       - Scalar pol value 
%       dcNet.bus       - DC bus data matrix
%       dcNet.branch    - DC branch data matrix
%       dcNet.converter - DC converter data matrix

    base_path = pwd;
    
    %---------------------------List .csv files----------------------------
    required_files = { ...
        [case_name '_baseMW_dc.csv'], ...
        [case_name '_pol_dc.csv'], ...
        [case_name '_bus_dc.csv'], ...
        [case_name '_branch_dc.csv'], ...
        [case_name '_conv_dc.csv'] ...
    };

    %-------------------------Check missing files--------------------------
    missing_files = {};
    for i = 1:length(required_files)
        file_path = fullfile(base_path, required_files{i});
        if ~isfile(file_path)
            missing_files{end+1} = required_files{i};
        end
    end
    if ~isempty(missing_files)
        error('Missing required DC files: %s', strjoin(missing_files, ', '));
    end

    %------------------------Read .csv file data---------------------------
    dcNet.baseMW = readmatrix(fullfile(base_path, ...
        [case_name '_baseMW_dc.csv']));

    dcNet.pol = readmatrix(fullfile(base_path, ...
        [case_name '_pol_dc.csv']));

    dcNet.bus = readmatrix(fullfile(base_path, ...
        [case_name '_bus_dc.csv']));

    dcNet.branch = readmatrix(fullfile(base_path, ...
        [case_name '_branch_dc.csv']));

    dcNet.converter = readmatrix(fullfile(base_path, ...
        [case_name '_conv_dc.csv']));
end


