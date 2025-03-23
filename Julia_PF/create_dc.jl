using CSV
using DataFrames

"""
    create_dc(case_name::String) -> Dict{String,Any}

Load DC grid data from CSV files with the given `case_name` prefix.

This function expects the following files to be present in the same directory as this source file:
  - `<case_name>_baseMW_dc.csv`
  - `<case_name>_pol_dc.csv`
  - `<case_name>_bus_dc.csv`
  - `<case_name>_branch_dc.csv`
  - `<case_name>_conv_dc.csv`

Returns a dictionary `dc` with the following keys:
  - "baseMW"    : Scalar base MW value.
  - "pol"       : Scalar pole value.
  - "bus"       : Matrix containing bus data.
  - "branch"    : Matrix containing branch data.
  - "converter" : Matrix containing converter data.
"""
function create_dc(case_name::String)
    # Set the base path to the directory containing this file.
    base_path = @__DIR__
    
    # Initialize the dictionary to store DC grid data.
    dc = Dict{String,Any}()

    # List of required CSV files.
    required_files = [
        "$(case_name)_baseMW_dc.csv",
        "$(case_name)_pol_dc.csv",
        "$(case_name)_bus_dc.csv",
        "$(case_name)_branch_dc.csv",
        "$(case_name)_conv_dc.csv"
    ]
    
    # Check for missing files.
    missing_files = filter(f -> !isfile(joinpath(base_path, f)), required_files)
    if !isempty(missing_files)
        error("Missing required DC files: " * join(missing_files, ", "))
    end
    
    # Load each CSV file
    read_csv_matrix(file) = Matrix(DataFrame(CSV.File(joinpath(base_path, file); header=false)))
    
    baseMW = read_csv_matrix("$(case_name)_baseMW_dc.csv")
    dc["baseMW"] = baseMW[1, 1]
    
    pol = read_csv_matrix("$(case_name)_pol_dc.csv")
    dc["pol"] = pol[1, 1]
    
    dc["bus"]       = read_csv_matrix("$(case_name)_bus_dc.csv")
    dc["branch"]    = read_csv_matrix("$(case_name)_branch_dc.csv")
    dc["converter"] = read_csv_matrix("$(case_name)_conv_dc.csv")
    
    return dc
end

