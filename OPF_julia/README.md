<p align="left">
  <img src="OPF_julia.png" alt="Logo" width="350">
</p>   

## AC/DC Optimal Power Flow in Julia

⚠️ **Current Status:** This is an **early-stage Version 0.1, Still building and waiting for uploading** ... more features coming soon! 🚀 

📦 **Required Julia Packages**

| **Package**        | **Version**  | **Description**                                      |
|-------------------|------------|--------------------------------------------------|
| 📄 **JuMP.jl**    | `1.23`      | Mathematical optimization modeling               |
| 📄 **Gurobi.jl**  | `1.0.0`     | Gurobi solver interface (requires license)      |
| 📄 **CSV.jl**     | `0.10.15`   | Reading/writing CSV files                        |
| 📄 **DataFrames.jl** | `1.7.0`  | Handling tabular data in Julia                  |

To install them, run:

```julia
using Pkg
Pkg.add(["JuMP", "Gurobi", "CSV", "DataFrames"])
