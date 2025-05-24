<p align="left">
  <img src="assets\OPF_cpp.png" alt="Logo" width="500">
</p>   

# ⚡ C++ Implementation of AC/DC OPF

This folder contains the C++-based implementation of the AC/DC OPF model using Gurobi C++ API optimization framework and the Gurobi solver.

> **Current Status:** This is an **early-stage Version 0.1, Still building** ... more features is ongoing !

---

## 📁 File Overview

| File | Description |
|------|-------------|
| `main.cpp` | Main script to run the AC/DC OPF. |
| `create_ac.cpp / create_ac.h` | Load AC grid data from `.csv`. |
| `create_dc.cpp / create_dc.h` | Load DC grid and VSC converter data from `.csv`. |
| `params_ac.cpp / params_ac.h` | Extract parameters for AC-side modeling. |
| `params_dc.cpp / params_dc.h` | Extract parameters for DC-side modeling. |
| `csv_reader.cpp / csv_reader.h` | Read customed `.csv` files. |
| `makeYbus.cpp / makeYbus.h` | Construct AC admittance matrix using bus/branch data. |
| `solve_opf.cpp / solve_opf.h` | Core OPF formulation using Gurobi C++ API. |
| `viz_opf.cpp / viz_opf.h` | Visualize AC/DC power flows. |
| `benchmarkTools.cpp / benchmarkTools.h` | Tools to run performance tests across cases. |
| `acmtdcpfvcx.sln` | Visual Studio solution file. |

---

## 🔧 Dependencies

To run this project, the following tools must be installed:

- [Gurobi C++ API](https://docs.gurobi.com/projects/optimizer/en/current/reference/cpp.html): C++ API for optimization (license required).
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page): C++ library for linear algebra.
- [Matplot++](https://alandefreitas.github.io/matplotplusplus/): C++ plotting library.

---

## 🚀 Getting Started

1. Clone the repository and navigate to the `OPF_c++` folder:

    ```bash
    git clone https://github.com/YourUsername/ACDC_OPF.git
    cd ACDC_OPF/OPF_c++
    ```
2. We recomend use [vcpkg](https://github.com/microsoft/vcpkg) manage C++ library. Download **vcpkg**:
   ```bash
   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   ./bootstrap-vcpkg.bat
   ```
   Intergrate **vcpkg** with Visual Studio:
   ```bash
   ./vcpkg integrate install
   ```
3. Install  `Eign` and  `Matplot++` via  **vcpkg**. Open  `Powershell` or `CMD`, and navigate to  **vcpkg** direcotroy, like: 

    ```bash
   cd C:\XXX\vcpkg
   ```

   In the root directory of **vcpkg**, type:

   ```bash
   .\vcpkg install eigen3
   .\vcpkg install matplotplusplus
   ```

   `Gurobi` is not in **vcpkg**, you must download [Gurobi](https://www.gurobi.com/downloads/) manually and install it follow a [Instruction](https://support.gurobi.com/hc/en-us/articles/4534161999889-How-do-I-install-Gurobi-Optimizer).

    ---
   

## 🧩 Gurobi Configuration in Visual Studio IDE
   Next are details regarding Gurobi Configuration. Based on our local setup, the configruation is demostrated using  **Gurobi 9.5.2** and **Visual Studio 2022**.

<p align="left">
  <img src="assets\gurobi_config_f1.png" alt="Logo" width="450">
</p>  

<p align="left">
  <img src="assets\gurobi_config_f2.png" alt="Logo" width="450">
</p> 

<p align="left">
  <img src="assets\gurobi_config_f3.png" alt="Logo" width="450">
</p> 

<p align="left">
  <img src="assets\gurobi_config_f4.jpg" alt="Logo" width="800">
</p> 

<p align="left">
  <img src="assets\gurobi_config_f5.jpg" alt="Logo" width="800">
</p> 

<p align="left">
  <img src="assets\gurobi_config_f6.jpg" alt="Logo" width="800">
</p> 




## 🖨️  Expected Output

1. You will see the printed [AC/DC OPF results](assets/opf_result.txt) on the terminal window :

     ```txt
    =================================================================================
    |      AC Grids Bus Data                                                        |
    =================================================================================
     Area   Branch        Voltage            Generation                Load        
     #      #       Mag [pu]  Ang [deg]   Pg [MW]   Qg [MVAr]   Pd [MW]   Qd [MVAr]
    -----   -----   --------  ---------   --------  ---------   -------   ---------
       1       1      1.048      0.000    100.966    119.890     0.000       0.000
       1       2      1.100      0.000    150.474    299.987     0.000       0.000
       1       3      1.045      0.000    104.361     96.882     0.000       0.000
       1       4      1.011      0.000        -         -        0.000       0.000
       1       5      1.000      0.000        -         -       90.000      30.000
    
    ...... (omitted here)

2. You will see the plotted AC/DC OPF results like:

  <p align="left">
  <img src="assets\viz_C++.png" alt="Logo" width="700">
  </p>  

