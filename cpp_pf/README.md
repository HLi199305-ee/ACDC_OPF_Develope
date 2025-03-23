<p align="left">
  <img src="OPF_cpp.png" alt="Logo" width="350">
</p>   

## AC/DC Optimal Power Flow in C++

🚀 **Current Status:** This is an **early-stage Version 0.1, under active development** … more features coming soon! ⚡

📦 **Required C++ Libraries**

| **Package** | **Version** | **Description**                                                  |
|-------------|-------------|------------------------------------------------------------------|
| **Eigen**   |  3.4.0      |  C++ library for linear algebra and numerical computations. |
| **Gurobi**  |  9.5.2       | optimization solver (requires license). |

---

## Installing Eign with vcpkg
[vcpkg](https://github.com/microsoft/vcpkg)
1. **Install vcpkg**:
   ```bash
   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   .\bootstrap-vcpkg.bat  # on Windows

2. **Install Eign**:
   ./vcpkg install eigen3

## Configure Gurobi in Microsoft Visual Studio
You can find guidance (https://support.gurobi.com/hc/en-us/articles/360013194392-How-do-I-configure-a-new-Gurobi-C-project-with-Microsoft-Visual-Studio)
