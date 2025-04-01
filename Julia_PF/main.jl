# References:
# [1] J.Beerten, S.Cole and R.Belmans, "Generalized Steady-State VSC MTDC Model 
# for Sequential AC / DC Power Flow Algorithms, " in IEEE Transactions on Power Systems, 
# vol. 27, no. 2, pp. 821 - 829, May 2012, doi : 10.1109 / TPWRS.2011.2177867.
# [2]  Mauro Escobar, https ://github.com/me2533/acopf
# [3]  Matacdc1.0 unser's manual, https://www.esat.kuleuven.be/electa/teaching/matacdc/MatACDCManual

cd(dirname(@__FILE__))
include("create_ac.jl") # define interconnected ac grids
include("create_dc.jl") # define multi-terminal dc grid
include("makeYbus.jl") # calculate Bus Admittance Matrix
include("params_dc.jl") # obtain ac grid parameters 
include("params_ac.jl") # obtain dc grid parameters 
include("solve_opf.jl") # solve ac/dc OPF

result_opf = solve_opf("ac14ac57", "mtdc3slack_a")
