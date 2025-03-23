"""
    params_dc(case_name::String) -> NamedTuple

Compute and return DC grid parameters by reading the DC grid data from CSV files.

This function extracts and computes parameters for the DC network from the
specified case.

Returns a NamedTuple with the following fields:
  - network_dc   : Dict. DC network containing all network data.
  - baseMW_dc    : Int64. DC network MW base.
  - pol_dc       : Int64. DC network pole.
  - nbuses_dc    : Int64. Number of DC buses.
  - nbranches_dc : Int64. Number of DC branches.
  - nconvs_dc    : Int64. Number of converters.
  - bus_dc       : Matrix{Float64}. DC bus data matrix.
  - branch_dc    : Matrix{Float64}. DC branch data matrix.
  - conv_dc      : Matrix{Float64}. VSC converter data matrix.
  - ybus_dc      : SparseMatrixCSC{Float64, Int64}. DC network admittance matrix (absolute values).
  - basekV_dc    : Vector{Float64}. AC voltage base for VSC converters.
  - fbus_dc      : Vector{Int64}. "From" bus indices for branches.
  - tbus_dc      : Vector{Int64}. "To" bus indices for branches.
  - rtf_dc       : Vector{Float64}. PCC side transformer resistances (p.u.).
  - xtf_dc       : Vector{Float64}. PCC side transformer reactances (p.u.).
  - bf_dc        : Vector{Float64}. Filter susceptances (p.u.).
  - rc_dc        : Vector{Float64}. AC terminal side converter resistances (p.u.).
  - xc_dc        : Vector{Float64}. AC terminal side converter reactances (p.u.).
  - ztfc_dc      : Vector{ComplexF64}. VSC impedances for PCC side to AC terminal side (p.u.).
  - gtfc_dc      : Vector{Float64}. Real parts of 1./ztfc_dc (p.u.).
  - btfc_dc      : Vector{Float64}. Imaginary parts of 1./ztfc_dc (p.u.).
  - aloss_dc     : Vector{Float64}. Converter loss parameters A (p.u.).
  - bloss_dc     : Vector{Float64}. Converter loss parameters B (p.u.).
  - closs_dc     : Vector{Float64}. Converter loss parameters C (p.u.).
  - convState_dc : Vector{Int64}. Converter state indicators (1 for inverter, 0 for rectifier).

See also: create_dc.jl, makeYbus.jl
"""
function params_dc(case_name::String) ::NamedTuple
    #--------------------------------------------------------------------------
    # Load DC grid data
    #--------------------------------------------------------------------------
    network_dc = create_dc(case_name)
    baseMW_dc = network_dc["baseMW"][1, 1]
    bus_dc = network_dc["bus"]
    branch_dc = network_dc["branch"][network_dc["branch"][:, 11] .== 1, :]
    conv_dc = network_dc["converter"]
    pol_dc = network_dc["pol"][1, 1]
    basekV_dc = conv_dc[:, 14]

    # Determine the number of buses, branches, and converters
    nbuses_dc = size(bus_dc, 1)
    nbranches_dc = size(branch_dc, 1)
    nconvs_dc = size(conv_dc, 1)

    # Extract branch "from" and "to" bus indices
    fbus_dc = Int.(branch_dc[:, 1])
    tbus_dc = Int.(branch_dc[:, 2])

    #--------------------------------------------------------------------------
    # Compute DC network admittance matrix (Ybus)
    #--------------------------------------------------------------------------
    ybus_dc = makeYbus(baseMW_dc, bus_dc, branch_dc)
    ybus_dc = abs.(ybus_dc) 

    #--------------------------------------------------------------------------
    # Extract VSC Impedance parameters
    # The equivalent circuit for the VSC converter is as follows:
    #
    # U_s  ● --[ r_tf + j*x_tf ]-- ● --[ r_c + j*x_c ]-- ● U_c --(VSC)--
    #     PCC                    Filter               AC terminal
    #
    # PCC side parameters (r_tf, x_tf, bf_dc) and AC terminal side 
    # parameters (r_c, x_c) are combined to form the overall converter impedance.
    #--------------------------------------------------------------------------
    # PCC side transformer parameters
    rtf_dc = conv_dc[:, 9]    
    xtf_dc = conv_dc[:, 10]  
    
    # Filter parameters
    bf_dc  = conv_dc[:, 11]   
    
    # AC terminal side converter impedance parameters
    rc_dc = conv_dc[:, 12]   
    xc_dc  = conv_dc[:, 13]   

    # Combined converter impedance and admittance
    ztfc_dc = Complex.(rtf_dc .+ rc_dc, xtf_dc .+ xc_dc)
    gtfc_dc = real.(1 ./ ztfc_dc)
    btfc_dc = imag.(1 ./ ztfc_dc)

    #--------------------------------------------------------------------------
    # Converter Loss Parameters and State Determination
    #--------------------------------------------------------------------------
    aloss_dc = zeros(nconvs_dc)
    bloss_dc = zeros(nconvs_dc)
    closs_dc = zeros(nconvs_dc)  
    convState_dc = zeros(Int, nconvs_dc)  

    for i in 1:nconvs_dc
        if conv_dc[i, 6] >= 0  # Inverter state
            closs_dc[i] = conv_dc[i, 22]
            convState_dc[i] = 1
        else  # Rectifier state
            closs_dc[i] = conv_dc[i, 21]
            convState_dc[i] = 0
        end
    end

    # Convert loss parameters to per unit values
    closs_dc .= closs_dc ./ (basekV_dc.^2 / baseMW_dc)
    aloss_dc .= conv_dc[:, 19] ./ baseMW_dc
    bloss_dc .= conv_dc[:, 20] ./ basekV_dc

    #--------------------------------------------------------------------------
    # Return a NamedTuple containing all computed DC parameters
    #--------------------------------------------------------------------------
    return NamedTuple((
       network_dc   = network_dc,
       baseMW_dc    = baseMW_dc,
       bus_dc       = bus_dc,
       branch_dc    = branch_dc,
       conv_dc      = conv_dc,
       pol_dc       = pol_dc,
       nbuses_dc    = nbuses_dc,
       nbranches_dc = nbranches_dc,
       nconvs_dc    = nconvs_dc,
       fbus_dc      = fbus_dc,
       tbus_dc      = tbus_dc,
       ybus_dc      = ybus_dc,
       rtf_dc       = rtf_dc,
       xtf_dc       = xtf_dc,
       bf_dc        = bf_dc,
       rc_dc        = rc_dc,
       xc_dc        = xc_dc,
       ztfc_dc      = ztfc_dc,
       gtfc_dc      = gtfc_dc,
       btfc_dc      = btfc_dc,
       aloss_dc     = aloss_dc,
       bloss_dc     = bloss_dc,
       closs_dc     = closs_dc,
       convState_dc    = convState_dc,
       basekV_dc    = basekV_dc
    ))
end

