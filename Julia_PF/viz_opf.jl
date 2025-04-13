using Printf
using Graphs, GraphPlot
using Random
using GLMakie
using ColorSchemes, Colors
import GLMakie as G

# ---------------------------------------------------------------- #
function find_row_index(rowVec::AbstractVector, M::AbstractMatrix)
    for i in 1:size(M, 1)
        if all(M[i, :] .== rowVec)
            return i
        end
    end
    return nothing
end
# ---------------------------------------------------------------- #

"""
    viz_opf(bus_entire_ac, branch_entire_ac, bus_dc, branch_dc, conv_dc, gen_entire_ac,
            pgen_ac_k, qgen_ac_k, baseMVA_ac, vn2_ac_k, vn2_dc_k, pij_ac_k, qij_ac_k,
            pij_dc_k, ps_dc_k, baseMW_dc, pol_dc)

Visualizes AC/DC OPF results 

INPUTS:
  - bus_entire_ac:      Complete bus data from the AC network.
  - branch_entire_ac:   Complete branch data from the AC network.
  - gen_entire_ac:      Complete generator data from the AC network.
  - baseMVA_ac:         AC system base MVA value.
  - pgen_ac_k:          Optimized resutls of generator active power.
  - qgen_ac_k:          Optimized results of generator reactive power.
  - vn2_ac_k:           Optimized results of the squared AC voltage.
  - pij_ac_k:           Optimized results of the AC branch active power.
  - qij_ac_k:           Optimized results of the AC branch reactive power.

  - bus_dc:             Bus data from the DC network.
  - branch_dc:          Branch data from the DC network.
  - conv_dc:            Converter data.
  - baseMW_dc:          DC system base MW value.
  - pol_dc:             Polarity of the DC network
  - vn2_dc_k:           Optimized results of the DC network
  - pij_dc_k:           Optimized resutls of the DC branch active power.
  - ps_dc_k:            Optimized results of the VSC PCC active power.
  - qs_dc_k:            Optimized results of the VSC PCC reactive power.

OUTPUT:
  - Figure
"""
function viz_opf(bus_entire_ac, branch_entire_ac, bus_dc, branch_dc, conv_dc, gen_entire_ac,
                 pgen_ac_k, qgen_ac_k, baseMVA_ac, vn2_ac_k, vn2_dc_k, pij_ac_k, qij_ac_k,
                 pij_dc_k, ps_dc_k, qs_dc_k, baseMW_dc, pol_dc)

  ########## 1. Reorder AC Data ##########
  numBuses_ac = size(bus_entire_ac, 1)
  oldBusNums_ac = bus_entire_ac[:, 1]
  busAreas_ac   = bus_entire_ac[:, end]
  
  newBusNums_ac = collect(1:numBuses_ac)
  bus_entire_ac_new = copy(bus_entire_ac)
  bus_entire_ac_new[:, 1] .= newBusNums_ac  
  
  mappingMatrix_ac = hcat(oldBusNums_ac, busAreas_ac)
  branchAreas_ac = branch_entire_ac[:, end]
  
  newFrom = [ newBusNums_ac[find_row_index([branch_entire_ac[i, 1], branchAreas_ac[i]], mappingMatrix_ac)]
              for i in 1:size(branch_entire_ac, 1) ]
  newTo = [ newBusNums_ac[find_row_index([branch_entire_ac[i, 2], branchAreas_ac[i]], mappingMatrix_ac)]
            for i in 1:size(branch_entire_ac, 1) ]
  
  branch_entire_ac_new = copy(branch_entire_ac)
  branch_entire_ac_new[:, 1] .= newFrom
  branch_entire_ac_new[:, 2] .= newTo

  ########## 2. Reorder DC Data ##########
  numBuses_dc = size(bus_dc, 1)
  bus_dc_new = copy(bus_dc)
  newBusNums_dc = bus_dc[:, 1] .+ numBuses_ac
  bus_dc_new[:, 1] .= newBusNums_dc
  
  branch_dc_new = copy(branch_dc)
  branch_dc_new[:, 1] .= branch_dc[:, 1] .+ numBuses_ac
  branch_dc_new[:, 2] .= branch_dc[:, 2] .+ numBuses_ac

  ########## 3. Reorder Converter (VSC) Data ##########
  conv_dc_new = copy(conv_dc)
  conv_dc_new[:, 1] .= conv_dc[:, 1] .+ numBuses_ac
  mappingMatrix = hcat(bus_entire_ac[:, 1], bus_entire_ac_new[:, 1], bus_entire_ac[:, end])
  conv_new_col = Vector{eltype(bus_entire_ac_new)}(undef, size(conv_dc, 1))
  for i in 1:size(conv_dc, 1)
      rowVec = conv_dc[i, 2:3]  
      idx = find_row_index(rowVec, mappingMatrix[:, [1, 3]])
      conv_new_col[i] = mappingMatrix[idx, 2]
  end
  conv_dc_new[:, 2] .= conv_new_col

  ########## 4. Reorder Generator Data ##########
  mappingMatrix = hcat(bus_entire_ac[:, 1], bus_entire_ac_new[:, 1], bus_entire_ac[:, end])
  gen_new_col = similar(gen_entire_ac[:, 1])
  for i in 1:size(gen_entire_ac, 1)
      rowVec = gen_entire_ac[i, [1, end]]
      idx = find_row_index(rowVec, mappingMatrix[:, [1, 3]])
      gen_new_col[i] = mappingMatrix[idx, 2]
  end
  gen_entire_ac_new = copy(gen_entire_ac)
  gen_entire_ac_new[:, 1] .= gen_new_col

  gen_entire_ac_new[:, 2] .= vcat(pgen_ac_k...) .* baseMVA_ac
  gen_entire_ac_new[:, 3] .= vcat(qgen_ac_k...) .* baseMVA_ac

  ########## 5. Construct Network Graph ##########
  # Define fromNode and toNode 
  fromNode = vcat(branch_entire_ac_new[:, 1],
                  branch_dc_new[:, 1],
                  conv_dc_new[:, 1])
  toNode = vcat(branch_entire_ac_new[:, 2],
                branch_dc_new[:, 2],
                conv_dc_new[:, 2])
  allNodes = vcat(bus_entire_ac_new[:, 1], bus_dc_new[:, 1])
  numNodes_total = length(allNodes)
  acNodes = bus_entire_ac_new[:, 1]
  dcNodes = bus_dc_new[:, 1]
  genNodes = gen_entire_ac_new[:, 1]
  
  node2Index = Dict(node => i for (i, node) in enumerate(allNodes))
  Random.seed!(1234)
  gNet = Graph(numNodes_total)
  for (f, t) in zip(fromNode, toNode)
      add_edge!(gNet, node2Index[f], node2Index[t])
  end

  # Add Node Voltage Text
  # Compute voltage magnitudes (square root of squared values)
  voltMag_ac = sqrt.(reshape(vcat(vn2_ac_k...), :, 1))
  voltMag_dc = sqrt.(vn2_dc_k)
  voltVal = Vector{Float64}(undef, numNodes_total)
  for i in 1:numNodes_total
      if allNodes[i] in acNodes
          idx_ac = findfirst(x -> x == allNodes[i], bus_entire_ac_new[:, 1])
          voltVal[i] = voltMag_ac[idx_ac]
      elseif allNodes[i] in dcNodes
          idx_dc = findfirst(x -> x == allNodes[i], bus_dc_new[:, 1])
          voltVal[i] = voltMag_dc[idx_dc]
      else 
          voltVal[i] = NaN
      end
  end
  voltStr = [ isnan(x) ? "NaN" : @sprintf("%.3f", x) for x in voltVal ]
  
  # Get node coordinates 
  xyList = spring_layout(gNet, C=2)
  xy = Dict(i => (xyList[1][i], xyList[2][i]) for i in 1:nv(gNet))
  
  fig = Figure()
  ax = Axis(fig[1, 1];
      aspect = DataAspect(),
      xticksvisible = false,
      yticksvisible = false,
      xgridvisible = false,
      ygridvisible = false,
      xticklabelsvisible = false,
      yticklabelsvisible = false,
      spinewidth = 0,
      title = "AC/DC OPF Results"
  )
  
  acBranches = branch_entire_ac_new[:, 1:2]
  dcBranches = branch_dc_new[:, 1:2]
  convBranches = conv_dc_new[:, 1:2]
  
  edgePower = Vector{Float64}(undef, length(edges(gNet)))
  for (edgeIndex, e) in enumerate(edges(gNet))
      fNode = e.src
      tNode = e.dst
      x1, y1 = xy[fNode]
      x2, y2 = xy[tNode]
      
      # Determine branch type and power flow on branches
      isACEdge = any(row -> (row[1] == fNode && row[2] == tNode) || (row[2] == fNode && row[1] == tNode), eachrow(acBranches))
      isDCEdge = any(row -> (row[1] == fNode && row[2] == tNode) || (row[2] == fNode && row[1] == tNode), eachrow(dcBranches))
      isConvEdge = any(row -> (row[1] == fNode && row[2] == tNode) || (row[2] == fNode && row[1] == tNode), eachrow(convBranches))
      
      if isACEdge 
          r = findfirst(row -> (row[1] == fNode && row[2] == tNode) || (row[2] == fNode && row[1] == tNode), eachrow(acBranches))
          i_val = Int(branch_entire_ac[r, 1])
          j_val = Int(branch_entire_ac[r, 2])
          ng = Int(branch_entire_ac[r, end])   # Ensure the last column contains the grid index
          edgePower[edgeIndex] = sqrt(pij_ac_k[ng][i_val, j_val]^2 + qij_ac_k[ng][i_val, j_val]^2) * baseMVA_ac
      elseif isDCEdge
          r = findfirst(row -> (row[1] == fNode && row[2] == tNode) || (row[2] == fNode && row[1] == tNode), eachrow(dcBranches))
          i_val = Int(branch_dc[r, 1])
          j_val = Int(branch_dc[r, 2])
          edgePower[edgeIndex] = pij_dc_k[i_val, j_val] * baseMW_dc * pol_dc
      elseif isConvEdge
          r = findfirst(row -> (row[1] == fNode && row[2] == tNode) || (row[2] == fNode && row[1] == tNode), eachrow(convBranches))
          i_val = Int(conv_dc[r, 1])
          edgePower[edgeIndex] = sqrt(ps_dc_k[i_val]^2 + qs_dc_k[i_val]^2) * baseMVA_ac
      else
          edgePower[edgeIndex] = NaN
      end
  end

  # Determine edge color based on Power magnitude.
  minEdgePower = minimum(edgePower)
  maxEdgePower = maximum(edgePower)
  norm_val(v) = (v - minEdgePower) / (maxEdgePower - minEdgePower)
  edgeColors = [ get(ColorSchemes.Oranges, norm_val(v)) for v in edgePower ]
  
  for (edgeIndex, e) in enumerate(edges(gNet))
      fNode = e.src
      tNode = e.dst
      x1, y1 = xy[fNode]
      x2, y2 = xy[tNode]
      lines!(ax, [x1, x2], [y1, y2]; color = edgeColors[edgeIndex])
  end

  cb = Colorbar(fig[1,2], colormap=ColorSchemes.Oranges, limits=(minEdgePower, maxEdgePower),
                label = "MVA(MW)", width = 30, height = Relative(0.8))
  
  # Define node markers and sizes
  for (i, (x, y)) in xy
      if i in acNodes
          if i in genNodes
              idx_ac = findfirst(x -> x == i, bus_entire_ac_new[:, 1])
              loadPower = sqrt(bus_entire_ac_new[idx_ac, 3]^2 + bus_entire_ac_new[idx_ac, 4]^2)
              idx_gen = findfirst(x -> x == i, gen_entire_ac_new[:, 1])
              genPower = sqrt(gen_entire_ac_new[idx_gen, 2]^2 + gen_entire_ac_new[idx_gen, 3]^2)
              if loadPower >= genPower   # draw load first then generator
                  powerSize = 1e-3 + loadPower * 0.2
                  G.scatter!(ax, [x], [y]; color=:red, markersize=powerSize)
                  powerSize = 1e-3 + genPower * 0.2
                  G.scatter!(ax, [x], [y]; color=:green, markersize=powerSize) 
              else                      # draw generator first then load
                  powerSize = 1e-3 + genPower * 0.2
                  G.scatter!(ax, [x], [y]; color=:green, markersize=powerSize)
                  powerSize = 1e-3 + loadPower * 0.2
                  G.scatter!(ax, [x], [y]; color=:red, markersize=powerSize)
              end
          else
              idx_ac = findfirst(x -> x == i, bus_entire_ac_new[:, 1])
              loadPower = sqrt(bus_entire_ac_new[idx_ac, 3]^2 + bus_entire_ac_new[idx_ac, 4]^2)
              powerSize = 1e-3 + loadPower * 0.2
              G.scatter!(ax, [x], [y]; color=:red, markersize=powerSize)
          end
      else 
          G.scatter!(ax, [x], [y]; color=:gray, marker=:utriangle, markersize=16)
      end
  end
  
  # Add node ID and voltage text
  for (i, (x, y)) in xy
      if i in acNodes
          row = findfirst(x -> x == i, bus_entire_ac_new[:, 1])
          id = bus_entire_ac[row, 1]
          G.text!(ax, "# $(Int(id))", position=(x, y), align=(:center, :center), color=:black, fontsize=12)
          G.text!(ax, voltStr[i] * " p.u.", position=(x+0.02, y+0.02), color=RGB(0.647, 0.649, 0.647), fontsize=11)
      elseif i in dcNodes
          row = findfirst(x -> x == i, bus_dc_new[:, 1])
          id = bus_dc[row, 1]
          G.text!(ax, "# $(Int(id))", position=(x, y), align=(:center, :center), color=:black, fontsize=12)
          G.text!(ax, voltStr[i] * " p.u.", position=(x+0.02, y+0.02), color=RGB(0.647, 0.649, 0.647), fontsize=11)
      end   
  end
  
  # Create legend
  legendACLoad = G.scatter!(ax, [-9999.0], [-9999.0];
      color=:red, marker=:circle, markersize=15, label="AC Loads")
  legendACGen  = G.scatter!(ax, [-9999.0], [-9999.0];
      color=:green, marker=:circle, markersize=15, label="AC Generators")
  legendDCNode = G.scatter!(ax, [-9999.0], [-9999.0];
      color=:gray, marker=:utriangle, markersize=16, label="DC Nodes")
  legendBranch = G.lines!(ax, [-9999.0, -9999.0], [-9999.0, -9999.0];
      color=:brown, linewidth=2, label="Branch Lines")
  
  G.axislegend(ax; position = :rb)
  G.xlims!(ax, -1.5, 1.5)
  G.ylims!(ax, -1.5, 1.5)
  
  return fig
end



