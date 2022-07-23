using Distances
using Concorde
include("DPS.jl")
include("tsp_ep_all.jl")
include("utils.jl")
include("tspd_utils.jl")

checkTestInstancesAgatz("uniform", 1, 100, 1, 10, 4)