function Calculate_distance_matrices_Agatz(alpha::Float64, depot::Tuple{Float64, Float64}, Nodes::Vector{Tuple{Float64, Float64}})
    num_of_nodes = length(Nodes)
    D = zeros(num_of_nodes,num_of_nodes)
    T = zeros(num_of_nodes,num_of_nodes)
    Dp = zeros(num_of_nodes)
    Tp = zeros(num_of_nodes)
    for i=1:num_of_nodes
        Dp[i] = euclidean(depot, Nodes[i])/alpha
        Tp[i] = euclidean(depot, Nodes[i])
        for j=1:num_of_nodes
            D[i,j] = euclidean(Nodes[i],Nodes[j])/alpha
            T[i,j] = euclidean(Nodes[i],Nodes[j])
        end
    end


    DD = zeros(num_of_nodes+2,num_of_nodes+2)
    TT = zeros(num_of_nodes+2,num_of_nodes+2)
    DD[2:num_of_nodes+1,2:num_of_nodes+1] = D
    DD[2:num_of_nodes+1,1] = Dp
    DD[1,2:num_of_nodes+1] = Dp
    DD[2:num_of_nodes+1,num_of_nodes+2] = Dp
    DD[num_of_nodes+2,2:num_of_nodes+1] = Dp
    TT[2:num_of_nodes+1,2:num_of_nodes+1] = T
    TT[2:num_of_nodes+1,1] = Tp
    TT[1,2:num_of_nodes+1] = Tp
    TT[2:num_of_nodes+1,num_of_nodes+2] = Tp
    TT[num_of_nodes+2,2:num_of_nodes+1] = Tp
   
    return TT, DD
end 

function read_all_names(distribution::String, alpha::Int, num_nodes::Int)
    dir_name = joinpath(@__DIR__, "TSP-D-Instances-Agatz/$(distribution)")
    data = readdir(dir_name)
    All_file_names = Vector{String}()
    for name in data
        if alpha==2
            S = split(name,"-")
            if length(S)==3
                if parse(Int,split(split(S[3],".")[1],"n")[2]) == num_nodes
                    push!(All_file_names, name)
                end
            end
        else
            S = split(name,"-")
            if length(S)==4
                if parse(Int,split(S[2],"_")[2])==alpha
                    if parse(Int,split(split(S[4],".")[1],"n")[2]) == num_nodes
                        push!(All_file_names, name)
                    end
                end
            end
        end
    end

    return All_file_names
end


function read_data_Agatz(sample_name::String)
    dir_name = split(sample_name,"-")[1]
    filename = joinpath(@__DIR__, "TSP-D-Instances-Agatz/$(dir_name)/$(sample_name)")
    f = open(filename, "r")
    lines = readlines(f)
    alpha = parse(Float64,lines[2])/parse(Float64,lines[4])
    n_nodes = parse(Int64,lines[6])-1
    depot = (parse(Float64,split(lines[8]," ")[1]),parse(Float64,split(lines[8]," ")[2]))
    customers = Vector{Tuple{Float64, Float64}}()
    for i=1:n_nodes
        node = (parse(Float64,split(lines[9+i]," ")[1]),parse(Float64,split(lines[9+i]," ")[2]))
        push!(customers, node)
    end
    T, D = Calculate_distance_matrices_Agatz(alpha, depot, customers)
    return T, D, depot, customers
end


function checkTestInstancesAgatz(distribution::String, alpha::Int, num_nodes::Int, startSample::Int, endSample::Int, n_groups::Int)
    All_file_names = read_all_names(distribution, alpha, num_nodes)
    if length(All_file_names)==0
        println("There are no such file")
        return false
    end
    mean_objs = 0.0
    run_times = 0.0
    for sample_num =1:length(All_file_names)
        if sample_num>=startSample && sample_num<= endSample
            sample_name = All_file_names[sample_num]
            T,D, depot, customers = read_data_Agatz(sample_name)
            n_nodes = length(customers)
            t1 = time()
            a = Function[two_point_move, one_point_move, two_opt_move]
            MAX_DRONE_RANGE = Inf
            MAX_TIME_LIMIT = Inf
            obj , _, _ = divide_partition_search(T, D, n_groups, a, MAX_DRONE_RANGE, MAX_TIME_LIMIT) #, n_groups, Function[two_point_move, one_point_move, two_opt_move], Inf, Inf)
            # worst_obj, best_obj, mean_obj, run_time = run_GA(T,D,h,n_nodes,run_nums)
            run_time = time() - t1
            mean_objs += obj
            run_times += run_time
        end
    end
    println()
    println()
    println("The final result for ", distribution, ", n=" ,num_nodes, " alpha=",alpha)
    println("Objective= ", mean_objs/10,", time=",run_times/10)

    # touch("GA_results.txt")
    # abc = open("GA_results.txt", "w")
    # for i=1:length(best_objs)
    #     write(abc, "best= ", string(best_objs[i])," ,average= ", string(mean_objs[i]), " , run_time= ", string(run_times[i]))
    # end
    # close(abc)
    # return mean(mean_objs), mean(best_objs), mean(run_times)
end