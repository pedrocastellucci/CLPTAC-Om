push!(LOAD_PATH, pwd())

using Utils
using JuMP
using Gurobi
using Combinatorics

function logInfo(filename, model, boxes, vehicles, p, x, y, z, printStdout)

    logInfo = ""

    volumeUsedPerVehicle = Dict{Int, Rational{Int}}()

    vehiclesUsed = 1

    for (k, dims) in sort(collect(vehicles), by=x->x[1])

        logInfo *= "\nVehicle $(k). Dimensions ($(dims[1]), $(dims[2]), $(dims[3])).\n"

        volumeUsedPerVehicle[k] = 0
        vehicleVolume = dims[1]*dims[2]*dims[3]
        for (bId, (bx, by, bz, bt)) in sort(collect(boxes), by=x->x[1])
            if getvalue(p[bId, k]) > 0.99

                posX = round(Int, getvalue(x[bId]))
                posY = round(Int, getvalue(y[bId]))
                posZ = round(Int, getvalue(z[bId]))
                logInfo *= "$(bId) \t $((posX)) \t $((posY)) \t"
                logInfo *= "$((posZ)) \t $(bx)\t $(by)\t $(bz)\t NA\n"
                volumeUsedPerVehicle[k] += (bx*by*bz)//vehicleVolume
            end
        end

    end

    logInfo *= "\nVehicles used: $(vehiclesUsed)\n"

    meanVolume = @sprintf("%.4f", float(mean(values(volumeUsedPerVehicle))))
    logInfo *= "\nMean volume used per vehicle: $meanVolume\n"

    logInfo *= "\nNode count: $(Int(getnodecount(model)))\n"

    timeToSolve = @sprintf("%.4f", getsolvetime(model))
    logInfo *= "\nTime to solve: $(timeToSolve)s\n"

    # Getting the gap. I need to compute this explicitly,
    # since getobjgap is not working (for Gurobi)
    primalBound = getobjectivevalue(model)

    dualBound = getobjbound(model)
    gap = abs(primalBound - dualBound)/abs(primalBound)

    gap = @sprintf("%.4f", gap)
    logInfo *= "\nMIPGap: $gap\n"

    if printStdout
        println(logInfo)
    end

    if isfile(filename) == false
        open(filename, "w") do fd
            write(fd, logInfo)
        end
    else
        println("Error! File $filename already exists!")
    end
    logInfo
end


function addXyzVariables(model, boxes, vehicles)

    maxX, maxY, maxZ = 0, 0, 0
    for (x, y, z) in values(vehicles)
        maxX = max(maxX, x)
        maxY = max(maxY, y)
        maxZ = max(maxZ, z)
    end

    @variable(model,
        x[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxX - boxes[b][1]
        )

    @variable(model,
        y[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxY - boxes[b][2]
        )

    @variable(model,
        z[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxZ - boxes[b][3]
        )

    model, x, y, z, maxX, maxY, maxZ
end


function addRelativePositionVariables(model, boxes, vehicles)

    @variable(model, a[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, b[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, c[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, d[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, e[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, f[i=keys(boxes), j=keys(boxes); i < j], Bin)

    model, a, b, c, d, e, f
end


function addPikVariables(model, boxes, vehicles)

    assert(length(vehicles) == 1)

    boxVehicle = Dict{Any, Any}()

    @variable(model, p[b=keys(boxes), k=keys(vehicles)], Bin)

    model, p
end


function setObjKnapsack(model, boxes, vehicles, p)

    boxesVolumes = Dict{Int, Int}()

    for (bId, (bx, by, bz, _)) in boxes
        bVol = bx*by*bz
        boxesVolumes[bId] = bVol
    end

    totalValue = 0
    for dims in values(vehicles)
        totalValue += dims[1]*dims[2]*dims[3]
    end

    @constraint(model, sum(
        boxesVolumes[bi]*p[bi, k] for bi in keys(boxes),
        k in keys(vehicles)) <= totalValue)

    @objective(model, Max, sum(
        boxesVolumes[bi]*p[bi, k] for bi in keys(boxes),
        k in keys(vehicles))/totalValue
        )

    model
end


function solveCLPwithBoxes(boxes, vehicles, outputFile, timeLimit)

    model = Model(solver=GurobiSolver(TimeLimit=timeLimit, OutputFlag=0, Threads=4))  # Time limit in seconds

    ##################### Variables section #####################

    model, x, y, z, Lmax, Wmax, Hmax = addXyzVariables(model, boxes, vehicles)

    model, a, b, c, d, e, f = addRelativePositionVariables(model, boxes,
                                                        vehicles)

    model, p = addPikVariables(model, boxes, vehicles)

    #################### Objective function ###################

    model = setObjKnapsack(model, boxes, vehicles, p)

    ##################### Constraints section #####################

    # Each box can be in one container at most:
    for bi in keys(boxes)
        @constraint(model, sum(p[bi, k] for k in keys(vehicles)) <= 1)
    end

    # Preventing overlap:
    for (bi, (bxi, byi, bzi, _)) in boxes
        for (bj, (bxj, byj, bzj, _)) in boxes
            if bi < bj
                @constraint(model, x[bi] + bxi <= x[bj] + (1 - a[bi, bj])*Lmax)
                @constraint(model, x[bj] + bxj <= x[bi] + (1 - b[bi, bj])*Lmax)
                @constraint(model, y[bi] + byi <= y[bj] + (1 - c[bi, bj])*Wmax)
                @constraint(model, y[bj] + byj <= y[bi] + (1 - d[bi, bj])*Wmax)
                @constraint(model, z[bi] + bzi <= z[bj] + (1 - e[bi, bj])*Hmax)
                @constraint(model, z[bj] + bzj <= z[bi] + (1 - f[bi, bj])*Hmax)
            end
        end
    end

    for k in keys(vehicles)
        for bi in keys(boxes)
            for bj in keys(boxes)
                if bi < bj
                    @constraint(model, a[bi, bj] + b[bi, bj] + c[bi, bj] +
                                    d[bi, bj] + e[bi, bj] + f[bi, bj] >= p[bi, k] + p[bj, k] - 1)
                end
            end
        end
    end

    # Boxes have to be fully inside the container:
    for (k, vDims) in vehicles
        for (bi, bDims) in boxes
            setupperbound(x[bi], vDims[1] - bDims[1])
            setupperbound(y[bi], vDims[2] - bDims[2])
            setupperbound(z[bi], vDims[3] - bDims[3])
        end
    end

    solve(model)
    toPrint = logInfo(outputFile, model, boxes, vehicles, p, x, y, z, true)
    # println(toPrint)
end


function main(radix, instId, resultsFolder)

    filename = @sprintf("%s%d", radix, instId)
    T = 1:10
    timeLimit = 5  # Five second for each problem

    boxes, vehicles = readTimedThpack(filename)

    arrivedShips = combinations(T)
    for ships in arrivedShips
        arrivedBoxes = Dict{Int, Any}()
        for b in boxes
            if b[2][4] in ships
                arrivedBoxes[b[1]] = b[2]
            end
        end

        # Creating the output file name
        outputFile = filename
        for as in ships
            outputFile *= "_" * string(as)
        end
        outputFile *= ".res"
        outputFile = joinpath(resultsFolder, basename(outputFile))
        println("Solving $outputFile")
        # solveCLPwithBoxes(arrivedBoxes, vehicles, outputFile, timeLimit)
    end


end

radix = "../datasets/thpack9.1."
resultsFolder = joinpath("..", "results", "2018_CLPTAC_dynProg_AllStates")


if length(ARGS) > 1
    lb = parse(Int, ARGS[1])
    ub = parse(Int, ARGS[2])
    for i in lb:ub
        main(radix, i, resultsFolder)
    end
else
    main(radix, 1, resultsFolder)
end
