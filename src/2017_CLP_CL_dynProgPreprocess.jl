push!(LOAD_PATH, pwd())

using Utils
using JuMP
using Gurobi

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
end


function addXyzVariables(model, boxes, vehicles, boxesPositions)

    xStart = Dict{Int, Float64}()
    yStart = Dict{Int, Float64}()
    zStart = Dict{Int, Float64}()

    if boxesPositions != false
        for (i, values) in boxesPositions
            xStart[i] = values[1]
            yStart[i] = values[2]
            zStart[i] = values[3]
        end
    end


    maxX, maxY, maxZ = 0, 0, 0
    for (x, y, z) in values(vehicles)
        maxX = max(maxX, x)
        maxY = max(maxY, y)
        maxZ = max(maxZ, z)
    end

    @variable(model,
        x[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxX - boxes[b][1],
        start=get(xStart, b, 0)
        )

    @variable(model,
        y[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxY - boxes[b][2],
        start=get(yStart, b, 0)
        )

    @variable(model,
        z[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxZ - boxes[b][3],
        start=get(zStart, b, 0)
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


function addPikVariables(model, boxes, vehicles, boxesPositions)

    assert(length(vehicles) == 1)
    pikStart = Dict{Any, Int}()

    if boxesPositions != false
        for k in keys(vehicles)
            for b in keys(boxes)
                if b in keys(boxesPositions)
                    pikStart[b, k] = 1
                else
                    pikStart[b, k] = 0
                end
            end
        end
    end

    boxVehicle = Dict{Any, Any}()

    @variable(model, p[b=keys(boxes), k=keys(vehicles)],
        Bin, start=get(pikStart, (b, k), 0))

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


function getPreviousSolution(previousFile)

    boxesPositions = Dict{Int, Any}()

    open(previousFile) do fd
        for line in eachline(fd)
            words = split(line)

            if length(words) == 8
                id = parse(Int, words[1])
                a = parse(Int, words[2])
                b = parse(Int, words[3])
                c = parse(Int, words[4])
                d = parse(Int, words[5])
                e = parse(Int, words[6])
                f = parse(Int, words[7])
                boxesPositions[id] = a, b, c, d, e, f
            end
        end
    end

    boxesPositions

end


function solveCLPwithBoxes(boxes, vehicles, outputFile, timeLimit, previousFile)

    boxesPositions = false
    if isfile(previousFile)
        boxesPositions = getPreviousSolution(previousFile)
        println(previousFile, " ", boxesPositions)
    end

    model = Model(solver=GurobiSolver(TimeLimit=timeLimit, OutputFlag=1, Threads=4))  # Time limit in seconds

    ##################### Variables section #####################

    model, x, y, z, Lmax, Wmax, Hmax = addXyzVariables(model, boxes, vehicles, boxesPositions)

    model, a, b, c, d, e, f = addRelativePositionVariables(model, boxes,
                                                        vehicles)

    model, p = addPikVariables(model, boxes, vehicles, boxesPositions)

    #################### Objective function ###################

    model = setObjKnapsack(model, boxes, vehicles, p)

    ##################### Constraints section #####################

    # Each box can be in one container at most:
    for bi in keys(boxes)
        @constraint(model, sum{p[bi, k], k=keys(vehicles)} <= 1)
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
            @constraint(model, x[bi] <= vDims[1] - bDims[1] + (1 - p[bi, k])*(Lmax - bDims[1]))
            @constraint(model, y[bi] <= vDims[2] - bDims[2] + (1 - p[bi, k])*(Wmax - bDims[2]))
            @constraint(model, z[bi] <= vDims[3] - bDims[3] + (1 - p[bi, k])*(Hmax - bDims[3]))
        end
    end

    solve(model)
    logInfo(outputFile, model, boxes, vehicles, p, x, y, z, true)

    getsolvetime(model)

end


function main(radix, instId)

    filename = @sprintf("%s%d", radix, instId)
    T = 1:10
    totalTime = 3600

    boxes, vehicles = readTimedThpack(filename)

    Bt = Dict{Int, Any}()
    for t in T
        auxSet = Set()
        for (bId, (_, _, _, bt)) in boxes
            if bt <= t
                push!(auxSet, bId)
            end
        end
        Bt[t] = auxSet
    end

    auxBoxes = Dict{Int, Any}()
	usedTime = 0

    for t in T
        for bi in Bt[t]
            auxBoxes[bi] = boxes[bi]
        end
        outputFilename = @sprintf("CLP_preprocessFair.%s.%d", instId, t)
        previousFile = @sprintf("CLP_preprocessFair.%s.%d", instId, t-1)

        timeToSolve = (totalTime - usedTime)/(T[end] - t + 1)

        solveTime = solveCLPwithBoxes(
      		auxBoxes, vehicles, outputFilename,
            timeToSolve, previousFile)

		usedTime += solveTime
    end
end

radix = "../datasets/thpack9.1."

lb = parse(Int, ARGS[1])
ub = parse(Int, ARGS[2])
for i in lb:ub
    main(radix, i)
end
