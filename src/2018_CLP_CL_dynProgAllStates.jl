push!(LOAD_PATH, pwd())

using Printf
using Utils
using JuMP
using Gurobi
using Combinatorics
using Statistics

using Heuristics

function printDict(d)
    for i in d
        print(i[1])
        print(" => ")
        println(i[2])
    end
end


function logInfo(filename, model, boxes, vehicles, obj, p, x, y, z,
	 axVar, ayVar, azVar, bxVar, byVar, bzVar, cxVar, cyVar, czVar, printStdout)

    logInfo = ""

    volumeUsedPerVehicle = Dict{Int, Rational{Int}}()

    vehiclesUsed = 1

    for (k, dims) in sort(collect(vehicles), by=x->x[1])

        logInfo *= "\nVehicle $(k). Dimensions ($(dims[1]), $(dims[2]), $(dims[3])).\n"

        volumeUsedPerVehicle[k] = 0
        vehicleVolume = dims[1]*dims[2]*dims[3]
        for (bid, (bli, bwi, bhi, _)) in sort(collect(boxes), by=x->x[1])
            if getvalue(p[bid, k]) > 0.99
                ax, ay, az = getvalue(axVar[bid]), getvalue(ayVar[bid]), getvalue(azVar[bid])
				bx, by, bz = getvalue(bxVar[bid]), getvalue(byVar[bid]), getvalue(bzVar[bid])
				cx, cy, cz = getvalue(cxVar[bid]), getvalue(cyVar[bid]), getvalue(czVar[bid])

                posX = round(Int, getvalue(x[bid]))
                posY = round(Int, getvalue(y[bid]))
                posZ = round(Int, getvalue(z[bid]))

				boxX = bli*ax + bwi*bx + bhi*cx
				boxY = bwi*by + bli*ay + bhi*cy
				boxZ = bhi*cz + bwi*bz + bli*az

				boxX = round(Int, boxX)
				boxY = round(Int, boxY)
				boxZ = round(Int, boxZ)
                logInfo *= "$(bid) \t $((posX)) \t $((posY)) \t"
                logInfo *= "$((posZ)) \t $(boxX)\t $(boxY)\t $(boxZ)\t NA\n"
                volumeUsedPerVehicle[k] += (boxX*boxY*boxZ)//vehicleVolume
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
    logInfo *= "\nMIPGap: $gap"

    if printStdout
        println(logInfo)
    end

    if isfile(filename)
        println("Error file $filename already exists!")
        exit(-1)
    else
        open(filename, "w") do fd
            write(fd, logInfo)
        end
    end
end


function addXyzVariables(model, boxes, vehicles, packed)

    maxX, maxY, maxZ = 0, 0, 0
    for (x, y, z) in values(vehicles)
        maxX = max(maxX, x)
        maxY = max(maxY, y)
        maxZ = max(maxZ, z)
    end

    @variable(model,
        x[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxX - (haskey(packed, b) ? minimum(boxes[b]) : 0),
        start=haskey(packed, b) ? packed[b][1] : 0,
        )

    @variable(model,
        y[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxY - (haskey(packed, b) ? minimum(boxes[b]) : 0),
        start=haskey(packed, b) ? packed[b][2] : 0,
        )

    @variable(model,
        z[b=keys(boxes)],
        lowerbound=0,
        upperbound=maxZ - (haskey(packed, b) ? minimum(boxes[b]) : 0),
        start=haskey(packed, b) ? packed[b][3] : 0,
        )

    model, x, y, z, maxX, maxY, maxZ
end


function addRelativePositionVariables(model, boxes, vehicles, packed)

    @variable(model, a[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, b[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, c[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, d[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, e[i=keys(boxes), j=keys(boxes); i < j], Bin)
    @variable(model, f[i=keys(boxes), j=keys(boxes); i < j], Bin)

    model, a, b, c, d, e, f
end


function addPikVariables(model, boxes, vehicles, packed)

    # We are considering just one container!
    @assert(length(vehicles) == 1)
    k = 1

    boxVehicle = Dict{Any, Any}()

    for (bId, (x, y, z, )) in boxes
        if haskey(packed, bId)
            boxVehicle[bId, k] = 1
        else
            boxVehicle[bId, k] = 0
        end
    end

    @variable(model, p[b=keys(boxes), k=keys(vehicles)],
            start=boxVehicle[b, k], Bin)
    model, p
end

function addRotationVariables(model, boxes)

    @variable(model, axi[i=keys(boxes)], Bin)
	@variable(model, ayi[i=keys(boxes)], Bin)
    @variable(model, azi[i=keys(boxes)], Bin)
	@variable(model, bxi[i=keys(boxes)], Bin)
    @variable(model, byi[i=keys(boxes)], Bin)
	@variable(model, bzi[i=keys(boxes)], Bin)
	@variable(model, cxi[i=keys(boxes)], Bin)
	@variable(model, cyi[i=keys(boxes)], Bin)
    @variable(model, czi[i=keys(boxes)], Bin)

	for bi in keys(boxes)
		@constraint(model, axi[bi] + ayi[bi] + azi[bi] == 1)
		@constraint(model, bxi[bi] + byi[bi] + bzi[bi] == 1)
		@constraint(model, cxi[bi] + cyi[bi] + czi[bi] == 1)
		@constraint(model, axi[bi] + bxi[bi] + cxi[bi] == 1)
		@constraint(model, ayi[bi] + byi[bi] + cyi[bi] == 1)
		@constraint(model, azi[bi] + bzi[bi] + czi[bi] == 1)
	end

    model, axi, ayi, azi, bxi, byi, bzi, cxi, cyi, czi

end

function setObjKnapsack(model, boxes, vehicles, p, packed)

    boxesVolumes = Dict{Int, Int}()

    for (bId, (bx, by, bz, _)) in boxes
        bVol = bx*by*bz
        boxesVolumes[bId] = bVol
    end

    totalValue = 0
    for dims in values(vehicles)
        totalValue += dims[1]*dims[2]*dims[3]
    end

    if length(packed) > 0
        usedVol = sum(
            boxes[b][1]*boxes[b][2]*boxes[b][3] for b in keys(packed)
        )
    else
        usedVol = 0
    end

    @variable(model, obj,
        lowerbound=usedVol,
        upperbound=totalValue,
        start=usedVol)

    @constraint(model, obj == sum(
        boxesVolumes[bi]*p[bi, k] for bi in keys(boxes),
        k in keys(vehicles)))

    @objective(model, Max, obj)

    model, obj
end


function solveCLPwithBoxes(boxes, vehicles, outputFile, timeLimit)

    vehicleDim = vehicles[1]

    #packed, _ = primalKnapsackHeuristic(boxes, vehicleDim, false, false)
    packed = Dict{Int64,Any}()

    model = Model(solver=GurobiSolver(TimeLimit=timeLimit, Threads=4, OutputFlag=0))  # Time limit in seconds

    ##################### Variables section #####################

    model, x, y, z, Lmax, Wmax, Hmax = addXyzVariables(model, boxes,
                                                        vehicles, packed)

	model, a, b, c, d, e, f = addRelativePositionVariables(model, boxes,
                                                        vehicles, packed)

    model, ax, ay, az, bx, by, bz, cx, cy, cz = addRotationVariables(model, boxes)

    model, p = addPikVariables(model, boxes, vehicles, packed)

    #################### Objective function ###################
    model, obj = setObjKnapsack(model, boxes, vehicles, p, packed)

    ##################### Constraints section #####################

    # Preventing overlap:
    for (bi, (bli, bwi, bhi, _)) in boxes
        for (bj, (blj, bwj, bhj, _)) in boxes
            if bi < bj
                @constraint(model, x[bi]
                    + bli*ax[bi]
                    + bwi*bx[bi]
                    + bhi*cx[bi]
                    <= x[bj]  + (1 - a[bi, bj])*Lmax
                    )

                @constraint(model, x[bj]
                    + blj*ax[bj]
                    + bwj*bx[bj]
                    + bhj*cx[bj] <=
                    x[bi] + (1 - b[bi, bj])*Lmax
                    )

                @constraint(model, y[bi]
                    + bwi*by[bi]
                    + bli*ay[bi]
                    + bhi*cy[bi]<=
                    y[bj] + (1 - c[bi, bj])*Wmax
                    )

                @constraint(model, y[bj]
                    + bwj*by[bj]
                    + blj*ay[bj]
                    + bhj*cy[bj] <=
                    y[bi] + (1 - d[bi, bj])*Wmax
                    )

                @constraint(model, z[bi]
                    + bhi*cz[bi]
                    + bwi*bz[bi]
                    + bli*az[bi] <=
                    z[bj] + (1 - e[bi, bj])*Hmax
                    )

                @constraint(model, z[bj]
                    + bhj*cz[bj]
                    + bwj*bz[bj]
                    + blj*az[bj] <=
                    z[bi] + (1 - f[bi, bj])*Hmax
                    )
            end
        end
    end

	for k in keys(vehicles), bi in keys(boxes), bj in keys(boxes)
        if bi < bj
            @constraint(model, a[bi, bj] + b[bi, bj] + c[bi, bj] +
                            d[bi, bj] + e[bi, bj] + f[bi, bj] >= p[bi, k] + p[bj, k] - 1)
        end
    end

    # Boxes have to be fully inside the container:
    for (k, vDims) in vehicles
        for (bi, bDims) in boxes
            bMinDim = minimum(bDims)

            @constraint(model, x[bi]
                + bDims[1]*ax[bi]
                + bDims[2]*bx[bi]
                + bDims[3]*cx[bi]
                <= vDims[1])

            @constraint(model, y[bi]
                + bDims[2]*by[bi]
                + bDims[1]*ay[bi]
                + bDims[3]*cy[bi]
                <= vDims[2])

            @constraint(model, z[bi]
                + bDims[3]*cz[bi]
                + bDims[2]*bz[bi]
                + bDims[1]*az[bi] <= vDims[3])
        end
    end

	solve(model)
    logInfo(outputFile, model, boxes, vehicles, obj, p, x, y, z,
		ax, ay, az, bx, by, bz, cx, cy, cz, true)
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
        solveCLPwithBoxes(arrivedBoxes, vehicles, outputFile, timeLimit)
    end
end


for bla in ["3", "4", "5", "6", "7", "8", "9", "10"]
    radix = "../datasets/br/br" * bla * "."
    resultsFolder = joinpath("..", "results", "CLPTAC_dynProg_AllStates_br")

    if length(ARGS) > 1
        lb = parse(Int, ARGS[1])
        ub = parse(Int, ARGS[2])
        for i in lb:ub
            main(radix, i, resultsFolder)
        end
    else
        main(radix, 1, resultsFolder)
    end
end
