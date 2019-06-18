    push!(LOAD_PATH, pwd())

using Random
using Statistics
using Combinatorics
using Utils

Random.seed!(6)

include("2018_CLP_CL_dynPackingBoxSetManipulator.jl")

function getFilenameForCombination(filename, comb, resultFolder)
    combName = filename
    for c in comb
        combName *= "_" * string(c)
    end
    joinpath(resultFolder, combName * ".res")
end


function getVolumeFromFile(filename)
    lines = []
    open(filename) do fd
        lines = readlines(fd)
    end
    volume = 0.0
    for line in lines
        if occursin("Mean volume used per vehicle", line)
            _, volume = split(line, ":")
            volume = parse(Float64, volume)
        end
    end
    volume
end


function getBoxesFromShips(allBoxes, shipments)

    boxesLeft = Dict{Int64, Any}()
    for (bId, dims) in allBoxes
        if dims[4] in shipments
            boxesLeft[bId] = dims
        end
    end

    boxesLeft
end


function dynSolve(T, alpha, mu, filename, resultFolder)

    filename = joinpath("..", "datasets", filename)

    allBoxes, truck = readTimedThpack(filename)
    truckDims = truck[1]

    instName = basename(filename)
    states = Dict{Int, Float64}()
    states[0] = 1.0

    for comb in combinations(T)
        bm = BoxSetManipulator()
        bm.boxes, bm.maxSize = 0, length(T)
        combName = getFilenameForCombination(instName, comb, resultFolder)
        if isfile(combName) == false
            println("Not found $combName")
            continue
        end
        volume = getVolumeFromFile(combName)
        for c in comb
            addBoxSet!(bm, c)
        end
        states[bm.boxes] = 1.0 - volume
        @assert(states[bm.boxes] >= 0.0)
    end

    M = Dict{Any, Float64}()
    MD = Dict{Any, Int}()

    truckVol = truckDims[1]*truckDims[2]*truckDims[3]
    for i in T, j in 0:2^length(T)-1

        if i == T[end]
            # Last period cost is based on the volume
            # of boxes left behind:
            shipsLeft = getBoxSets(j, 10)
            boxesLeft = getBoxesFromShips(allBoxes, shipsLeft)

            volLeft = 0
            for (b, dims) in boxesLeft
                volLeft += dims[1]*dims[2]*dims[3]
            end
            volLeft = volLeft/truckVol
            volLeft = volLeft - (1 - states[j])
            volLeft = ceil(volLeft)*truckVol

            #M[j, i] = 1.0/length(T) + mu*states[j]
            M[j, i] = 1.0/length(T) + volLeft
            MD[j, i] = 0
        else
            M[j, i] = Inf
        end
    end

    auxBoxSet = BoxSetManipulator()
    auxBoxSet.maxSize = 10

    for i in reverse(T[1:end-1])
        idealBoxesNextPeriod = 1:i+1

        for j in 0:2^length(T)-1
            auxBoxSet.boxes = j

            ns = nextStates(auxBoxSet, i+1)
            currBoxList = getBoxSets(auxBoxSet)

            expectedCost = 1.0/float(length(T))
            stopCost = mu*states[j] + 1.0/float(length(T))

            sumProds = 0.0

            for n in ns
                nextStatesList = getBoxSets(n, auxBoxSet.maxSize)
                countArrived = 0
                countMissing = 0

                for nBox in nextStatesList
                    if !(nBox in currBoxList)
                        countArrived += 1
                    end
                end

                for iBox in idealBoxesNextPeriod
                    if !(iBox in nextStatesList)
                        countMissing += 1
                    end
                end

                prod1 = alpha ^ countArrived
                prod2 = (1 - alpha) ^ countMissing

                expectedCost += prod1*prod2*M[n, i+1]
                sumProds += prod1*prod2

            end
            @assert(abs(sumProds - 1) <= 0.00001)

            if expectedCost < stopCost
                M[j, i] = expectedCost
                MD[j, i] = 1
            else
                M[j, i] = stopCost
                MD[j, i] = 0
            end
        end
    end
    M, MD, states
end


function simul(filename, M, MD, mu, alpha, T)

    bm = BoxSetManipulator()
    bm.boxes, bm.maxSize = 0, 10

    ns = getBinomialNextState(bm, alpha, 0)
    bm.boxes = ns
    shipments = [bm.boxes]

    decisions = []
    for t in T
        d = MD[bm.boxes, t]
        push!(decisions, d)
        if d < 0.0001
            return decisions, shipments, M
        end

        ns = getBinomialNextState(bm, alpha, t)
        bm.boxes = ns
        push!(shipments, bm.boxes)
    end
    assert(false)
end


function runSimulations(filename, mu, alpha, repetitions, resultsFolder)
    stopTimes, volumes, objs = [], [], []

    T = 1:10
    M, MD, states = dynSolve(T, alpha, mu, filename, resultsFolder)
    obj = M[0, 1]

    for r in 1:repetitions
        if r % 1000 == 0
            println("Doing repetition $r")
        end
        decisions, shipments, M = simul(filename, M, MD, mu, alpha, T)
        st = length(decisions)
        vol = states[shipments[end]]
        push!(stopTimes, st)
        push!(volumes, vol)
    end

    mean(stopTimes), 1.0 - mean(volumes), obj
end


function experiments(benchFolder, makespanFile, volumeFile, resultsFolder)

    repetitions = 5000
    mus = [1, 2, 4]
    alphas = 0.5:0.1:1.0

    for mu in mus
        outMake = makespanFile * "_$mu"
        outVol = volumeFile * "_$mu"

        makeMatrix = []
        volMatrix = []

        toPrintMake = ""
        toPrintVol = ""
        for filename in readdir(benchFolder)
            lineVol = ""
            lineMake = ""

            # This next line might be different in Windows systems:
            filename = joinpath(benchFolder, filename)

            if occursin("br", filename)
                if filename[end] == "~"
                    continue
                end
                println(filename, " ", "mu = $mu")

                fileId = split(filename, ".")[end]
                lineVol *= "$fileId, "
                lineMake *= "$fileId, "
                for alpha in alphas
                    st, vl, obj = runSimulations(filename, mu, alpha,
                        repetitions, resultsFolder)
                    lineVol *= "$vl, "
                    lineMake *= "$st, "
                end
                lineVol = rstrip(lineVol, [',', ' '])
                lineMake = rstrip(lineMake, [',', ' '])

                lineVol *= "\n"
                lineMake *= "\n"

                toPrintVol *= lineVol
                toPrintMake *= lineMake
            end
        end

        open(outVol, "w") do fd
            write(fd, toPrintVol)
        end
        open(outMake, "w") do fd
            write(fd, toPrintMake)
        end
    end
end


benchFolder = joinpath("..", "datasets", "br")
resultsFolder = joinpath("..", "results", "CLPTAC_dynProg_AllStates_br")

experiments(benchFolder, "make.out", "volume.out", resultsFolder)

# T = 1:10
# alpha = 0.6
# mu = 4.0
# stopTime, volume, obj = runSimulations(filename, mu, alpha, 5000)

# println(stopTime)
# println(volume)
# println(obj)
