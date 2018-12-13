using Combinatorics

mutable struct BoxSetManipulator
    maxSize::Int64
    boxes::Int64
    BoxSetManipulator() = new(0, 0)
end

function cleanBoxes!(bm::BoxSetManipulator)
    bm.boxes = 0
end


function getBoxSets(bm::BoxSetManipulator)
    getBoxSets(bm.boxes, bm.maxSize)
end


function getBoxSets(codedBoxes::Int64, maxSize::Int64)
    mask = 1
    count = 1
    boxesList = []

    while count <= maxSize
        if mask & codedBoxes == mask
            push!(boxesList, count)
        end

        mask = mask << 1
        count += 1
    end

    boxesList
end



function addBoxSet!(bm::BoxSetManipulator, setAdd::Int64)
    mask = 1 << (setAdd - 1)
    bm.boxes = bm.boxes | mask
end


function addBoxSetFromList(bm::BoxSetManipulator, setAddList)
    for b in setAddList
        addBoxSet!(bm, b)
    end
    bm
end


function nextStates(bm::BoxSetManipulator, tPeriod)
    # tPeriod is the next time period!

    mask = 0
    i = 0
    while i < tPeriod
        mask = (mask << 1) | 1
        i += 1
    end

    codedMissingBoxes = xor(mask, bm.boxes)
    missingBoxes = getBoxSets(codedMissingBoxes, bm.maxSize)

    partialState = []
    allCombinations = combinations(missingBoxes)

    for comb in allCombinations
        if length(comb) <= length(missingBoxes)
            push!(partialState, comb)
        end
    end

    # println(partialState)
    statesRet = [bm.boxes]
    for ps in partialState
        auxSet = BoxSetManipulator()
        auxSet.maxSize = bm.maxSize
        auxSet.boxes = bm.boxes
        auxSet = addBoxSetFromList(auxSet, ps)
        push!(statesRet, auxSet.boxes)
    end

    collect(Set(statesRet))

end


function getBinomialNextState(bm::BoxSetManipulator, alpha, t)
    # t is the current time!

    idealBoxesNextPeriod = 1:t+1

    ns = nextStates(bm, t+1)
    expectedCost = 0
    nsMapProbability = Dict{Any, Any}()

    for n in ns
        nextStatesList = getBoxSets(n, bm.maxSize)
        countArrived = 0
        countMissing = 0
        for nBox in nextStatesList
            if !(nBox in getBoxSets(bm))
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

        expectedCost += prod1 * prod2
        nsMapProbability[n] = expectedCost
    end

    @assert(abs(expectedCost - 1) <= 0.00001)
    boxProbList = zip(keys(nsMapProbability), values(nsMapProbability))
    boxProbList = collect(boxProbList)
    boxProbList = sort(boxProbList, by=x->x[1], rev=true)
    boxProbList = sort(boxProbList, by=x->x[2])

    pivot = rand()
    while pivot < 0  # We do not want zeros
        pivot = rand()
    end

    for (i, (k, v)) in enumerate(boxProbList)
        if v >= pivot
            return k
        end
    end
end
