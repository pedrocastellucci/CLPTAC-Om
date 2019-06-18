module Heuristics

export getBinPackingBottomLeft
export getDualBoundBinPacking
export primalKnapsackHeuristic
export getDualBoundKnapsack

function checkFreespaceSize(fs, xLB, xUB, yLB, yUB, zLB, zUB)
    "We check if freespace fs is enough to contain the
    minimum box (xLB, yLB, zLB) and is totally inside
    the container (xUB, yUB, zUB)."

    ok = false
    dx, dy, dz = [fs[i+3] - fs[i] for i in collect(1:3)]
    ok = dx >= xLB && dy >= yLB && dz >= zLB

    ok
end


function pack1Box(freespaces, box, xLim, yLim, zLim, boxMinX, boxMinY, boxMinZ)

    # There will be the returns:
    boxFits = false
    boxPosition = (-1, -1, -1)

    toBeUsedSpaceIdx = 0

    # Computing free space in every dimension:
    for (j, fs_loop) in enumerate(freespaces)
        dx, dy, dz = [fs_loop[i+3] - fs_loop[i] for i in collect(1:3)]

        if box[1] <= dx && box[2] <= dy && box[3] <= dz
            # It fits!
            # println("Box $(box) fits on $(fs)")
            boxFits = true
            toBeUsedSpaceIdx = j
            boxPosition = (fs_loop[1], fs_loop[2], fs_loop[3])
            break
        end
    end

    if boxFits
        fs = freespaces[toBeUsedSpaceIdx]

        # Deleting j-th freespace from the lbHeuristic:
        deleteat!(freespaces, toBeUsedSpaceIdx)

        # There is one new freespace above the box:
        newFs1 = (fs[1], fs[2], fs[3] + box[3],
                fs[1] + box[1], fs[2] + box[2], fs[6])

        # Now, we have two options, I'm just generating one
        # newFs2 and newFs3 must NOT overlap:
        newFs2 = (fs[1] + box[1], fs[2], fs[3],
                fs[4], fs[5], fs[6])

        newFs3 = (fs[1], fs[2] + box[2], fs[3],
                fs[1] + box[1], fs[5], fs[6])

        # We want to ensure these new spaces can contain a box:
        ok = checkFreespaceSize(newFs1, boxMinX, fs[4],
                            boxMinY, fs[5], boxMinZ, fs[6])

        if ok  # And we check if we can make a fusion of the new freespaces
            push!(freespaces, newFs1)
        end

        ok = checkFreespaceSize(newFs2, boxMinX, fs[4],
                            boxMinY, fs[5], boxMinZ, fs[6])

        if ok
            push!(freespaces, newFs2)
        end

        ok = checkFreespaceSize(newFs3, boxMinX, fs[4],
                            boxMinY, fs[5], boxMinZ, fs[6])

        if ok
            push!(freespaces, newFs3)
        end
    end

    boxFits, boxPosition, freespaces
end


function primalKnapsackHeuristic(boxes, vehiclesDim, timed, rotation)
    "This heuristic is *based on* George, J. A., and D. F. Robinson.
     “A Heuristic for Packing Boxes into a Container.”
      Computers and Operations Research 7.3 (1980): 147–156. Web."

    # We are assuming are vehicles are the same:
    xLim, yLim, zLim = vehiclesDim

    xLbBox, yLbBox, zLbBox = Inf, Inf, Inf

    # We need bound on boxes dimensions:
    for bDims in values(boxes)
        bx, by, bz = bDims[1], bDims[2], bDims[3]
        if bx < xLbBox
            xLbBox = bx
        end
        if by < yLbBox
            yLbBox = by
        end
        if bz < zLbBox
            zLbBox = bz
        end
    end

     # Just one freespace at the beginning
    freespaces = [(0, 0, 0, xLim, yLim, zLim)]

    # We will separate packed and unpacked boxes:
    packedBoxes = Dict{Int, Any}()
    unpackedBoxes = Dict{Int, Any}()

    sortedBoxes = collect(zip(keys(boxes), values(boxes)))
    if timed
        sort!(sortedBoxes, by=x -> x[2][4], alg=MergeSort)
    end

    # We sort the boxes from largest to smallest
    sort!(sortedBoxes, by=x -> x[2][1]*x[2][2]*x[2][3], alg=MergeSort, rev=true)

    for (bId, bDims) in sortedBoxes
        bx, by, bz = bDims[1], bDims[2], bDims[3]

        rotations = 1:1
        if rotation == true
            rotations = 1:6
        end

		ok = false
		for r in rotations  # We try different rotations for each box
			if r == 1
				bx, by, bz = bx, by, bz  # The original rotation
			elseif r == 2
				bx, by, bz = by, bx, bz
			elseif r == 3
				bx, by, bz = by, bz, bx
			elseif r == 4
				bx, by, bz = bz, bx, by
			elseif r == 5
				bx, by, bz = bz, by, bx
			elseif r == 6
				bx, by, bz = bx, bz, by
			end

	        # Let us try to pack a box:
	        ok, pos, freespaces = pack1Box(freespaces, (bx, by, bz),
	                        xLim, yLim, zLim,
	                        xLbBox, yLbBox, zLbBox)

			if ok
	            # Every time a box is packed we need to resort the freespaces
	            sort!(freespaces, by=x -> x[3])
	            sort!(freespaces, by=x -> x[2])
	            sort!(freespaces, by=x -> x[1])
	            packedBoxes[bId] = pos
				break
	        end
		end
		if ok == false
			unpackedBoxes[bId] = boxes[bId]
		end
    end

    packedBoxes, unpackedBoxes
end

function solveKnapsack(items, capacity)
  """
  Solve a knapsack problem with items and capacity.
  """
    sort!(items, rev=true)
    M = zeros(Int, length(items), capacity+1)

    for (i, w) in enumerate(items)
        for cIdx in collect(1:capacity+1)
            c = cIdx - 1
            if w <= c
                if i <= 1  # Smallest item
                    M[i, cIdx] = w + M[i, cIdx-w]
                else
                    M[i, cIdx] = max(w + M[i, cIdx-w], M[i-1, cIdx])
                end

            end
        end
    end
    M
end


function getBinPackingBottomLeft(boxes, vehiclesDim, timed, rotation)

    # We will assume all vehicles have the same dimension:

    vehiclesCount = 0
    boxesToPack = length(boxes)

    unpacked = boxes
    packed = Dict{Any, Any}()
    i = 0
    while i < boxesToPack

        packedAux, unpacked = primalKnapsackHeuristic(unpacked, vehiclesDim, timed, rotation)
        vehiclesCount += 1

        for (a, b) in packedAux
            packed[a] = b, vehiclesCount
            # println("$(b[1]) $(b[2]) $(b[3]) $(boxes[a][1]) $(boxes[a][2]) $(boxes[a][3])")
        end

        i += length(packedAux)  # How many more we could pack?
    end

    packed, vehiclesCount
end


function getDualBoundBinPacking(boxes, vDims, withRotations=false)
    xValues = Set()
    yValues = Set()
    zValues = Set()

    allValues = Set()

    boxesVolume = 0
    for (b, (x, y, z, _)) in boxes
        boxesVolume += x*y*z
        push!(xValues, x)
        push!(yValues, y)
        push!(zValues, z)
        push!(allValues, x)
        push!(allValues, y)
        push!(allValues, z)
    end

    if withRotations == false
        Mx = solveKnapsack(collect(xValues), vDims[1])
        My = solveKnapsack(collect(yValues), vDims[2])
        Mz = solveKnapsack(collect(zValues), vDims[3])
    else
        Mx = solveKnapsack(collect(allValues), vDims[1])
        My = solveKnapsack(collect(allValues), vDims[2])
        Mz = solveKnapsack(collect(allValues), vDims[3])
    end

    maxVol = Mx[end]*My[end]*Mz[end]

    ceil(boxesVolume/maxVol)
end


function getDualBoundKnapsack(boxes, vDims, withRotations=false)
    xValues = Set()
    yValues = Set()
    zValues = Set()
    allValues = Set()

    boxesVolume = 0
    for (b, (x, y, z, _)) in boxes
        boxesVolume += x*y*z
        push!(xValues, x)
        push!(yValues, y)
        push!(zValues, z)
        push!(allValues, x)
        push!(allValues, y)
        push!(allValues, z)
    end

    Mx = solveKnapsack(collect(xValues), vDims[1])
    My = solveKnapsack(collect(yValues), vDims[2])
    Mz = solveKnapsack(collect(zValues), vDims[3])

    if withRotations == false
        Mx = solveKnapsack(collect(xValues), vDims[1])
        My = solveKnapsack(collect(yValues), vDims[2])
        Mz = solveKnapsack(collect(zValues), vDims[3])
    else
        Mx = solveKnapsack(collect(allValues), vDims[1])
        My = solveKnapsack(collect(allValues), vDims[2])
        Mz = solveKnapsack(collect(allValues), vDims[3])
    end

    maxVol = Mx[end]*My[end]*Mz[end]

    maxVol
end

end  # End module

# boxes = Dict{Any, Any}()
#
# for i in collect(1:10)
#     boxes[i] = (1, 2, 3, 9)
# end
#
# vCount = primalBinPacking(boxes, [10, 10, 10])
# println("We used $(vCount) vehicles")
