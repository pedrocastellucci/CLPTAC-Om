
module Utils

export readTimedThpack, readBrThpack, readNetScheduling, readConfig

# Manually handling some deprecations from v0.6 to 0.7:
function contains(haystack, needle)
    occursin(needle, haystack)
end

function readConfig(param)
    readConfig("environment.cfg", param)
end


function readConfig(filename, param)
    open(filename) do fd
        line = readline(fd)
        if contains(line, param)
            _, env = split(line, "=")
            return strip(env)
        end
    end
end

function readNetScheduling(filename)
    udcs, cds, consumers, nBoxes = 0, 0, 0, 0
    points = []
    boxesFromUDC = Dict{Int, Any}()
    inbDoors, outDoors = 0, 0
    boxesForConsumers = Dict{Int, Any}()
    consumersTW = Dict{Int, Any}()
    boxesDims = Dict{Int, Any}()
    truckDims = Dict{Int, Any}()

    open(filename) do fd
        while !eof(fd)
            line = readline(fd)
            if contains(line, "UDCs:")
                udcs = split(line)[end]
                udcs = parse(Int, udcs)
            elseif contains(line, "CDs:")
                cds = split(line, ':')[end]
                cds = parse(Int, cds)
            elseif contains(line, "Consumers:")
                consumers = split(line, ':')[end]
                consumers = parse(Int, consumers)
            elseif contains(line, "Boxes:")
                nBoxes = split(line, ':')[end]
                nBoxes = parse(Int, nBoxes)
            elseif contains(line, "NODE_COORD_SECTION:")
                line = readline(fd)  # Reading first coordinates
                i = 0
                while i < udcs + cds + consumers
                    id, coords = split(line, ':')

                    a, b = map(x -> parse(Int, x), split(coords))
                    push!(points, [a, b])

                    i += 1
                    line = readline(fd)
                end
            elseif contains(line, "UDC_SECTION:")
                line = readline(fd)
                i = 0
                while i < udcs
                    id, boxes = split(line, ':')
                    boxes = map(x -> parse(Int, x), split(boxes))
                    boxesFromUDC[parse(Int, id)] = boxes
                    i += 1
                    line = readline(fd)
                end
            elseif contains(line, "CROSSDOCK_SECTION:")
                line = readline(fd)
                i = 0
                while i < cds
                    id, doors = split(line, ':')
                    inbDoors, outDoors = map(x -> parse(Int, x), split(doors))

                    line = readline(fd)
                    i += 1
                end
            elseif contains(line, "CONSUMER_SECTION:")
                line = readline(fd)
                i = 0
                while i < consumers
                    id, boxes = split(line, ':')
                    boxes = map(x -> parse(Int, x), split(boxes))
                    boxesForConsumers[parse(Int, id)] = boxes
                    line = readline(fd)
                    i += 1
                end
            elseif contains(line, "TIME_WINDOWS_SECTION:")
                line = readline(fd)
                i = 0
                while i < consumers
                    id, tw = split(line, ':')
                    lb, ub = map(x -> parse(Float64, x), split(tw))
                    consumersTW[parse(Int, id)] = lb, ub
                    line = readline(fd)
                    i += 1
                end
            elseif contains(line, "BOXES_SECTION:")
                line = readline(fd)
                i = 0
                while i < nBoxes
                    id, dims = split(line, ':')
                    dims = map(x -> parse(Int, x), split(dims))
                    boxesDims[parse(Int, id)] = dims
                    line = readline(fd)
                    i += 1
                end
            elseif contains(line, "CONTAINERS_SECTION:")
                line = readline(fd)
                id, dims = split(line, ':')
                dims = map(x -> parse(Int, x), split(dims))
                truckDims[parse(Int, id)] = dims
            end

            #line = readline(fd)
        end
    end

    @assert(length(points) == udcs + cds + consumers)
    # println("$udcs, $cds, $consumers\n")
    # println("$points\n")
    # println("$boxesFromUDC\n")
    # println("$inbDoors, $outDoors\n")
    # println("$boxesForConsumers\n")

    toReturn = [udcs, cds, consumers, points, boxesFromUDC]
    push!(toReturn, boxesForConsumers, nBoxes)
    push!(toReturn, boxesDims, consumersTW, truckDims)
    # println(sort(collect(keys(boxesDims))))
    toReturn
end

function readTimedThpack(filename)
    boxes = Dict{Int, Any}()
    vehicles = Dict{Int, Any}()
    vIdx = 1
    vMaxX, vMaxY, vMaxZ = 0, 0, 0

    open(filename) do fd
        for (bId, line) in enumerate(eachline(fd))
            words = split(line)

            if length(words) == 3  # It is a vehicle
                a, b, c = split(line)
                a, b, c = map(x -> parse(Int, x), [a, b, c])
                vehicles[vIdx] = a, b, c
                vIdx += 1

                vMaxX = max(vMaxX, a)
                vMaxY = max(vMaxY, b)
                vMaxZ = max(vMaxZ, c)
            else
                a, b, c, t = split(line)
                a, b, c, t = map(x -> parse(Int, x), [a, b, c, t])

                # We add only the boxes that fit:
                if a <= vMaxX && b <= vMaxY && c <= vMaxZ
                    boxes[bId] = (a, b, c, t)
                end
            end
        end
    end
    boxes, vehicles
end


function readBrThpack(filename, instId)
    boxes = Dict{Int, Any}()
    boxesRotations = Dict{Int, Any}()

    vehicles = Dict{Int, Any}()

    lines = 0
    open(filename) do fd
        lines = readlines(fd)
    end

    lIter = 2  # We are skipping the first line
    while lIter <= length(lines)
        a = split(lines[lIter])[1]
        a = parse(Int, a)
        #println("lIter = $(lIter), a = $(a), lines[lIter] = $(lines[lIter])")
        if a == instId
            lIter += 1

            l, w, h = [parse(Int, strip(word)) for word in split(lines[lIter])]
            vehicles[1] = l, w, h

            lIter += 1
            boxTypes = parse(Int, split(lines[lIter])[1])

            lIter += 1
            bId = 1
            #println("Boxtypes $(boxTypes), lIter $(lIter)")
            for i in collect(lIter: lIter + boxTypes - 1)
                #print("i = $(i): ")
                #println(lines[i])
                _, l, rl, w, rw, h, rh, n = split(lines[i])
                l, w, h, n = [parse(Int, s) for s in [l, w, h, n]]
                rl, rw, rh = [parse(Int, s) for s in [rl, rw, rh]]

                # Making sure boxes fit in the container:

                if l <= vehicles[1][1] && w <= vehicles[1][2] && h <= vehicles[1][3] || true  # We do not need this for the bin packing version
                    for bi in collect(1:n)
                        boxes[bId] = (l, w, h, 1)
                        boxesRotations[bId] = (rl, rw, rh)
                        bId += 1
                    end
                end
            end
            # We do not need to read anymore line
            break
        else
            lIter += 2  # Skipping container line
            toSkip = parse(Int, split(lines[lIter])[1])
            lIter += toSkip
        end
        lIter += 1
    end

    boxes, boxesRotations, vehicles
end

end
