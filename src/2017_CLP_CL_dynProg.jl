push!(LOAD_PATH, pwd())

using Utils

function readPreprocessed(instId)

    radix = @sprintf("../results/CLP_CL/CLP_preprocessFair.%d", instId)

    volumeMap = Dict{Int, Float64}()

    for t in 1:10
        filename = @sprintf("%s.%d", radix, t)

        open(filename) do fd
            for line in eachline(fd)
                if contains(line, "Mean volume used per vehicle")
                    vol = split(line, ":")[end]
                    vol = parse(Float64, strip(vol))
                    volumeMap[t] = 1.0 - vol  # We want the unused volume
                    break
                end
            end
        end
    end

    volumeMap
end

function dynProg(instId, mu)
    T = 1:10
    INF = 1000

    volumeMap = readPreprocessed(instId)

    M = zeros(length(T))
    MD = zeros(Int, length(T))

    fill!(M, INF)

    M[T[end]] = 1/length(T) + mu*volumeMap[T[end]]
    MD[T[end]] = 1

    for t in reverse(T[1:end-1])
        M[t] = minimum([1/length(T) + M[t+1], 1/length(T) + mu*volumeMap[t]])
        # println("$t $(M[t+1]) $(volumeMap[t])")
        if 1/length(T) + M[t+1] >= 1/length(T) + mu*volumeMap[t]
            MD[t] = 1  # We stop
        end
    end

    M, MD, volumeMap
end

lb = parse(Int, ARGS[1])
ub = parse(Int, ARGS[2])
mu = parse(Float64, ARGS[3])

outputFile = @sprintf("../results/CLPTAC-dyn-%.1f", mu)

open(outputFile, "w") do fd

    for instId in lb:ub
        M, MD, volumeMap = dynProg(instId, mu)
        # println(MD)

        readyTime = findfirst(MD, 1)
        finalVol = 1 - volumeMap[readyTime]

        toFile = "Inst id: $instId\n"
        toFile *= @sprintf("Volume: %.4f\n", finalVol)
        toFile *= "Ready time: $readyTime\n\n"
        # println(toFile)

        write(fd, toFile)
    end
end
