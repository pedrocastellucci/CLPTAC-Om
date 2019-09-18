function overlapping_detected_message(pos_i, pos_j)
    println("Overlapping detected!")
    println(pos_i)
    println(pos_j)
    exit(0)
end


function box_outside_container_message(pos, container)
    println("Box outside container detected!")
    println(pos)
    println("Container size: ", container)
    exit(0)
end

function get_vehicle_dimensions(line)
    _, values = split(line, "Dimensions")
    values = strip(values, [' ', '(', '.', ')'])
    values = map(x -> parse(Int, x), split(values, ','))
    values
end


function get_box_position(line)
    values = split(line, " ")
    values = [parse(Int, x)
        for x in values
            if length((strip(x, [' ', '\t', 'N', 'A']))) > 0
            ]
    values
end


function is_overlapping_in_a_dimension(bxi, lxi, bxj)
    return bxi + lxi > bxj
end


function is_overlapping(pos_i, pos_j)
    bxi, byi, bzi = pos_i[2:4]
    lxi, lyi, lzi = pos_i[5:7]

    bxj, byj, bzj = pos_j[2:4]
    lxj, lyj, lzj = pos_j[5:7]

    x_separated = true
    if bxi < bxj
        if is_overlapping_in_a_dimension(bxi, lxi, bxj)
            x_separated = false
        end
    elseif is_overlapping_in_a_dimension(bxj, lxj, bxi)
        x_separated = false
    end

    y_separated = true
    if byi < byj
        if is_overlapping_in_a_dimension(byi, lyi, byj)
            y_separated = false
        end
    elseif is_overlapping_in_a_dimension(byj, lyj, byi)
        y_separated = false
    end

    z_separated = true
    if bzi < bzj
        if is_overlapping_in_a_dimension(bzi, lzi, bzj)
            z_separated = false
        end
    elseif is_overlapping_in_a_dimension(bzj, lzj, bzi)
        z_separated = false
    end

    return !(x_separated || y_separated || z_separated)
end


function check_inside(positions, container_size)
    Lx, Ly, Lz = container_size
    for pos in positions
        bx, by, bz, lx, ly, lz = pos[2:7]
        if bx + lx > Lx
            box_outside_container_message((bx, by, bz), container_size)
        end
        if by + ly > Ly
            box_outside_container_message((bx, by, bz), container_size)
        end
        if bz + lz > Lz
            box_outside_container_message((bx, by, bz), container_size)
        end
    end
    return true
end


function check_overlap(positions)
    for (i, pos_i) in enumerate(positions)
        for (j, pos_j) in enumerate(positions)
            if i < j && is_overlapping(pos_i, pos_j)
                overlapping_detected_message(pos_i, pos_j)
            end
        end
    end
    return true
end


function read_outputfile(filename)
    positions = []
    Lx, Ly, Lz = nothing, nothing, nothing
    open(outputfile) do fd
        for line in eachline(fd)
            if occursin("Dimensions", line)
                Lx, Ly, Lz = get_vehicle_dimensions(line)
            elseif length(line) > 0 && isnumeric(line[1])
                pos = get_box_position(line)
                push!(positions, pos)
            end
        end
    end
    return (Lx, Ly, Lz), positions
end


function main(inputfile, outputfile)

    container_size, positions = read_outputfile(outputfile)

    println("Checking if every box is inside the container...")
    if check_inside(positions, container_size)
        println("No issue found!")
    end

    println("Checking for possible overlaps...")
    if check_overlap(positions)
        println("No overlap found!")
    end


end

inputfile = ARGS[1]
outputfile = ARGS[2]
main(inputfile, outputfile)
