using Agents

# =====================================
# === A* PATHFINDING UTILITIES ========
# =====================================
function manhattan_distance(a, b)
    return abs(a[1] - b[1]) + abs(a[2] - b[2])
end

function get_neighbors(pos, matrix)
    x, y = pos
    neighbors = []
    for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        nx, ny = x + dx, y + dy
        if 1 ≤ nx ≤ size(matrix, 1) && 1 ≤ ny ≤ size(matrix, 2) && matrix[nx, ny] == 1
            push!(neighbors, (nx, ny))
        end
    end
    return neighbors
end

function a_star_path(start, goal, matrix)
    if start == goal
        return [start]
    end
    
    open_set = [start]
    came_from = Dict()
    g_score = Dict(start => 0)
    f_score = Dict(start => manhattan_distance(start, goal))
    
    while !isempty(open_set)
        current = open_set[argmin([get(f_score, n, Inf) for n in open_set])]
        deleteat!(open_set, findfirst(==(current), open_set))
        
        if current == goal
            path = [current]
            while haskey(came_from, current)
                current = came_from[current]
                pushfirst!(path, current)
            end
            return path
        end
        
        for neighbor in get_neighbors(current, matrix)
            tentative_g = get(g_score, current, Inf) + 1
            if tentative_g < get(g_score, neighbor, Inf)
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + manhattan_distance(neighbor, goal)
                if !(neighbor in open_set)
                    push!(open_set, neighbor)
                end
            end
        end
    end
    return [start]
end

matrix = [
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
]

@agent struct Ghost(GridAgent{2})
    route::Vector{Tuple{Int,Int}}
    target_banana::Union{Tuple{Int,Int}, Nothing}
    type::String
    dir::Tuple{Int,Int}
    steps::Int
end

# =====================================
# === BANANAS =========================
# =====================================
banana_positions = Set{Tuple{Int,Int}}()

function place_banana_randomly()
    walkable_positions = [(i, j) for i in 1:size(matrix, 1), j in 1:size(matrix, 2) if matrix[i, j] == 1]
    available = filter(pos -> !(pos in banana_positions), walkable_positions)
    if !isempty(available)
        pos = rand(available)
        push!(banana_positions, pos)
        return pos
    end
    return nothing
end

#a*
function find_best_banana_for_agent(agent, model)
    if isempty(banana_positions)
        return nothing
    end
    
    best_banana = nothing
    best_score = -1
    for banana_pos in banana_positions
        our_distance = manhattan_distance(agent.pos, banana_pos)
        base_score = 100 - our_distance
        competition_penalty = 0
        for other in allagents(model)
            if other.id != agent.id
                d2 = manhattan_distance(other.pos, banana_pos)
                if d2 < our_distance
                    competition_penalty += (our_distance - d2) * 10
                end
            end
        end
        score = base_score - competition_penalty
        if score > best_score
            best_score, best_banana = score, banana_pos
        end
    end
    return best_banana
end

function should_switch_target(agent, model)
    if agent.target_banana === nothing
        return true
    end
    current = agent.target_banana
    dist = manhattan_distance(agent.pos, current)
    closer_agents = count(a -> a.id != agent.id && manhattan_distance(a.pos, current) < dist, allagents(model))
    if closer_agents > 0
        alt = find_best_banana_for_agent(agent, model)
        if alt !== nothing && alt != current && manhattan_distance(agent.pos, alt) < dist - 2
            return true
        end
    end
    return false
end

function a_star_behavior!(agent, model)
    if agent.pos in banana_positions
        delete!(banana_positions, agent.pos)
        agent.route = []
        agent.target_banana = nothing
        return
    end
    if agent.target_banana !== nothing && !(agent.target_banana in banana_positions)
        agent.target_banana = nothing
        agent.route = []
    end
    if agent.target_banana === nothing || should_switch_target(agent, model)
        agent.target_banana = find_best_banana_for_agent(agent, model)
        agent.route = []
    end
    if agent.target_banana !== nothing && (isempty(agent.route) || agent.route[1] != agent.pos)
        path = a_star_path(agent.pos, agent.target_banana, matrix)
        agent.route = length(path) > 1 ? path[2:end] : []
    end
    if !isempty(agent.route)
        next_pos = popfirst!(agent.route)
        move_agent!(agent, next_pos, model)
    end
end

function free_neighbors(pos, matrix)
    y, x = pos
    moves = [(y-1, x), (y+1, x), (y, x-1), (y, x+1)]
    return [(ny, nx) for (ny, nx) in moves if 1 <= ny <= size(matrix, 1) && 1 <= nx <= size(matrix, 2) && matrix[ny, nx] == 1]
end

random_direction() = rand([(-1,0),(1,0),(0,-1),(0,1)])

function random_behavior!(agent, model)
    if agent.pos in banana_positions
        delete!(banana_positions, agent.pos)
        return
    end
    
    if agent.steps < 5
        dy, dx = agent.dir
        new_pos = (agent.pos[1] + dy, agent.pos[2] + dx)
        if 1 <= new_pos[1] <= size(matrix, 1) && 1 <= new_pos[2] <= size(matrix, 2) && matrix[new_pos[1], new_pos[2]] == 1
            move_agent!(agent, new_pos, model)
            agent.steps += 1
            return
        end
    end
    options = free_neighbors(agent.pos, matrix)
    if !isempty(options)
        new_pos = rand(options)
        move_agent!(agent, new_pos, model)
        agent.dir = random_direction()
        agent.steps = 0
    end
end

function agent_step!(agent, model)
    if agent.type == "AStar"
        a_star_behavior!(agent, model)
    else
        random_behavior!(agent, model)
    end
end

function initialize_model()
    space = GridSpace((14,17); periodic=false, metric=:manhattan)
    model = StandardABM(Ghost, space; agent_step!)
    
    add_agent!(Ghost, pos=(2,2), route=[], target_banana=nothing,
               type="AStar", dir=(0,0), steps=0, model)
    
    add_agent!(Ghost, pos=(12,15), route=[], target_banana=nothing,
               type="Random", dir=random_direction(), steps=0, model)
    
    for i in 1:7
        place_banana_randomly()
    end
    return model
end

model = initialize_model()
