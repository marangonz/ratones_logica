using Agents

# Simple A* pathfinding implementation
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
        # Find node with lowest f_score
        current = open_set[1]
        current_idx = 1
        for (i, node) in enumerate(open_set)
            if get(f_score, node, Inf) < get(f_score, current, Inf)
                current = node
                current_idx = i
            end
        end
        
        # Remove current from open_set
        deleteat!(open_set, current_idx)
        
        if current == goal
            # Reconstruct path
            path = [current]
            while haskey(came_from, current)
                current = came_from[current]
                pushfirst!(path, current)
            end
            return path
        end
        
        for neighbor in get_neighbors(current, matrix)
            tentative_g_score = get(g_score, current, Inf) + 1
            
            if tentative_g_score < get(g_score, neighbor, Inf)
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + manhattan_distance(neighbor, goal)
                
                if !(neighbor in open_set)
                    push!(open_set, neighbor)
                end
            end
        end
    end
    
    return [start] # No path found, stay in place
end

matrix = [
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 0;
    0 1 0 1 0 0 0 1 1 1 0 1 0 1 0 1 0;
    0 1 1 1 0 1 0 0 0 0 0 1 0 1 1 1 0;
    0 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 0;
    0 1 0 1 0 1 0 0 0 0 0 1 1 1 0 1 0;
    0 1 1 1 0 1 0 1 1 1 0 1 0 1 0 1 0;
    0 1 0 1 0 1 0 1 1 1 0 1 0 1 0 1 0;
    0 1 0 1 1 1 0 0 1 0 0 1 0 1 1 1 0;
    0 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 0;
    0 1 1 1 0 1 0 0 0 0 0 1 0 1 1 1 0;
    0 1 0 1 0 1 0 1 1 1 0 0 0 1 0 1 0;
    0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
]


function agent_step!(agent, model)
    # Check if agent reached the banana
    if agent.pos == banana_pos
        # Reached the banana! Place it somewhere else
        place_banana_randomly()
        agent.route = [] # Clear route since banana moved
        return
    end
    
    # If no route or route is invalid/empty, calculate new path to banana
    if isempty(agent.route) || (length(agent.route) > 0 && agent.route[1] != agent.pos)
        path = a_star_path(agent.pos, banana_pos, matrix)
        if length(path) > 1
            agent.route = path[2:end] # Exclude current position
        else
            agent.route = [] # No valid path
        end
    end
    
    # Move along the route if available
    if !isempty(agent.route)
        next_pos = agent.route[1]
        agent.route = agent.route[2:end] # Remove the position we're moving to
        move_agent!(agent, next_pos, model)
    end
end

@agent struct Ghost(GridAgent{2})
    route::Vector{Tuple{Int,Int}}
end

# Global variable to store banana position
banana_pos = (0, 0)

function place_banana_randomly()
    # Find all walkable positions (value = 1)
    walkable_positions = []
    for i in 1:size(matrix, 1)
        for j in 1:size(matrix, 2)
            if matrix[i, j] == 1
                push!(walkable_positions, (i, j))
            end
        end
    end
    # Pick a random walkable position
    global banana_pos = rand(walkable_positions)
    return banana_pos
end

function initialize_model()
    space = GridSpace((14,17); periodic = false, metric = :manhattan)
    model = StandardABM(Ghost, space; agent_step!)
    return model
end

model = initialize_model()
a = add_agent!(Ghost, pos=(2, 2), route=Tuple{Int,Int}[], model)
# Place banana randomly on initialization
place_banana_randomly()