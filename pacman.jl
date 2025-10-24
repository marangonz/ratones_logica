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
    # Check if agent reached any banana
    if agent.pos in banana_positions
        # Reached a banana! Remove it and clear claims
        delete!(banana_positions, agent.pos)
        release_banana_claim(agent.pos)
        agent.route = [] # Clear route since we need to find a new target
        agent.target_banana = nothing
        return
    end
    
    # Check if our current target is still valid (not taken by someone else or removed)
    if agent.target_banana !== nothing && 
       (!(agent.target_banana in banana_positions) || 
        (haskey(banana_claims, agent.target_banana) && banana_claims[agent.target_banana] != agent.id))
        # Our target is no longer valid, clear it
        agent.target_banana = nothing
        agent.route = []
    end
    
    # Find a new target if we don't have one
    if agent.target_banana === nothing
        target_banana = find_nearest_unclaimed_banana(agent.pos, agent.id)
        if target_banana !== nothing
            agent.target_banana = target_banana
            claim_banana(target_banana, agent.id)
        else
            # No bananas available, stay in place
            agent.route = []
            return
        end
    end
    
    # If no route or route is invalid/empty, calculate new path to our target
    if isempty(agent.route) || (length(agent.route) > 0 && agent.route[1] != agent.pos)
        path = a_star_path(agent.pos, agent.target_banana, matrix)
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
    target_banana::Union{Tuple{Int,Int}, Nothing}
end

# Global variable to store banana positions (multiple bananas)
banana_positions = Set{Tuple{Int,Int}}()
# Global variable to track which bananas are claimed by which agents
banana_claims = Dict{Tuple{Int,Int}, Int}()  # banana_pos => agent_id

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
    
    # Pick a random walkable position that doesn't already have a banana
    available_positions = filter(pos -> !(pos in banana_positions), walkable_positions)
    if !isempty(available_positions)
        new_banana_pos = rand(available_positions)
        push!(banana_positions, new_banana_pos)
        return new_banana_pos
    end
    return nothing # No available positions
end

function find_nearest_unclaimed_banana(agent_pos, agent_id)
    if isempty(banana_positions)
        return nothing
    end
    
    nearest_banana = nothing
    min_distance = Inf
    
    for banana_pos in banana_positions
        # Skip bananas that are claimed by other agents
        if haskey(banana_claims, banana_pos) && banana_claims[banana_pos] != agent_id
            continue
        end
        
        distance = manhattan_distance(agent_pos, banana_pos)
        if distance < min_distance
            min_distance = distance
            nearest_banana = banana_pos
        end
    end
    
    return nearest_banana
end

function claim_banana(banana_pos, agent_id)
    if banana_pos in banana_positions
        banana_claims[banana_pos] = agent_id
    end
end

function release_banana_claim(banana_pos)
    if haskey(banana_claims, banana_pos)
        delete!(banana_claims, banana_pos)
    end
end

function initialize_model()
    space = GridSpace((14,17); periodic = false, metric = :manhattan)
    model = StandardABM(Ghost, space; agent_step!)
    return model
end

model = initialize_model()
# Add two monkeys at different starting positions
monkey1 = add_agent!(Ghost, pos=(2, 2), route=Tuple{Int,Int}[], target_banana=nothing, model)
monkey2 = add_agent!(Ghost, pos=(12, 15), route=Tuple{Int,Int}[], target_banana=nothing, model)

# Place multiple bananas initially
for i in 1:7  # Start with 7 bananas
    place_banana_randomly()
end