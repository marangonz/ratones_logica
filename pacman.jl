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

# =====================================
# === LÓGICA DE PASO PRINCIPAL ========
# =====================================
# Se cambia `type` por `state`
@agent struct Ghost(GridAgent{2})
    state::Symbol  # :wander o :chase
    route::Vector{Tuple{Int,Int}}
    target_banana::Union{Tuple{Int,Int}, Nothing}
    dir::Tuple{Int,Int} # Dirección para el modo wander
    steps::Int      # Pasos en la misma dirección para el modo wander
    powerup_time::Int # Time remaining for speed boost
    slowdown_time::Int # Time remaining for speed penalty
end

@agent struct Gato(GridAgent{2})
    state::Symbol
    route::Vector{Tuple{Int,Int}}
    target_ghost::Union{Int, Nothing}
    dir::Tuple{Int,Int}
    steps::Int
    # Definir propiedades del gato si es necesario
end

const vision_range = 6  # Increased vision for more aggressive cat - ORIGINALMENTE 5

# =====================================
# === BANANAS =========================
# =====================================

banana_positions = Set{Tuple{Int,Int}}()
magic_banana_positions = Set{Tuple{Int,Int}}()
green_banana_positions = Set{Tuple{Int,Int}}()

# Constante para definir cuántas bananas reaparecen
const num_bananas_respawn = 7

function place_banana(type::Symbol=:normal)
    walkable_positions = [(i, j) for i in 1:size(matrix, 1), j in 1:size(matrix, 2) if matrix[i, j] == 1]
    available = filter(pos -> !(pos in banana_positions) && !(pos in magic_banana_positions) && !(pos in green_banana_positions), walkable_positions)
    if !isempty(available)
        pos = rand(available)
        
        if type == :magic
            push!(magic_banana_positions, pos)
        elseif type == :green
            push!(green_banana_positions, pos)
        else
            push!(banana_positions, pos)
        end
        return pos
    end
    return nothing
end

# Keep for backward compatibility but default to normal
function place_banana_randomly()
    return place_banana(:normal)
end

# Función para buscar la banana más cercana y menos disputada.
function find_banana_in_vision(agent, model)
    if isempty(banana_positions) && isempty(magic_banana_positions) && isempty(green_banana_positions)
        return nothing
    end
    
    # Buscar todas las bananas visibles y ordenarlas por distancia
    visible_bananas = []
    
    # Check normal bananas
    for banana_pos in banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist))
        end
    end
    
    # Check magic bananas (prioritize them by artificially reducing distance)
    for banana_pos in magic_banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist - 2)) # Prioritize magic!
        end
    end

    # Check green bananas
    for banana_pos in green_banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist)) # Treat as normal
        end
    end

    if isempty(visible_bananas)
        return nothing
    end

    sort!(visible_bananas, by = x -> x[2]) # Ordenar por distancia (x[2])

    # Encontrar la más cercana que no esté "mejor" reclamada por otro agente
    for (banana_pos, my_dist) in visible_bananas
        is_contested_by_closer = false
        for other in allagents(model)
            # Solo considerar otros ratones
            if other isa Ghost && other.id != agent.id && 
            other.state == :chase && 
            other.target_banana == banana_pos &&
            manhattan_distance(other.pos, banana_pos) < my_dist
                is_contested_by_closer = true
                break 
            end
        end
        
        if !is_contested_by_closer
            return banana_pos
        end
    end
    return nothing # Todas las bananas visibles están mejor reclamadas por otros
end

# Comportamiento de persecución
function chase_behavior!(agent, model)
    #Validar si el objetivo aún existe
    if agent.target_banana === nothing || (!(agent.target_banana in banana_positions) && !(agent.target_banana in magic_banana_positions) && !(agent.target_banana in green_banana_positions))
        # El objetivo desapareció
        agent.state = :wander
        agent.route = []
        agent.target_banana = nothing
        wander_behavior!(agent, model) # Intentar merodear inmediatamente
        return
    end

    # Validar si otro agente está más cerca y también persiguiendo
    my_dist = manhattan_distance(agent.pos, agent.target_banana)
    for other in allagents(model)
        if other isa Ghost && other.id != agent.id &&
        other.state == :chase &&
        other.target_banana == agent.target_banana &&
        manhattan_distance(other.pos, agent.target_banana) < my_dist
            agent.state = :wander
            agent.route = []
            agent.target_banana = nothing
            return 
        end
    end

    #Comprobar si llegamos al objetivo
    if agent.pos == agent.target_banana
        if agent.pos in banana_positions
            delete!(banana_positions, agent.pos) # "Comer" la banana normal
        elseif agent.pos in magic_banana_positions
            delete!(magic_banana_positions, agent.pos) # "Comer" la banana mágica
            agent.powerup_time = 20 # ~3 seconds boost
            agent.slowdown_time = 0 # Clear slowdown
        elseif agent.pos in green_banana_positions
            delete!(green_banana_positions, agent.pos) # "Comer" la banana verde
            agent.slowdown_time = 20 # ~3 seconds slow
            agent.powerup_time = 0 # Clear powerup
        end
        
        # Volver a modo wander
        agent.state = :wander
        agent.route = []
        agent.target_banana = nothing
        return # Termina el turno
    end

    #Recalcular ruta si es necesario
    if isempty(agent.route)
        path = a_star_path(agent.pos, agent.target_banana, matrix)
        agent.route = length(path) > 1 ? path[2:end] : []
    end

    #Moverse por la ruta
    if !isempty(agent.route)
        next_pos = popfirst!(agent.route)
        move_agent!(agent, next_pos, model)
    else
        # No hay ruta, volver a wander
        agent.state = :wander
        agent.target_banana = nothing
    end
end

function free_neighbors(pos, matrix)
    y, x = pos
    moves = [(y-1, x), (y+1, x), (y, x-1), (y, x+1)]
    return [(ny, nx) for (ny, nx) in moves if 1 <= ny <= size(matrix, 1) && 1 <= nx <= size(matrix, 2) && matrix[ny, nx] == 1]
end

random_direction() = rand([(-1,0),(1,0),(0,-1),(0,1)])

# Comportamiento de wander
function wander_behavior!(agent, model)
    #Buscar bananas antes de moverse
    seen_banana = find_banana_in_vision(agent, model)
    if seen_banana !== nothing
        # Cambiar a modo persecución
        agent.state = :chase
        agent.target_banana = seen_banana
        agent.route = []
        chase_behavior!(agent, model) # Ejecutar lógica de persecución este mismo turno
        return
    end

    # Comprobar si nos tropezamos con una banana
    if agent.pos in banana_positions
        delete!(banana_positions, agent.pos)
        return # Termina el turno
    elseif agent.pos in magic_banana_positions
        delete!(magic_banana_positions, agent.pos)
        agent.powerup_time = 20 # ~3 seconds boost
        agent.slowdown_time = 0 # Clear slowdown
        return
    elseif agent.pos in green_banana_positions
        delete!(green_banana_positions, agent.pos)
        agent.slowdown_time = 20 # ~3 seconds slow
        agent.powerup_time = 0 # Clear powerup
        return
    end
    
    # Lógica de movimiento aleatorio con inercia
    if agent.steps < 5
        dy, dx = agent.dir
        new_pos = (agent.pos[1] + dy, agent.pos[2] + dx)
        
        # Comprobar límites y muros
        if 1 <= new_pos[1] <= size(matrix, 1) && 
           1 <= new_pos[2] <= size(matrix, 2) && 
           matrix[new_pos[1], new_pos[2]] == 1
            
            move_agent!(agent, new_pos, model)
            agent.steps += 1
            return
        else
            # Chocó con un muro, forzar cambio de dirección
            agent.steps = 5 
        end
    end

    # Cambiar de dirección (si steps >= 5 o si chocó)
    options = free_neighbors(agent.pos, matrix)
    if !isempty(options)
        new_pos = rand(options)
        # La nueva dirección es el movimiento que acabamos de decidir
        agent.dir = (new_pos[1] - agent.pos[1], new_pos[2] - agent.pos[2])
        move_agent!(agent, new_pos, model)
        agent.steps = 1 # Reiniciar contador (ya dimos 1 paso en la nueva dir)
    else
        # Elegir nueva dirección al azar
        agent.dir = random_direction()
        agent.steps = 0
    end
end

function agent_step!(agent::Ghost, model)
    # Handle powerup timer
    if agent.powerup_time > 0
        agent.powerup_time -= 1
    end

    # Handle slowdown timer
    if agent.slowdown_time > 0
        agent.slowdown_time -= 1
    end

    # Calculate effective speed
    # Normal: 1 step
    # Powerup: 2 steps
    # Slowdown: 0.5 steps (move every other tick)
    
    steps_to_take = 1
    
    if agent.powerup_time > 0 && agent.slowdown_time == 0
        steps_to_take = 2
    elseif agent.slowdown_time > 0 && agent.powerup_time == 0
        # Move only on odd ticks of slowdown (or even, doesn't matter)
        if agent.slowdown_time % 2 == 0
            steps_to_take = 0
        end
    elseif agent.powerup_time > 0 && agent.slowdown_time > 0
        # They cancel out to normal speed
        steps_to_take = 1
    end
    
    for _ in 1:steps_to_take
        if agent.state == :wander
            wander_behavior!(agent, model)
        else # agent.state == :chase
            chase_behavior!(agent, model)
        end
    end
end

function find_ghost_in_vision(gato::Gato, model)
    visible_ghosts = [(ghost.id, manhattan_distance(gato.pos, ghost.pos)) for ghost in allagents(model) if ghost isa Ghost && manhattan_distance(gato.pos, ghost.pos) <= vision_range]
    isempty(visible_ghosts) && return nothing
    # Always target the closest mouse for maximum efficiency
    sort!(visible_ghosts, by = x -> x[2]) 
    return visible_ghosts[1][1]  # Return ID of closest mouse
end

# Aggressive chase behavior - direct pursuit
function chase_behavior!(gato::Gato, model)
    target = gato.target_ghost !== nothing ? model[gato.target_ghost] : nothing
    
    # Validar que el objetivo (ratón) todavía exista
    if target === nothing
        gato.state = :wander
        gato.route = []
        gato.target_ghost = nothing
        return
    end

    # Always pursue the closest visible mouse for maximum aggression
    closest_ghost_id = find_ghost_in_vision(gato, model)
    if closest_ghost_id !== nothing
        gato.target_ghost = closest_ghost_id
        target = model[closest_ghost_id]
    end

    # Direct movement towards target (simplified pathfinding)
    target_y, target_x = target.pos
    cat_y, cat_x = gato.pos
    
    # Calculate direct movement (more aggressive than A*)
    dy = target_y > cat_y ? 1 : (target_y < cat_y ? -1 : 0)
    dx = target_x > cat_x ? 1 : (target_x < cat_x ? -1 : 0)
    
    # Try to move directly towards target
    next_pos = (cat_y + dy, cat_x + dx)
    
    # Check if direct move is valid
    if 1 <= next_pos[1] <= size(matrix, 1) && 
       1 <= next_pos[2] <= size(matrix, 2) && 
       matrix[next_pos[1], next_pos[2]] == 1
        move_agent!(gato, next_pos, model)
    else
        # If direct path blocked, use A* as fallback
        path = a_star_path(gato.pos, target.pos, matrix)
        if length(path) > 1
            move_agent!(gato, path[2], model)
        end
    end
    
    # Comprobar la captura después de moverse
    if gato.pos == target.pos
        remove_agent!(target, model)
        gato.state = :wander
        gato.route = []
        gato.target_ghost = nothing
        return
    end
end

# Aggressive wander - actively hunt for mice
function wander_behavior!(gato::Gato, model)
    # Look for ANY mouse in vision range (increased range)
    seen_ghost = find_ghost_in_vision(gato, model)
    if seen_ghost !== nothing
        gato.state = :chase
        gato.target_ghost = seen_ghost
        gato.route = [] 
        chase_behavior!(gato, model) # Switch to chase immediately
        return
    end
    
    # If no mice visible, move more aggressively to find them
    if gato.steps < 3  # Changed from 5 to 3 for more erratic hunting
        dy, dx = gato.dir
        new_pos = (gato.pos[1] + dy, gato.pos[2] + dx)
        if 1 <= new_pos[1] <= size(matrix, 1) && 
           1 <= new_pos[2] <= size(matrix, 2) &&
           matrix[new_pos[1], new_pos[2]] == 1
            
            move_agent!(gato, new_pos, model)
            gato.steps += 1
            return
        else
            gato.steps = 5
        end
    end
    options = free_neighbors(gato.pos, matrix)
    if !isempty(options)
        new_pos = rand(options)
        gato.dir = (new_pos[1] - gato.pos[1], new_pos[2] - gato.pos[2])
        move_agent!(gato, new_pos, model)
        gato.steps = 1
    else
        gato.dir = random_direction()
        gato.steps = 0
    end
end

function agent_step!(gato::Gato, model)
    if gato.state == :wander
        wander_behavior!(gato, model)
    else
        chase_behavior!(gato, model)
    end
end

# Función de paso del modelo para gestionar el reabastecimiento
function model_step!(model)
    # Si el set de bananas está vacío (y no hay mágicas ni verdes)
    if isempty(banana_positions) && isempty(magic_banana_positions) && isempty(green_banana_positions)
        
        # Check if any mouse is currently powered up or slowed down
        any_powered_up = false
        any_slowed_down = false
        for agent in allagents(model)
            if agent isa Ghost
                if agent.powerup_time > 0
                    any_powered_up = true
                end
                if agent.slowdown_time > 0
                    any_slowed_down = true
                end
            end
        end
        
        count = num_bananas_respawn
        
        # Logic: 1 in 7 is magic, BUT only if no one is powered up
        if !any_powered_up
            place_banana(:magic) # 1 Magic
            count -= 1
        end

        # Logic: Spawn green if no one is slowed down
        if !any_slowed_down && count > 0
            place_banana(:green) # 1 Green
            count -= 1
        end
        
        # Fill the rest with normal bananas
        for i in 1:count
            place_banana(:normal)
        end
    end
end


function initialize_model()
    space = GridSpace((14,17); periodic=false, metric=:manhattan)
    model = StandardABM(Union{Ghost,Gato}, space; agent_step!, model_step!, scheduler=Schedulers.ByID())
    
    #Añadimos los ratones
    positions = [(1,1), (1,17), (14,17), (3,3)]  # Moved (14,1) to (3,3) for better spacing
    for pos in positions
        add_agent!(Ghost, pos=pos, state=:wander, route=[], target_banana=nothing, dir=random_direction(), steps=0, powerup_time=0, slowdown_time=0, model)
    end

    #Añadimos el gato
    add_agent!(Gato, pos=(7,9), state=:wander, route=[], target_ghost=nothing, dir=random_direction(), steps=0, model)

    #add_agent!(Ghost, pos=(2,2), 
    #          state=:wander, route=[], target_banana=nothing,
    #          dir=random_direction(), steps=0, model)
    
    #add_agent!(Ghost, pos=(12,15), 
    #          state=:wander, route=[], target_banana=nothing,
    #          dir=random_direction(), steps=0, model)
                      
    # Usamos la constante para la carga inicial
    # Initial spawn: 1 Magic, 1 Green, rest Normal
    place_banana(:magic)
    place_banana(:green)
    for i in 1:(num_bananas_respawn - 2)
        place_banana(:normal)
    end
    return model
end

model = initialize_model()