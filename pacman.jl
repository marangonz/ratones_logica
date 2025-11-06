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
end

@agent struct Gato(GridAgent{2})
    state::Symbol
    route::Vector{Tuple{Int,Int}}
    target_ghost::Union{Int, Nothing}
    dir::Tuple{Int,Int}
    steps::Int
    # Definir propiedades del gato si es necesario
end

const vision_range = 5

# =====================================
# === BANANAS =========================
# =====================================
banana_positions = Set{Tuple{Int,Int}}()

# Constante para definir cuántas bananas reaparecen
const num_bananas_respawn = 7

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

# Función para buscar la banana más cercana y menos disputada.
function find_banana_in_vision(agent, model)
    if isempty(banana_positions)
        return nothing
    end
    
    # Buscar todas las bananas visibles y ordenarlas por distancia
    visible_bananas = []
    for banana_pos in banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist))
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
    if agent.target_banana === nothing || !(agent.target_banana in banana_positions)
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
        delete!(banana_positions, agent.pos) # "Comer" la banana
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
    if agent.state == :wander
        wander_behavior!(agent, model)
    else # agent.state == :chase
        chase_behavior!(agent, model)
    end
end

function find_ghost_in_vision(gato::Gato, model)
    visible_ghosts = [(ghost.id, manhattan_distance(gato.pos, ghost.pos)) for ghost in allagents(model) if ghost isa Ghost && manhattan_distance(gato.pos, ghost.pos) <= vision_range]
    isempty(visible_ghosts) && return nothing
    sort!(visible_ghosts, by = x -> x[2]) 
    return first(visible_ghosts[1])
end

# Se añade lógica para re-evaluar el objetivo en cada paso
function chase_behavior!(gato::Gato, model)
    target = gato.target_ghost !== nothing ? model[gato.target_ghost] : nothing
    
    # Validar que el objetivo (ratón) todavía exista
    if target === nothing
        gato.state = :wander
        gato.route = []
        gato.target_ghost = nothing
        return # Termina el turno, merodeará en el siguiente
    end

    # Re-evaluar el objetivo visible más cercano
    best_visible_ghost_id = find_ghost_in_vision(gato, model)
    
    # Si vemos un ratón, y no es nuestro objetivo actual
    if best_visible_ghost_id !== nothing && best_visible_ghost_id != gato.target_ghost
        current_target_dist = manhattan_distance(gato.pos, target.pos)
        
        # Comprobar la distancia al nuevo objetivo potencial
        best_visible_target = model[best_visible_ghost_id]
        best_visible_dist = manhattan_distance(gato.pos, best_visible_target.pos)
        
        # Si el nuevo objetivo está más cerca que el actual, cambiar de objetivo
        if best_visible_dist < current_target_dist
            gato.target_ghost = best_visible_ghost_id
            target = best_visible_target # Actualizar el 'target' local
            gato.route = [] # Forzar recálculo de ruta
        end
    end

    # Recalcular la ruta si está vacía o si el objetivo se movió
    if isempty(gato.route) || last(gato.route) != target.pos
        path = a_star_path(gato.pos, target.pos, matrix)
        gato.route = length(path) > 1 ? path[2:end] : []
    end
    
    # Moverse por la ruta
    if !isempty(gato.route)
        move_agent!(gato, popfirst!(gato.route), model)
    end
    
    # Comprobar la captura después de moverse
    target = gato.target_ghost !== nothing ? model[gato.target_ghost] : nothing
    if target !== nothing && gato.pos == target.pos
        remove_agent!(target, model)
        gato.state = :wander
        gato.route = []
        gato.target_ghost = nothing
        return
    end
end

# Añadida la comprobación de muros
function wander_behavior!(gato::Gato, model)
    seen_ghost = find_ghost_in_vision(gato, model)
    if seen_ghost !== nothing
        gato.state = :chase
        gato.target_ghost = seen_ghost
        gato.route = [] 
        chase_behavior!(gato, model) # Llamar a chase de inmediato
        return
    end
    if gato.steps < 5
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
    # Si el set de bananas está vacío
    if isempty(banana_positions)
        # Volver a llenar el mapa con bananas
        for i in 1:num_bananas_respawn
            place_banana_randomly()
        end
    end
end


function initialize_model()
    space = GridSpace((14,17); periodic=false, metric=:manhattan)
    model = StandardABM(Union{Ghost,Gato}, space; agent_step!, model_step!, scheduler=Schedulers.ByID())
    
    #Añadimos los ratones
    positions = [(1,1), (1,17), (14,1), (14,17)]
    for pos in positions
        add_agent!(Ghost, pos=pos, state=:wander, route=[], target_banana=nothing, dir=random_direction(), steps=0, model)
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
    for i in 1:num_bananas_respawn
        place_banana_randomly()
    end
    return model
end

model = initialize_model()
