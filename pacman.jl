using Agents

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
    1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1; 
    1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1; 
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1; 
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1  
]

@agent struct Ghost(GridAgent{2})
    state::Symbol  
    route::Vector{Tuple{Int,Int}}
    target_banana::Union{Tuple{Int,Int}, Nothing}
    dir::Tuple{Int,Int} 
    steps::Int      
    powerup_time::Int
    slowdown_time::Int 
    smart::Bool 
    teleport_cooldown::Int 
end

@agent struct Gato(GridAgent{2})
    state::Symbol
    route::Vector{Tuple{Int,Int}}
    target_ghost::Union{Int, Nothing}
    dir::Tuple{Int,Int}
    steps::Int
end

const vision_range = 6  # Rango de inteligencia/vision para el gato- ORIGINALMENTE 5

banana_positions = Set{Tuple{Int,Int}}()
magic_banana_positions = Set{Tuple{Int,Int}}()
green_banana_positions = Set{Tuple{Int,Int}}()

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

function place_banana_randomly()
    return place_banana(:normal)
end

function find_banana_in_vision(agent, model)
    if isempty(banana_positions) && isempty(magic_banana_positions) && isempty(green_banana_positions)
        return nothing
    end
    
    visible_bananas = []
    
    for banana_pos in banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist))
        end
    end
    
    for banana_pos in magic_banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist - 2)) 
        end
    end

    for banana_pos in green_banana_positions
        dist = manhattan_distance(agent.pos, banana_pos)
        if dist <= vision_range
            push!(visible_bananas, (banana_pos, dist)) 
        end
    end

    if isempty(visible_bananas)
        return nothing
    end

    sort!(visible_bananas, by = x -> x[2])

    # Encontrar la más cercana que no esté "mejor" reclamada por otro agente
    for (banana_pos, my_dist) in visible_bananas
        is_contested_by_closer = false
        for other in allagents(model)
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
    return nothing 
end

function is_occupied_by_mouse(pos, model, self_agent)
    for id in ids_in_position(pos, model)
        if id != self_agent.id && model[id] isa Ghost
            return true
        end
    end
    return false
end

# Comportamiento de persecución
function chase_behavior!(agent, model)
    if agent.target_banana === nothing || (!(agent.target_banana in banana_positions) && !(agent.target_banana in magic_banana_positions) && !(agent.target_banana in green_banana_positions))
        agent.state = :wander
        agent.route = []
        agent.target_banana = nothing
        wander_behavior!(agent, model) 
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
            delete!(banana_positions, agent.pos) 
        elseif agent.pos in magic_banana_positions
            delete!(magic_banana_positions, agent.pos) 
            agent.powerup_time = 20 
            agent.slowdown_time = 0 
        elseif agent.pos in green_banana_positions
            delete!(green_banana_positions, agent.pos) 
            agent.slowdown_time = 20
            agent.powerup_time = 0
        end
        
        # Volver a modo wander
        agent.state = :wander
        agent.route = []
        agent.target_banana = nothing
        return 
    end

    #Recalcular ruta si es necesario
    if isempty(agent.route)
        path = a_star_path(agent.pos, agent.target_banana, matrix)
        agent.route = length(path) > 1 ? path[2:end] : []
    end

    #Moverse por la ruta
    if !isempty(agent.route)
        next_pos = agent.route[1]
        if !is_occupied_by_mouse(next_pos, model, agent)
            popfirst!(agent.route)
            move_agent!(agent, next_pos, model)
        end
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
        agent.state = :chase
        agent.target_banana = seen_banana
        agent.route = []
        chase_behavior!(agent, model) 
        return
    end

    # Comprobar si nos tropezamos con una banana
    if agent.pos in banana_positions
        delete!(banana_positions, agent.pos)
        return 
    elseif agent.pos in magic_banana_positions
        delete!(magic_banana_positions, agent.pos)
        agent.powerup_time = 20 
        agent.slowdown_time = 0 
        return
    elseif agent.pos in green_banana_positions
        delete!(green_banana_positions, agent.pos)
        agent.slowdown_time = 20 
        agent.powerup_time = 0 
        return
    end
    
    # Lógica de movimiento aleatorio con inercia
    if agent.steps < 5
        dy, dx = agent.dir
        new_pos = (agent.pos[1] + dy, agent.pos[2] + dx)
        
        # Comprobar límites y muros
        if 1 <= new_pos[1] <= size(matrix, 1) && 
           1 <= new_pos[2] <= size(matrix, 2) && 
           matrix[new_pos[1], new_pos[2]] == 1 &&
           !is_occupied_by_mouse(new_pos, model, agent)
            
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
    options = filter(pos -> !is_occupied_by_mouse(pos, model, agent), options)

    if !isempty(options)
        new_pos = rand(options)
        # La nueva dirección es el movimiento que acabamos de decidir
        agent.dir = (new_pos[1] - agent.pos[1], new_pos[2] - agent.pos[2])
        move_agent!(agent, new_pos, model)
        agent.steps = 1 
    else
        agent.dir = random_direction()
        agent.steps = 0
    end
end

function find_cat_in_vision(agent, model)
    for other in allagents(model)
        if other isa Gato
            dist = manhattan_distance(agent.pos, other.pos)
            if dist <= vision_range
                return other.pos
            end
        end
    end
    return nothing
end

function flee_behavior!(agent, model, cat_pos)
    agent.state = :flee
    agent.route = [] 
    agent.target_banana = nothing
    
    neighbors = free_neighbors(agent.pos, matrix)
    if isempty(neighbors)
        return # Trapped
    end
    
    # MADRIGUERAS
    portal_1 = (1, 9)
    portal_2 = (14, 9)
    
    # Comprobar si un portal es accesible y seguro
    # Si estamos cerca de un portal, priorizarlo como ruta de escape
    # PERO SOLO si no lo hemos usado recientemente (teleport_cooldown == 0)
    target_portal = nothing
    
    if agent.teleport_cooldown == 0
        dist_p1 = manhattan_distance(agent.pos, portal_1)
        dist_p2 = manhattan_distance(agent.pos, portal_2)
        
        if dist_p1 <= 3 && manhattan_distance(cat_pos, portal_1) > 2 # Solo si el gato no está acampando la madriguera
            target_portal = portal_1
        elseif dist_p2 <= 3 && manhattan_distance(cat_pos, portal_2) > 2
            target_portal = portal_2
        end
    end
    
    if target_portal !== nothing
        # Mover hacia la madriguera
        path = a_star_path(agent.pos, target_portal, matrix)
        if length(path) > 1
            next_pos = path[2]
            if !is_occupied_by_mouse(next_pos, model, agent)
                move_agent!(agent, next_pos, model)
                return
            end
        end
    end
    
    # Lógica estándar de huida: Elegir vecino que maximice la distancia al gato
    neighbors = filter(pos -> !is_occupied_by_mouse(pos, model, agent), neighbors)
    
    best_pos = agent.pos
    max_dist = -1
    
    for pos in neighbors
        d = manhattan_distance(pos, cat_pos)
        if d > max_dist
            max_dist = d
            best_pos = pos
        end
    end
    
    if best_pos != agent.pos
        move_agent!(agent, best_pos, model)
        agent.dir = (best_pos[1] - agent.pos[1], best_pos[2] - agent.pos[2])
    end
end

function agent_step!(agent::Ghost, model)
    if agent.powerup_time > 0
        agent.powerup_time -= 1
    end

    if agent.slowdown_time > 0
        agent.slowdown_time -= 1
    end

    if agent.teleport_cooldown > 0
        agent.teleport_cooldown -= 1
    end

    portal_1 = (1, 9)
    portal_2 = (14, 9)
    
    if agent.teleport_cooldown == 0
        if agent.pos == portal_1
            if !is_occupied_by_mouse(portal_2, model, agent)
                move_agent!(agent, portal_2, model)
                agent.teleport_cooldown = 10 
                agent.route = [] 
                return 
            end
        elseif agent.pos == portal_2
            if !is_occupied_by_mouse(portal_1, model, agent)
                move_agent!(agent, portal_1, model)
                agent.teleport_cooldown = 10 
                agent.route = []
                return
            end
        end
    end
    
    steps_to_take = 1
    
    if agent.powerup_time > 0 && agent.slowdown_time == 0
        steps_to_take = 2
    elseif agent.slowdown_time > 0 && agent.powerup_time == 0
        if agent.slowdown_time % 2 == 0
            steps_to_take = 0
        end
    elseif agent.powerup_time > 0 && agent.slowdown_time > 0
        steps_to_take = 1
    end
    
    for _ in 1:steps_to_take
        # Lógica inteligente
        cat_pos = nothing
        if agent.smart
             cat_pos = find_cat_in_vision(agent, model)
        end

        if cat_pos !== nothing
             flee_behavior!(agent, model, cat_pos)
        elseif agent.state == :wander
            wander_behavior!(agent, model)
        else
            chase_behavior!(agent, model)
        end
    end
end

function find_ghost_in_vision(gato::Gato, model)
    visible_ghosts = [(ghost.id, manhattan_distance(gato.pos, ghost.pos)) for ghost in allagents(model) if ghost isa Ghost && manhattan_distance(gato.pos, ghost.pos) <= vision_range]
    isempty(visible_ghosts) && return nothing
    # Siempre apuntar al ratón más cercano para máxima eficiencia
    sort!(visible_ghosts, by = x -> x[2]) 
    return visible_ghosts[1][1]  # Devolver ID del ratón más cercano
end

function chase_behavior!(gato::Gato, model)
    target = gato.target_ghost !== nothing ? model[gato.target_ghost] : nothing
    
    # Validar que el objetivo (ratón) todavía exista
    if target === nothing
        gato.state = :wander
        gato.route = []
        gato.target_ghost = nothing
        return
    end

    # Siempre perseguir al ratón visible más cercano para máxima agresividad
    closest_ghost_id = find_ghost_in_vision(gato, model)
    if closest_ghost_id !== nothing
        gato.target_ghost = closest_ghost_id
        target = model[closest_ghost_id]
    end

    # Movimiento directo hacia el objetivo (búsqueda simplificada)
    target_y, target_x = target.pos
    cat_y, cat_x = gato.pos
    
    # Calcular movimiento directo (más agresivo que A*)
    dy = target_y > cat_y ? 1 : (target_y < cat_y ? -1 : 0)
    dx = target_x > cat_x ? 1 : (target_x < cat_x ? -1 : 0)
    
    # Intentar moverse directamente hacia el objetivo
    next_pos = (cat_y + dy, cat_x + dx)
    
    if 1 <= next_pos[1] <= size(matrix, 1) && 
       1 <= next_pos[2] <= size(matrix, 2) && 
       matrix[next_pos[1], next_pos[2]] == 1
        move_agent!(gato, next_pos, model)
    else
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

# Gato wander behavior
function wander_behavior!(gato::Gato, model)
    seen_ghost = find_ghost_in_vision(gato, model)
    if seen_ghost !== nothing
        gato.state = :chase
        gato.target_ghost = seen_ghost
        gato.route = [] 
        chase_behavior!(gato, model)
    end
    
    # Si no detecta a los ratones se vuelve el gato más agresivo
    if gato.steps < 3  
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
        
        # Checa si el ratón está potenciado o ralentizado, o estado original
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
        
        # ógica: 1 de cada 7 es mágica, PERO solo si ningún ratón está potenciado
        if !any_powered_up
            place_banana(:magic)
            count -= 1
        end

        # Sale banana verde si ningún ratón está ralentizado
        if !any_slowed_down && count > 0
            place_banana(:green)
            count -= 1
        end
        
        # Llenado de bananas normales hasta el total
        for i in 1:count
            place_banana(:normal)
        end
    end
end


function initialize_model()
    space = GridSpace((14,17); periodic=false, metric=:manhattan)
    model = StandardABM(Union{Ghost,Gato}, space; agent_step!, model_step!, scheduler=Schedulers.ByID())
    
    #AGREGA RATONES
    positions = [(1,1), (1,17), (14,17), (3,3)]  
    for (i, pos) in enumerate(positions)
        is_smart = i <= 2
        add_agent!(Ghost, pos=pos, state=:wander, route=[], target_banana=nothing, dir=random_direction(), steps=0, powerup_time=0, slowdown_time=0, smart=is_smart, teleport_cooldown=0, model)
    end

    #AGREGAMOS EL GATO
    add_agent!(Gato, pos=(7,9), state=:wander, route=[], target_ghost=nothing, dir=random_direction(), steps=0, model)

    place_banana(:magic)
    place_banana(:green)
    for i in 1:(num_bananas_respawn - 2)
        place_banana(:normal)
    end
    return model
end

model = initialize_model()