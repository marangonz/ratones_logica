include("pacman.jl")
using Genie, Genie.Renderer.Json, Genie.Requests, HTTP
using UUIDs

route("/run") do
    run!(model, 1)
    agents = []
    for ghost in allagents(model)
        push!(agents, ghost)
    end

    # Transformamos el Dict a una lista de listas [[coords, tipo], ...]
    # Esto asegura que el JSON sea [[[x,y], "normal"], ...]
    formatted_bananas = [[pos, type] for (pos, type) in banana_positions]

    json(Dict(:msg => "Adios", "agents" => agents, "bananas" => formatted_bananas))
end

route("/new-banana") do
    # place_banana_randomly usa :normal por defecto si no se especifica
    new_banana_pos = place_banana_randomly() 
    
    # Formateamos la colección completa para devolverla
    formatted_bananas = [[pos, type] for (pos, type) in banana_positions]

    if new_banana_pos !== nothing
        # Recuperamos el tipo de la nueva banana para enviarlo individualmente también
        new_type = banana_positions[new_banana_pos]
        
        json(Dict(
            :msg => "New banana placed", 
            "banana" => [new_banana_pos, new_type], # Enviamos [pos, tipo]
            "bananas" => formatted_bananas
        ))
    else
        json(Dict(:msg => "No available positions for new banana", "bananas" => formatted_bananas))
    end
end

route("/bananas") do
    # Formateamos para /bananas también
    formatted_bananas = [[pos, type] for (pos, type) in banana_positions]
    json(Dict(:msg => "Current bananas", "bananas" => formatted_bananas))
end

Genie.config.run_as_server = true
Genie.config.cors_headers["Access-Control-Allow-Origin"] = "*"
Genie.config.cors_headers["Access-Control-Allow-Headers"] = "Content-Type"
Genie.config.cors_headers["Access-Control-Allow-Methods"] = "GET,POST,PUT,DELETE,OPTIONS"
Genie.config.cors_allowed_origins = ["*"]


up(host="0.0.0.0")