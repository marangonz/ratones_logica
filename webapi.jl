include("pacman.jl")
using Genie, Genie.Renderer.Json, Genie.Requests, HTTP
using UUIDs

route("/run") do
    run!(model, 1)
    
    # Note: Banana maintenance is handled automatically in model_step!
    
    # Collect all agent positions (Same format as /game-state)
    mice_positions = []
    cat_position = nothing
    
    for agent in allagents(model)
        if agent isa Ghost
            push!(mice_positions, Dict(
                "id" => agent.id, 
                "pos" => [agent.pos[1], agent.pos[2]], 
                "state" => string(agent.state),
                "powered_up" => agent.powerup_time > 0
            ))
        elseif agent isa Gato
            cat_position = Dict("id" => agent.id, "pos" => [agent.pos[1], agent.pos[2]], "state" => string(agent.state))
        end
    end

    json(Dict(
        :msg => "Step completed", 
        "mice" => mice_positions,
        "cat" => cat_position,
        "bananas" => collect(banana_positions),
        "magic_bananas" => collect(magic_banana_positions),
        "grid_size" => [size(matrix, 1), size(matrix, 2)]
    ))
end

route("/new-banana") do
    new_banana_pos = place_banana_randomly()
    if new_banana_pos !== nothing
        json(Dict(:msg => "New banana placed", "banana" => new_banana_pos, "bananas" => collect(banana_positions)))
    else
        json(Dict(:msg => "No available positions for new banana", "bananas" => collect(banana_positions)))
    end
end

route("/bananas") do
    json(Dict(:msg => "Current bananas", "bananas" => collect(banana_positions)))
end

route("/regenerate-bananas") do
    # Clear existing bananas and regenerate
    empty!(banana_positions)
    for i in 1:num_bananas_respawn
        place_banana_randomly()
    end
    json(Dict(:msg => "Bananas regenerated for study purposes", "bananas" => collect(banana_positions)))
end

route("/game-state") do
    # Collect all agent positions
    mice_positions = []
    cat_position = nothing
    
    for agent in allagents(model)
        if agent isa Ghost
            push!(mice_positions, Dict(
                "id" => agent.id, 
                "pos" => [agent.pos[1], agent.pos[2]], 
                "state" => string(agent.state),
                "powered_up" => agent.powerup_time > 0
            ))
        elseif agent isa Gato
            cat_position = Dict("id" => agent.id, "pos" => [agent.pos[1], agent.pos[2]], "state" => string(agent.state))
        end
    end
    
    json(Dict(
        :msg => "Current game state",
        "mice" => mice_positions,
        "cat" => cat_position,
        "bananas" => collect(banana_positions),
        "magic_bananas" => collect(magic_banana_positions),
        "grid_size" => [size(matrix, 1), size(matrix, 2)]
    ))
end

Genie.config.run_as_server = true
Genie.config.cors_headers["Access-Control-Allow-Origin"] = "*"
Genie.config.cors_headers["Access-Control-Allow-Headers"] = "Content-Type"
Genie.config.cors_headers["Access-Control-Allow-Methods"] = "GET,POST,PUT,DELETE,OPTIONS"
Genie.config.cors_allowed_origins = ["*"]

#up(host="0.0.0.0", port=6000)
up()