include("pacman.jl")
using Genie, Genie.Renderer.Json, Genie.Requests, HTTP
using UUIDs

route("/run") do
    run!(model, 1)
    agents = []
    for ghost in allagents(model)
        push!(agents, ghost)
    end

    json(Dict(:msg => "Adios", "agents" => agents, "bananas" => collect(banana_positions)))
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

Genie.config.run_as_server = true
Genie.config.cors_headers["Access-Control-Allow-Origin"] = "*"
Genie.config.cors_headers["Access-Control-Allow-Headers"] = "Content-Type"
Genie.config.cors_headers["Access-Control-Allow-Methods"] = "GET,POST,PUT,DELETE,OPTIONS"
Genie.config.cors_allowed_origins = ["*"]

up(host="0.0.0.0")