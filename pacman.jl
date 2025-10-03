using Agents, Agents.Pathfinding

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

#function move_along_route!(agent, distance::Int)
    # Move the agent along its route for 'distance' steps
    #steps = min(distance, length(agent.route))
    #for i in 1:steps
    #    agent.pos = agent.route[1]
    #    popfirst!(agent.route)
    #end
    #return distance - steps
#end

#function agent_step!(agent, model)
#    start = agent.pos
#    goal = (13, 17)

#    accessible(p) = begin
#        x, y = p
#        1 ≤ x ≤ size(matrix, 1) && 1 ≤ y ≤ size(matrix, 2) && matrix[x, y] == 1
#    end

    # If agent has no route or reached goal, recalculate
#    if isempty(agent.route) && start != goal
#        path = a_star(start, goal; valid_move=accessible, metric=manhattan)
#        agent.route = path[2:end]  # Exclude current position
#    end

#    if !isempty(agent.route)
#        move_along_route!(agent, 1)  # Move 1 step per tick
#    end
#end

@agent struct Ghost(GridAgent{2})
end

function initialize_model()
    space = GridSpace((5,5); periodic = false, metric = :manhattan)
    model = StandardABM(Ghost, space; agent_step!)
    return model
end

model = initialize_model()
a = add_agent!(Ghost, pos=(3, 3), model)