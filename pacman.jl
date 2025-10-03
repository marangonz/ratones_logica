using Agents

@agent struct Ghost(GridAgent{2})
    rol::String = "Ghost"
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

#se agregó el matrix que vimos en clase. Este es el verdadero mapa que estará 
#evaluando nuestro mono. Por orden, lo eliminamos cuando estabamos en clase para
#que se entienda el proceso y la funcionalidad del mapa.


function agent_step!(agent, model)
    randomwalk!(agent, model)
    x, y = agent.pos #ponemos posición actual de nuestro mono (agente)
    movimientos =  [(x+1, y), (x-1, y), (x, y+1), (x, y-1)] #definimos posibles movimientos
    validos_moves =[]

    #Después, hay que comprobar que los movimientos que haga nuestro 
    #monito sean validos (es decir, que sea un camino valido)
      for (nx, ny) in movimientos
        if nx ≥ 1 && nx ≤ size(matrix, 1) && ny ≥ 1 && ny ≤ size(matrix, 2)
            if matrix[nx, ny] == 1
                push!(validos_moves, (nx, ny))
            end
        end
    end
    if !isempty(validos_moves)
        agent.pos = rand(validos_moves)
    end
end

function initialize_model()
    space = GridSpace((5,5); periodic = false, metric = :manhattan)
    model = StandardABM(Ghost, space; agent_step!)
    return model
end

model = initialize_model()
a = add_agent!(Ghost, pos=(3, 3), model)