import { useState, useEffect } from 'react'

const App = () => {
  const [agents, setAgents] = useState([]);
  const [bananas, setBananas] = useState([]);

  useEffect(() => {
    const interval = setInterval(() => {
      fetch("http://localhost:8000/run")
      .then(res => res.json())
      .then(res => {
        // Update all agent positions (convert from 1-based to 0-based indexing)
        setAgents(res.agents.map(agent => ({
          id: agent.id,
          x: agent.pos[0] - 1,
          y: agent.pos[1] - 1
        })));
        
        // Update banana positions (convert from 1-based to 0-based indexing)
        // Ahora procesamos el par [[x,y], tipo] que envía el backend
        if (res.bananas) {
          setBananas(res.bananas.map(banana => ({
            x: banana[0][0] - 1, // Coordenada X (ajustada a índice 0)
            y: banana[0][1] - 1, // Coordenada Y (ajustada a índice 0)
            type: banana[1]      // Tipo: "normal" o "special"
          })));
        }
      });
    }, 1000); // Changed from 5000ms to 1000ms (5x faster)

      return () => clearInterval(interval);
  }, []);

  let matrix = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
  ];

  return (
    <div>
      <svg width="800" height="500" style={{backgroundColor: "lightgray"}} xmlns="http://www.w3.org/2000/svg">
            {
        matrix.map((row, rowidx) =>
          row.map((value, colidx) =>
            <rect x={250 + 25 * rowidx} y={5 + 25 * colidx} width={25} height={25} fill={value == 1 ? "gray" : "lightgray"}/>
      ))
      }

        {agents.map((agent, index) => (
          <image 
            key={agent.id} 
            x={255 + 25 * agent.x} 
            y={9 + 25 * agent.y} 
            href={agent.id === 5 ? "jaguar.png" : "monkey.png"}
          />
        ))}
        {/* Renderizado condicional de la imagen según el tipo */}
        {bananas.map((banana, index) => (
          <image 
            key={index} 
            x={255 + 25 * banana.x} 
            y={9 + 25 * banana.y} 
            width={20}
            height={20}
            href={banana.type === "special" ? "banana_especial.png" : "banana2.png"}
          />
        ))}
      </svg>
    </div>
    
  );
};

export default App;