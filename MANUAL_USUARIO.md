# Manual de usuario - Proyecto Ratones

Este documento explica de forma sencilla cómo ejecutar y usar el proyecto ubicado en la carpeta `ratones`.

## Resumen

Es una pequeña escena 3D en Python que muestra ratones, un gato, quesos, cajas y escobas usando PyGame y OpenGL. El proyecto puede sincronizarse con una simulación externa (API en Julia) para actualizar posiciones en tiempo real si la API está disponible.

## Requisitos

- Python 3.8+ (probado con Python 3.8 - 3.11)
- pip
- Paquetes Python:
  - pygame
  - PyOpenGL
  - requests (opcional, solo si desea conectar con la API de Julia)

Instalar dependencias (Windows PowerShell):

```powershell
python -m pip install --upgrade pip
pip install pygame PyOpenGL PyOpenGL_accelerate requests
```

Nota: `PyOpenGL_accelerate` es opcional pero mejora el rendimiento.

## Archivos importantes

- `main.py`: Entrada principal. Inicializa la ventana, carga objetos, y ejecuta el bucle de render.
- `raton.py`: Clase `Raton` (ratones) — lógica de movimiento, colisiones y render.
- `gato.py`: Clase `Gato` — lógica y render del gato.
- `queso.py`: Clase `Queso` — objetos de recolección (queso/bananas).
- `caja.py`: Clase `Caja` — obstáculo estático.
- `escoba.py`: Clase `Escoba` — obstáculo estático.
- `objloader.py`: Cargador de archivos `.obj` y `.mtl` usado por los modelos 3D.
- `Textures/`, `raton/`, `gato/`: Carpetas con modelos `.obj` y texturas.

## Cómo ejecutar

1. Abra una terminal (PowerShell) y buscar en la carpeta del proyecto `ratones`:

```powershell
cd d:\mas25b\ratones
```

2. Inicie la aplicación:

```powershell
python main.py
```

Comportamiento esperado:
- Se abrirá una ventana con una escena 3D. Si la API de Julia (por defecto `http://localhost:8000`) está disponible, las posiciones de ratones, gato y quesos se sincronizarán con la simulación.

## Controles

- Cerrar la ventana: botón de cerrar de la ventana o salir del proceso.

## Parámetros configurables

- En `main.py` puede modificar: resolución de ventana (`screen_width`, `screen_height`), FOV, límites del tablero (`DimBoard`) y número de objetos (`NUM_ESCOBAS`, `NUM_QUESOS`, `NUM_CAJAS`, `NUM_RATONES`).
- URL de la API: cambiar `API_BASE_URL` al endpoint correcto si la API de Julia está en otro puerto/host.

## Integración con la API de Julia

Si dispone de la simulación en Julia que expone una API REST (por defecto `http://localhost:8000`), el proyecto intentará: 

- Obtener el estado del juego (`/game-state`) para posiciones iniciales y actualización de agentes.
- Enviar posición del ratón o gato mediante `POST` a `/update-mouse/{id}` y `/update-cat`.
- Hacer avanzar la simulación con `GET /run` en cada intervalo.

Para aprovechar esto, asegúrese de tener `requests` instalado y la API accesible.

## Errores y soluciones comunes

- Error: archivo `.obj` o `.mtl` no encontrado -> Verifique que las carpetas `raton/`, `gato/`, y `Textures/` estén completas y que las rutas relativas no hayan sido cambiadas.
- Error de OpenGL al crear contexto -> Asegúrese de tener controladores gráficos actualizados y que su entorno soporte OpenGL. En Windows, evite entornos remotos sin aceleración 3D.
- Si la aplicación no se conecta a la API de Julia -> Compruebe que la API esté en ejecución y accesible en `API_BASE_URL`. La aplicación ignora errores de red por rendimiento, así que revise logs o ejecute manualmente `curl http://localhost:8000/game-state` para verificar.

## Estructura del código (breve explicación)

- `main.py` inicializa PyGame y OpenGL, crea las listas de objetos (ratones, quesos, cajas, escobas, gato), y entra en un bucle de render/actualización:
  - `Init()` carga recursos y genera objetos.
  - `display()` dibuja skybox, piso y objetos.
  - Funciones auxiliares para conversión de coordenadas entre la cuadrícula de Julia y el mundo 3D.
- Cada clase (`Raton`, `Gato`, `Queso`, `Caja`, `Escoba`) implementa `draw()` y `update()` o lógica similar; los modelos 3D se cargan con `OBJ`.

