# Autor: Ivan Olmos Pineda (modificado)
# Curso: Multiagentes - Graficas Computacionales
# EQUIPO 3

import os
import sys
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
from pygame.locals import *

# OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import math
sys.path.append('..')

from raton import Raton
from escoba import Escoba
from queso import Queso
from caja import Caja
from gato import Gato
from objloader import *
import random
import json
import threading
import time

try:
    import requests
    API_AVAILABLE = True
except ImportError:
    API_AVAILABLE = False

class JuliaClient:
    def __init__(self, url):
        self.url = url
        self.lock = threading.Lock()
        self.state = None
        self.new_data = False 
        self.running = True
        self.should_step = False
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        
    def start(self):
        self.thread.start()
        
    def stop(self):
        self.running = False
        
    def _loop(self):
        target_interval = 0.1 
        
        while self.running:
            start_time = time.time()
            try:
                if self.should_step:
                    response = requests.get(f"{self.url}/run", timeout=0.5)
                else:
                    response = requests.get(f"{self.url}/game-state", timeout=0.5)
                
                if response.status_code == 200:
                    with self.lock:
                        self.state = response.json()
                        self.new_data = True
                
                elapsed = time.time() - start_time
                sleep_time = max(0, target_interval - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                time.sleep(1.0)

    def get_latest_state(self):
        with self.lock:
            if self.new_data:
                self.new_data = False
                return self.state
            return None 

    def set_stepping(self, enabled):
        self.should_step = enabled

# Ventana
screen_width = 1200
screen_height = 800

# Cámara / proyección - Ajustada para vista panorámica óptima
FOVY = 85.0     
ZNEAR = 1.0
ZFAR = 1500.0    

# POSICIÓN inicial de la cámara: Optimizada para vista panorámica completa
EYE_X = 0.0     
EYE_Y = 400.0    
EYE_Z = 200.0    

# La cámara mira hacia el CENTRO del escenario con ligera inclinación hacia abajo
CENTER_X = 0.0  
CENTER_Y = -50.0
CENTER_Z = 0.0  

# Vector UP normal
UP_X = 0
UP_Y = 1
UP_Z = 0

# Límites y tablero
X_MIN = -500
X_MAX = 500
Y_MIN = -500
Y_MAX = 500
Z_MIN = -500
Z_MAX = 500

# Tamaño del cubito y el plano
DimBoard = 600  
CUBE_SIZE = 1200 

# Movimiento/cámara
dir = [1.0, 0.0, 0.0]
theta = 0.0

pygame.init()
pygame.mixer.init()

# Load sounds
squeak_sound = None
try:
    if os.path.exists("squeak.mp3"):
        squeak_sound = pygame.mixer.Sound("squeak.mp3")
    else:
        print("Warning: squeak.mp3 not found")
except Exception as e:
    print(f"Warning: Could not load squeak.mp3: {e}")

# Texturas global
textures = []
floor_texture = None

# Listas de objetos
escobas = []
quesos = []
cajas = []
ratones = []  
gato = None
NUM_ESCOBAS = 5 
NUM_QUESOS = 7   
NUM_CAJAS = 4   
NUM_RATONES = 4  

# Configuración de la API
API_BASE_URL = "http://localhost:8000"
julia_client = None
last_api_update = 0
API_UPDATE_INTERVAL = 0.15

game_over = False
game_over_time = 0
GAME_OVER_DELAY = 3.0  
simulation_initialized = False  

last_cheese_positions = set()

JULIA_GRID_HEIGHT = 14
JULIA_GRID_WIDTH = 17

# Colisiones
class CollisionObject:
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius

def get_obstacle_list():
    """Genera la lista de colisiones """
    obstacles = []
    
    broom_positions = [
        [-150, 0, -100],  # Escoba 1
        [200, 0, -150],   # Escoba 2  
        [-200, 0, 150],   # Escoba 3
        [100, 0, 200],    # Escoba 4
        [0, 0, -250]      # Escoba 5
    ]
    for pos in broom_positions:
        obstacles.append(CollisionObject([pos[0], pos[1], pos[2]], 40.0))
    
    box_positions = [
        [-100, 0, -300],  # Caja 1
        [150, 0, -250],   # Caja 2
        [-200, 0, 300],   # Caja 3
        [200, 0, 250]     # Caja 4
    ]
    for pos in box_positions:
        obstacles.append(CollisionObject([pos[0], pos[1], pos[2]], 50.0))
    
    return obstacles

def check_collision(position, agent_radius, obstacles):
    for obstacle in obstacles:
        dx = position[0] - obstacle.position[0]
        dz = position[2] - obstacle.position[2] 
        distance = math.sqrt(dx*dx + dz*dz)
        
        if distance < (agent_radius + obstacle.radius):
            return True
    return False

def find_safe_position(current_pos, target_pos, agent_radius, obstacles, max_attempts=8):
    if not check_collision(target_pos, agent_radius, obstacles):
        return target_pos 
    
    for i in range(max_attempts):
        angle = (2 * math.pi * i) / max_attempts
        offset_distance = agent_radius * 2  
        
        safe_x = target_pos[0] + math.cos(angle) * offset_distance
        safe_z = target_pos[2] + math.sin(angle) * offset_distance
        safe_pos = [safe_x, target_pos[1], safe_z]
        
        safe_pos = clamp_to_camera_bounds(safe_pos)
        
        if not check_collision(safe_pos, agent_radius, obstacles):
            return safe_pos
    
    return current_pos

def clamp_to_camera_bounds(position, bounds=300):
    """Asegura que la posición se mantenga dentro del área visible de la cámara"""
    position[0] = max(-bounds, min(bounds, position[0]))  # Eje X
    position[2] = max(-bounds, min(bounds, position[2]))  # Eje Z
    return position

def julia_to_python_coords(julia_pos):
    grid_y, grid_x = julia_pos 
    
    norm_x = (grid_x - 1) / (JULIA_GRID_WIDTH - 1)  
    norm_y = (grid_y - 1) / (JULIA_GRID_HEIGHT - 1)  
    
    camera_visible_area = 300  
    world_x = (norm_x * 2 - 1) * camera_visible_area  
    world_z = (norm_y * 2 - 1) * camera_visible_area 
    
    position = [world_x, 0.0, world_z]
    return clamp_to_camera_bounds(position)

def python_to_julia_coords(python_pos):
    world_x, _, world_z = python_pos
    
    camera_visible_area = 300  
    norm_x = world_x / camera_visible_area  
    norm_z = world_z / camera_visible_area  
    
    grid_x = int((norm_x + 1) / 2 * (JULIA_GRID_WIDTH - 1)) + 1
    grid_y = int((norm_z + 1) / 2 * (JULIA_GRID_HEIGHT - 1)) + 1  
    
    grid_x = max(1, min(JULIA_GRID_WIDTH, grid_x))
    grid_y = max(1, min(JULIA_GRID_HEIGHT, grid_y))

    return (grid_y, grid_x)

# Buscar imagen de la casa (jpeg/jpg) en la carpeta del script
base_path = os.path.dirname(__file__)
def find_house_image():
    candidates = ["OLD_HOUSE.jpeg"]
    for name in candidates:
        p = os.path.join(base_path, name)
        if os.path.exists(p):
            return p
    return None

# Buscar imagen para el piso
def find_floor_image():
    candidates = ["floor3.jpg"]
    for name in candidates:
        p = os.path.join(base_path, name)
        if os.path.exists(p):
            return p
    return None

# API Communication Functions
def send_mouse_position(mouse_id, position):
    """Enviar posición del ratón a la API de Julia"""
    if not API_AVAILABLE:
        return None
    try:
        data = {
            "position": {
                "x": position[0],
                "y": position[1], 
                "z": position[2]
            }
        }
        response = requests.post(f"{API_BASE_URL}/update-mouse/{mouse_id}", 
                               json=data, timeout=5)
        if response.status_code == 200:
            return response.json()
    except Exception as e:
        pass 
    return None

def send_cat_position(position):
    """Enviar posición del gato a la API de Julia"""
    if not API_AVAILABLE:
        return None
    try:
        data = {
            "position": {
                "x": position[0],
                "y": position[1],
                "z": position[2]
            }
        }
        response = requests.post(f"{API_BASE_URL}/update-cat", 
                               json=data, timeout=5)
        if response.status_code == 200:
            return response.json()
    except Exception as e:
        pass 
    return None

def update_from_julia_simulation():
    """Actualiza todas las posiciones de los objetos desde la simulación de Pacman en Julia"""
    global ratones, gato, quesos, game_over, game_over_time, simulation_initialized
    
    if not julia_client:
        return False
        
    game_state = julia_client.get_latest_state()
    if not game_state:
        return False
    
    # Actualiza las posiciones de los quesos/bananas
    if "bananas" in game_state:
        global last_cheese_positions
        
        current_normal = set(tuple(p) for p in game_state.get("bananas", []))
        current_magic = set(tuple(p) for p in game_state.get("magic_bananas", []))
        current_green = set(tuple(p) for p in game_state.get("green_bananas", []))
        
        # Create a combined signature to detect changes
        current_signature = (frozenset(current_normal), frozenset(current_magic), frozenset(current_green))
        
        # Solo se actualiza solo si hay cambios en las posiciones de los quesos en JULIA
        if current_signature != last_cheese_positions:
            quesos.clear()
            
            # Agregamos quesos normales
            for banana_pos in game_state.get("bananas", []):
                world_pos = julia_to_python_coords(banana_pos)
                queso = Queso(dim_board=DimBoard, scale=1.0, cheese_type='normal')
                queso.Position = world_pos
                quesos.append(queso)
                
            # Agregamos quesos azules
            for banana_pos in game_state.get("magic_bananas", []):
                world_pos = julia_to_python_coords(banana_pos)
                queso = Queso(dim_board=DimBoard, scale=1.0, cheese_type='magic')
                queso.Position = world_pos
                quesos.append(queso)

            # Agregamos quesitos verdes
            for banana_pos in game_state.get("green_bananas", []):
                world_pos = julia_to_python_coords(banana_pos)
                queso = Queso(dim_board=DimBoard, scale=1.0, cheese_type='green')
                queso.Position = world_pos
                quesos.append(queso)
            
            last_cheese_positions = current_signature
    
    # Actualiza las posiciones de los ratones
    if "mice" in game_state:
        julia_mice = game_state["mice"]
        julia_mouse_ids = [mouse["id"] for mouse in julia_mice]
        
        if not simulation_initialized and len(julia_mice) > 0:
            if len(julia_mice) >= 4:
                simulation_initialized = True
            elif len(julia_mice) >= 3:
                simulation_initialized = True
            else:
                simulation_initialized = True
        
        previously_alive = [i for i, r in enumerate(ratones) if not r.captured]
        
        for i, raton in enumerate(ratones):
            raton.captured = True  
        
        if len(julia_mice) != len(set(julia_mouse_ids)):
            pass  
        
        alive_indices = []
        for julia_mouse in julia_mice:
            mouse_id = julia_mouse["id"]
            python_mouse_index = mouse_id - 1
            if 0 <= python_mouse_index < len(ratones):
                world_pos = julia_to_python_coords(julia_mouse["pos"])
                ratones[python_mouse_index].set_target_position(world_pos)
                ratones[python_mouse_index].captured = False 
                
                is_powered = julia_mouse.get("powered_up", False)
                is_slowed = julia_mouse.get("slowed_down", False)
                ratones[python_mouse_index].powered_up = is_powered
                ratones[python_mouse_index].slowed_down = is_slowed
                
                # Muestra visualmente los poderes cuando un raton come el queso mágico
                if is_powered:
                    ratones[python_mouse_index].max_speed = 40.0 
                    ratones[python_mouse_index].interpolation_speed = 0.4 
                elif is_slowed:
                    ratones[python_mouse_index].max_speed = 10.0 
                    ratones[python_mouse_index].interpolation_speed = 0.1 
                else:
                    ratones[python_mouse_index].max_speed = 20.0
                    ratones[python_mouse_index].interpolation_speed = 0.2 
                
                alive_indices.append(python_mouse_index)
            else:
                pass  
        
        if simulation_initialized:
            for idx in previously_alive:
                if idx not in alive_indices:
                    if gato:
                        gato.trigger_eating()
                    
                    # Squeak squeak (ratones lloran cuando son comidos)
                    if squeak_sound:
                        squeak_sound.play()
                        
                    break 
                
        if len(julia_mice) == 0 and not game_over:
            game_over = True
            game_over_time = time.time()
    
    if "cat" in game_state and game_state["cat"] and gato:
        cat_data = game_state["cat"]
        world_pos = julia_to_python_coords(cat_data["pos"])
        gato.set_target_position(world_pos)
    
    return True

# Cargar JPEG como textura GL_RGB
def load_texture_jpeg(filepath, repeat=False):
    tex = glGenTextures(1)
    textures.append(tex)
    glBindTexture(GL_TEXTURE_2D, tex)
    
    if repeat:
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    else:
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
    image = pygame.image.load(filepath).convert()
    w, h = image.get_rect().size
    image_data = pygame.image.tostring(image, "RGB", 1)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data)
    glGenerateMipmap(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, 0)
    return tex

def drawSkyboxHouse(size=DimBoard):  
    if len(textures) == 0:
        return
    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_TEXTURE_BIT)
    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_TEXTURE_2D)

    glPushMatrix()
    glTranslatef(0.0, 0.0, 0.0)  

    glBindTexture(GL_TEXTURE_2D, textures[0])

    def quad(v1, v2, v3, v4):
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex3fv(v1)
        glTexCoord2f(1, 0); glVertex3fv(v2)
        glTexCoord2f(1, 1); glVertex3fv(v3)
        glTexCoord2f(0, 1); glVertex3fv(v4)
        glEnd()

    s = size / 2.0
    quad([ s, -s, -s], [ s, -s,  s], [ s,  s,  s], [ s,  s, -s])  # Derecha
    quad([-s, -s,  s], [-s, -s, -s], [-s,  s, -s], [-s,  s,  s])  # Izquierda
    quad([-s,  s, -s], [ s,  s, -s], [ s,  s,  s], [-s,  s,  s])  # Arriba
    quad([-s, -s,  s], [ s, -s,  s], [ s, -s, -s], [-s, -s, -s])  # Abajo
    quad([ s, -s,  s], [-s, -s,  s], [-s,  s,  s], [ s,  s,  s])  # Frente
    quad([-s, -s, -s], [ s, -s, -s], [ s,  s, -s], [-s,  s, -s])  # Atrás

    glPopMatrix()

    glBindTexture(GL_TEXTURE_2D, 0)
    glDisable(GL_TEXTURE_2D)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glPopAttrib()

def drawFloor():
    global floor_texture
    
    if floor_texture is not None:
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, floor_texture)
        glColor3f(1.0, 1.0, 1.0)  
        
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0); glVertex3d(-DimBoard, 0, -DimBoard)
        glTexCoord2f(6.0, 0.0); glVertex3d(DimBoard, 0, -DimBoard)  
        glTexCoord2f(6.0, 6.0); glVertex3d(DimBoard, 0, DimBoard)
        glTexCoord2f(0.0, 6.0); glVertex3d(-DimBoard, 0, DimBoard)
        glEnd()
        
        glBindTexture(GL_TEXTURE_2D, 0)
        glDisable(GL_TEXTURE_2D)
    else:
        glColor3f(0.3, 0.3, 0.3)
        glBegin(GL_QUADS)
        glVertex3d(-DimBoard, 0, -DimBoard)
        glVertex3d(-DimBoard, 0, DimBoard)
        glVertex3d(DimBoard, 0, DimBoard)
        glVertex3d(DimBoard, 0, -DimBoard)
        glEnd()

    glColor3f(1.0,1.0,1.0) 

def generar_escobas():
    global escobas
    escobas = Escoba.crear_escobas_predefinidas(dim_board=DimBoard, scale=35.0)

def wait_for_julia_connection(max_attempts=30):
    if not API_AVAILABLE:
        return False
        
    for attempt in range(max_attempts):
        try:
            response = requests.get(f"{API_BASE_URL}/game-state", timeout=1.0)
            if response.status_code == 200:
                game_state = response.json()
                has_mice = "mice" in game_state and len(game_state["mice"]) >= 1
                has_cat = "cat" in game_state and game_state["cat"] is not None
                has_bananas = "bananas" in game_state and len(game_state["bananas"]) > 0
                
                if has_mice and has_cat and has_bananas:
                    return True 
            
            time.sleep(0.5)
        except Exception:
            time.sleep(0.5)
            
    return False

def generar_quesos():
    global quesos
    quesos = []
    if not API_AVAILABLE:
        generar_quesos_localmente()

def generar_quesos_localmente():
    global quesos
    quesos = Queso.crear_quesos_predefinidos(dim_board=DimBoard, scale=1.0)

def generar_cajas():
    global cajas
    cajas = Caja.crear_cajas_predefinidas(dim_board=DimBoard, scale=20.0)

def generar_ratones():
    global ratones
    ratones = []
    
    # Obteniendo las direcciones de los obstáculos para colisiones
    obstacles = get_obstacle_list()
    
    # Creando los 4 ratones (inicialmente capturados/invisibles hasta que Julia diga dónde están)
    for i in range(4):  
        raton = Raton(dim_board=DimBoard, vel=1.0, scale=12.0)
        raton.Position = [0.0, 0.0, 0.0]
        raton.Direction = [0.0, 0.0, -1.0]
        raton.set_obstacles(obstacles)
        raton.captured = True 
        ratones.append(raton) 

def generar_gato():
    global gato
    
    obstacles = get_obstacle_list()
    
    gato = Gato(dim_board=DimBoard, vel=1.0, scale=3.0)
    gato.Position = [0.0, 0.0, 0.0]  
    gato.Direction = [0.0, 0.0, -1.0]
    gato.set_obstacles(obstacles)

def dibujar_escobas():
    for escoba in escobas:
        escoba.draw()

def dibujar_quesos():
    for queso in quesos:
        queso.draw()

def dibujar_cajas():
    for caja in cajas:
        caja.draw()

def dibujar_ratones():
    for raton in ratones:
        raton.draw()

def dibujar_gato():
    if gato:
        gato.draw()

def Init():
    global carro, clock, llantaTraseraRoll, last_api_update
    screen = pygame.display.set_mode((screen_width, screen_height), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("OpenGL: PROYECTO_RATON_GAME_")
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(FOVY, screen_width/screen_height, ZNEAR, ZFAR)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(EYE_X,EYE_Y,EYE_Z,CENTER_X,CENTER_Y,CENTER_Z,UP_X,UP_Y,UP_Z)

    # Clear color visible si skybox no carga - causa que el hideout sea blanco en vez de negro
    glClearColor(0.15,0.15,0.2,1.0)
    glEnable(GL_DEPTH_TEST)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    clock = pygame.time.Clock()
    last_api_update = time.time()
    
    llantaTraseraRoll = 0.0

    #genera los objetos en las posiciones
    generar_ratones()  # Generar los 4 ratones
    generar_escobas()
    generar_quesos()  
    generar_cajas()
    generar_gato()

   
    img = find_house_image()
    if img:
        try:
            load_texture_jpeg(img, repeat=False) 
        except Exception as e:
            pass
    
    global floor_texture
    floor_img = find_floor_image()
    if floor_img:
        try:
            floor_texture = load_texture_jpeg(floor_img, repeat=True) 
        except Exception as e:
            pass

# update cámara según theta y dir (igual que original)
def lookat():
    global EYE_X, EYE_Z, CENTER_X, CENTER_Z, dir, theta
    rads = math.radians(theta)
    dir_x = math.cos(rads)*dir[0] + math.sin(rads)*dir[2]
    dir_z = -math.sin(rads)*dir[0] + math.cos(rads)*dir[2]
    dir[0] = dir_x
    dir[2] = dir_z
    CENTER_X = EYE_X + dir[0]
    CENTER_Z = EYE_Z + dir[2]
    glLoadIdentity()
    gluLookAt(EYE_X, EYE_Y, EYE_Z, CENTER_X, CENTER_Y, CENTER_Z, UP_X, UP_Y, UP_Z)

def drawHideouts():
    """Dibujado de las madrigueras"""
    if Caja.modelo is None:
        return

    glDisable(GL_TEXTURE_2D)
    glColor3f(0.0, 0.0, 0.0) #color neutro
    
    # Madriguera 1: Back Wall Center (0, 0, -300) -> Corresponde al de Julia (1, 9)
    glPushMatrix()
    glTranslatef(0, 0, -300) 
    glScalef(12.0, 12.0, 12.0) # Same scale as mice
    Caja.modelo.render()
    glPopMatrix()
    
    # Madriguera 2: Front Wall Center (0, 0, 300) -> Corresponde al de Julia (14, 9)
    glPushMatrix()
    glTranslatef(0, 0, 300) 
    glScalef(12.0, 12.0, 12.0) 
    Caja.modelo.render()
    glPopMatrix()
    
    glColor3f(1.0, 1.0, 1.0) 
    glEnable(GL_TEXTURE_2D)

# Función display principal: dibuja skybox y luego la escena (ratones y otros objetos)
def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    drawSkyboxHouse() 
    # Dibujar el suelo con textura
    drawFloor()
    # Dibujar madrigueras
    drawHideouts()
    # Dibujar todos los objetos
    dibujar_ratones()  
    dibujar_escobas()
    dibujar_quesos()
    dibujar_cajas()
    dibujar_gato()

def draw_connection_status():
    if simulation_initialized:
        return  
        
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, screen_width, 0, screen_height, -1, 1)
    
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()
    
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
    # CAJA
    glColor4f(0.8, 0.4, 0.0, 0.8) 
    margin = 20
    box_width = 200
    box_height = 60
    glBegin(GL_QUADS)
    glVertex2f(screen_width - margin - box_width, screen_height - margin - box_height)
    glVertex2f(screen_width - margin, screen_height - margin - box_height)
    glVertex2f(screen_width - margin, screen_height - margin)
    glVertex2f(screen_width - margin - box_width, screen_height - margin)
    glEnd()
    
    # icono de espera para indicar tiempo de conexion con julia [ASPECTO VISUAL] 
    dot_phase = int(time.time() * 3) % 3 
    glColor3f(1.0, 1.0, 1.0)
    for i in range(3):
        if i <= dot_phase:
            dot_x = screen_width - margin - box_width + 50 + i * 20
            dot_y = screen_height - margin - 30
            glBegin(GL_QUADS)
            glVertex2f(dot_x - 3, dot_y - 3)
            glVertex2f(dot_x + 3, dot_y - 3)
            glVertex2f(dot_x + 3, dot_y + 3)
            glVertex2f(dot_x - 3, dot_y + 3)
            glEnd()
    
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glColor3f(1.0, 1.0, 1.0)
    
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

# MAIN
done = False
debug_collision = False  # depuración para las colisiones

Init()

# SINCRONIZACION DE JULIA - MEJORAS
if API_AVAILABLE:
    julia_client = JuliaClient(API_BASE_URL)
    julia_client.start()

while not done:
    keys = pygame.key.get_pressed()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c: 
                debug_collision = not debug_collision
    
    if game_over and time.time() - game_over_time >= GAME_OVER_DELAY:
        done = True
    
    for raton in ratones:
        raton.update()
        raton.Position[1] = 0.0  
        
    if gato:
        gato.update()
    
    if not game_over:
        # Integracion pacman y webapi (MEJORADA)
        if API_AVAILABLE and julia_client:
            try:
                active_mice = len([r for r in ratones if not r.captured])
                
                should_step = simulation_initialized and active_mice >= 1
                julia_client.set_stepping(should_step)
                
                was_initialized = simulation_initialized
                
                if update_from_julia_simulation():
                    if simulation_initialized and not was_initialized:
                        active_mice = len([r for r in ratones if not r.captured])
            except Exception as e:
                pass
    
    glLoadIdentity()
    gluLookAt(EYE_X, EYE_Y, EYE_Z, CENTER_X, CENTER_Y, CENTER_Z, UP_X, UP_Y, UP_Z)

    display()
    
    if debug_collision:
        draw_collision_debug()
    
    if API_AVAILABLE and not simulation_initialized:
        draw_connection_status()
    
    pygame.display.flip()
    clock.tick(60)

if julia_client:
    julia_client.stop()

pygame.quit()


 