# Autor: Ivan Olmos Pineda (modificado)
# Curso: Multiagentes - Graficas Computacionales


import os
import pygame
from pygame.locals import *

# OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import math
import sys
sys.path.append('..')

from raton import Raton
from escoba import Escoba
from queso import Queso
from caja import Caja
from gato import Gato
from objloader import *
import random

# Ventana
screen_width = 1200
screen_height = 800

# Cámara / proyección - Ajustada para vista panorámica óptima
FOVY = 85.0      # Campo de visión más amplio para ver más área
ZNEAR = 1.0
ZFAR = 1500.0    # Mayor distancia para objetos lejanos

# POSICIÓN inicial de la cámara: Optimizada para vista panorámica completa
EYE_X = 0.0      # Centrada en X para vista balanceada
EYE_Y = 400.0    # Más elevada para vista aérea completa del escenario
EYE_Z = 200.0    # Posición trasera óptima para ver todo el panorama

# La cámara mira hacia el CENTRO del escenario con ligera inclinación hacia abajo
CENTER_X = 0.0   # Centro del mundo (donde está el ratón)
CENTER_Y = -50.0 # Ligeramente hacia abajo para mejor vista del piso
CENTER_Z = 0.0   # Centro del mundo (donde está el ratón)

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
DimBoard = 600   # tamaño del suelo y skybox más grande (para referencia antes era 400)
CUBE_SIZE = 1200  # skybox del mismo tamaño que el área del piso (DimBoard * 2)

# Movimiento/cámara
dir = [1.0, 0.0, 0.0]
theta = 0.0

pygame.init()

# Texturas global
textures = []
floor_texture = None

# Listas de objetos
escobas = []
quesos = []
cajas = []
ratones = []  # Lista de ratones
gato = None
NUM_ESCOBAS = 5  # Número de escobas a generar
NUM_QUESOS = 7   # Número de quesos a generar
NUM_CAJAS = 4    # Número de cajas a generar
NUM_RATONES = 4  # Número total de ratones (incluyendo el principal)

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

#def generar_quesos(): comentareado porque los quesos los genera julia
#    global quesos
#    quesos = Queso.crear_quesos_predefinidos(dim_board=DimBoard, scale=1.0)

def generar_cajas():
    global cajas
    cajas = Caja.crear_cajas_predefinidas(dim_board=DimBoard, scale=20.0)

def generar_ratones():
    global ratones
    ratones = []
    
    # Posiciones predefinidas para los 4 ratones
    posiciones_ratones = [
        [0.0, 0.0, 0.0],      # Ratón principal (centro)
        [-200.0, 0.0, -200.0], # Ratón esquina superior izquierda
        [200.0, 0.0, -200.0],  # Ratón esquina superior derecha
        [0.0, 0.0, 300.0]      # Ratón parte inferior centro
    ]
    
    # Direcciones iniciales para cada ratón
    direcciones_ratones = [
        [0.0, 0.0, -1.0],  # Mirando hacia adelante
        [1.0, 0.0, 0.0],   # Mirando hacia la derecha
        [-1.0, 0.0, 0.0],  # Mirando hacia la izquierda
        [0.0, 0.0, -1.0]   # Mirando hacia adelante
    ]
    
    for i in range(NUM_RATONES):
        raton = Raton(dim_board=DimBoard, vel=1.0, scale=12.0)
        raton.Position = posiciones_ratones[i].copy()
        raton.Direction = direcciones_ratones[i].copy()
        ratones.append(raton)
        print(f"Ratón {i+1} generado en posición: {raton.Position}")

def generar_gato():
    global gato
    gato = Gato(dim_board=DimBoard, vel=1.0, scale=3.0)
    gato.Position = [50.0, 0.0, 0.0]  
    gato.Direction = [0.0, 0.0, -1.0]  
    print(f"Posición inicial del gato: {gato.Position}")

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
    global carro, clock, llantaTraseraRoll
    screen = pygame.display.set_mode((screen_width, screen_height), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("OpenGL: PROYECTO_RATON_GAME_")
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(FOVY, screen_width/screen_height, ZNEAR, ZFAR)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(EYE_X,EYE_Y,EYE_Z,CENTER_X,CENTER_Y,CENTER_Z,UP_X,UP_Y,UP_Z)

    # Clear color visible si skybox no carga
    glClearColor(0.15,0.15,0.2,1.0)
    glEnable(GL_DEPTH_TEST)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    clock = pygame.time.Clock()

    
    llantaTraseraRoll = 0.0

    #genera los objetos en las posiciones
    generar_ratones()  # Generar los 4 ratones
    generar_escobas()
   # generar_quesos()
    generar_cajas()
    generar_gato()

   
    img = find_house_image()
    if img:
        try:
            load_texture_jpeg(img, repeat=False)  # Skybox no necesita repetir
            print("Skybox: cargada imagen de casa ->", img)
        except Exception as e:
            print("Error cargando la imagen del skybox:", e)
    else:
        print("No se encontró imagen de casa (house.jpg/jpeg o casa.jpg/jpeg) en:", base_path)
        print("Coloca la imagen en la carpeta del script o modifica find_house_image().")
    
    global floor_texture
    floor_img = find_floor_image()
    if floor_img:
        try:
            floor_texture = load_texture_jpeg(floor_img, repeat=True)  # Piso sí necesita repetir
            print("Piso: cargada textura ->", floor_img)
        except Exception as e:
            print("Error cargando la textura del piso:", e)
    else:
        print("No se encontró imagen para el piso. Se usará color sólido.")

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

# Función display principal: dibuja skybox y luego la escena (ratones y otros objetos)
def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # Dibujar cubo/skybox exactamente del mismo tamaño que el piso
    drawSkyboxHouse()  # Usa DimBoard por defecto
    # Dibujar el suelo con textura
    drawFloor()
    # Dibujar todos los objetos
    dibujar_ratones()  # Dibujar todos los ratones
    dibujar_escobas()
    dibujar_quesos()
    dibujar_cajas()
    dibujar_gato()

# MAIN
done = False
Init()

while not done:
    keys = pygame.key.get_pressed()
    
    # Controles del ratón principal (primer ratón de la lista)
    if len(ratones) > 0:
        raton_principal = ratones[0]
        raton_principal.en_movimiento = False
        
        if keys[pygame.K_UP]:
            raton_principal.move_forward()
            raton_principal.en_movimiento = True
        if keys[pygame.K_DOWN]:
            raton_principal.move_backward()
            raton_principal.en_movimiento = True
        if keys[pygame.K_LEFT]:
            raton_principal.turn(math.radians(5.0))
        if keys[pygame.K_RIGHT]:
            raton_principal.turn(math.radians(-5.0))
        if not keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            raton_principal.centrar_direccion()
    
    # Controles WASD para el gato
    if gato:
        gato_moving = False
        if keys[pygame.K_w]:
            gato.move_forward()
            gato_moving = True
            gato.moving_direction = 1
        if keys[pygame.K_s]:
            gato.move_backward()
            gato_moving = True
            gato.moving_direction = -1
        if keys[pygame.K_a]:
            gato.turn(math.radians(5.0))
        if keys[pygame.K_d]:
            gato.turn(math.radians(-5.0))
        
        gato.en_movimiento = gato_moving
        if not gato_moving:
            gato.moving_direction = 0
        
        if not gato_moving:
            gato.centrar_direccion()
            
#    camera_speed = 8.0
    # Q/E: Subir/bajar altura de cámara
#    if keys[pygame.K_q]:
#        EYE_Y += camera_speed
#    if keys[pygame.K_e]:
#        EYE_Y -= camera_speed
    
#    # I/K: Acercar/alejar cámara del centro
#    if keys[pygame.K_i]:
#        EYE_Z -= camera_speed
#    if keys[pygame.K_k]:
#        EYE_Z += camera_speed
    
    # J/L: Mover cámara izquierda/derecha
#    if keys[pygame.K_j]:
#        EYE_X -= camera_speed
#    if keys[pygame.K_l]:
#        EYE_X += camera_speed
    
    # U/O: Ajustar inclinación de la vista (hacia arriba/abajo)
 #   if keys[pygame.K_u]:
 #       CENTER_Y += 2.0
 #   if keys[pygame.K_o]:
 #       CENTER_Y -= 2.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    # Actualizar todos los ratones
    for raton in ratones:
        raton.update()
        raton.Position[1] = 0.0  # Mantener en el piso
    
    if gato:
        gato.update()
    
    glLoadIdentity()
    gluLookAt(EYE_X, EYE_Y, EYE_Z, CENTER_X, CENTER_Y, CENTER_Z, UP_X, UP_Y, UP_Z)

    display() #MANDAR UN MENSAJE POR SEGUNDO 
    dibujar_ratones()  # Dibujar todos los ratones
    pygame.display.flip()
    pygame.time.wait(10)

pygame.quit()


 