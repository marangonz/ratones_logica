import pygame
from pygame.locals import *

# Cargamos las bibliotecas de OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# importar clase obj
from objloader import OBJ

import numpy as np
import random
import math

class Escoba:
    modelo = None

    def __init__(self, dim_board=450, vel=0.5, scale=1.0, position=None):
        self.DimBoard = dim_board
        self.dimension = dim_board
        self.scale = scale

        self.en_movimiento = False
        
        if Escoba.modelo is None:
            Escoba.modelo = OBJ("Textures/Broom/broom_01.obj", swapyz=False)
            Escoba.modelo.generate()
      
        # Si se proporciona una posición específica, usarla; si no, posición aleatoria
        if position is not None:
            self.Position = position
        else:
            self.Position = [
                random.randint(-1 * self.DimBoard, self.DimBoard),
                0.0,  # Siempre en el piso
                random.randint(-1 * self.DimBoard, self.DimBoard)
            ]

    def draw(self):
        glPushMatrix()
        glTranslatef(self.Position[0], self.Position[1], self.Position[2])
        glScalef(self.scale, self.scale, self.scale)
        
        if Escoba.modelo:
            Escoba.modelo.render()
        glPopMatrix()
    
    @staticmethod
    def crear_escobas_predefinidas(dim_board=400, scale=10.0):
        posiciones_escobas = [
            [-150, 0, -100],  # Escoba 1: esquina izquierda-trasera
            [200, 0, -150],   # Escoba 2: esquina derecha-trasera  
            [-200, 0, 150],   # Escoba 3: esquina izquierda-delantera
            [100, 0, 200],    # Escoba 4: esquina derecha-delantera
            [0, 0, -250]      # Escoba 5: centro-trasero
        ]
        
        escobas = []
        for i, posicion in enumerate(posiciones_escobas):
            escoba = Escoba(dim_board=dim_board, scale=scale, position=posicion)
            escobas.append(escoba)
            print(f"Escoba {i+1} creada en posición: ({posicion[0]}, {posicion[1]}, {posicion[2]})")
        
        return escobas