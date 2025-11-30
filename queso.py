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

class Queso:
    modelo = None

    def __init__(self, dim_board=450, vel=0.5, scale=1.0, position=None, is_magic=False, cheese_type='normal'):
        self.DimBoard = dim_board
        self.dimension = dim_board
        self.scale = scale
        self.is_magic = is_magic
        self.cheese_type = cheese_type

        if is_magic and cheese_type == 'normal':
            self.cheese_type = 'magic'

        self.en_movimiento = False
        
        if Queso.modelo is None:
            Queso.modelo = OBJ("Textures/Cheese/cheesetriangle.obj", swapyz=False)
            Queso.modelo.generate()

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
        
        if self.cheese_type == 'magic':
            glDisable(GL_TEXTURE_2D)
            glColor3f(0.0, 0.5, 1.0) 
        elif self.cheese_type == 'green':
            glDisable(GL_TEXTURE_2D)
            glColor3f(0.0, 1.0, 0.0)
        
        if Queso.modelo:
            Queso.modelo.render()
            
        if self.cheese_type in ['magic', 'green']:
            glColor3f(1.0, 1.0, 1.0)
            glEnable(GL_TEXTURE_2D)
            
        glPopMatrix()
    
    @staticmethod
    def crear_quesos_predefinidos(dim_board=400, scale=1.0):
        """Crea una lista de 7 quesos en posiciones predefinidas"""
        posiciones_quesos = [
            [-300, 0, -200],  # Queso 1: esquina lejana izquierda-atrás
            [300, 0, -300],   # Queso 2: esquina lejana derecha-atrás
            [-250, 0, 250],   # Queso 3: esquina lejana izquierda-adelante
            [250, 0, 300],    # Queso 4: esquina lejana derecha-adelante
            [0, 0, 350],      # Queso 5: centro-adelante
            [-350, 0, 0],     # Queso 6: centro-izquierda
            [350, 0, 50]      # Queso 7: centro-derecha
        ]
        
        quesos = []
        for i, posicion in enumerate(posiciones_quesos):
            queso = Queso(dim_board=dim_board, scale=scale, position=posicion)
            quesos.append(queso)
            print(f"Queso {i+1} creado en posición: ({posicion[0]}, {posicion[1]}, {posicion[2]})")
        
        return quesos
