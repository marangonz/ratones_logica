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

class Caja:
    modelo = None

    def __init__(self, dim_board=450, vel=0.5, scale=1.0, position=None):
        self.DimBoard = dim_board
        self.dimension = dim_board
        self.scale = scale

        self.en_movimiento = False
        
        if Caja.modelo is None:
            Caja.modelo = OBJ("Textures/Box/box.obj", swapyz=False)
            Caja.modelo.generate()

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
        
        if Caja.modelo:
            Caja.modelo.render()
        glPopMatrix()
    
    @staticmethod
    def crear_cajas_predefinidas(dim_board=400, scale=20.0):
        posiciones_cajas = [
            [-100, 0, -300],  # Caja 1: esquina trasera-izquierda
            [150, 0, -250],   # Caja 2: esquina trasera-derecha
            [-100, 5, 300],   # Caja 3: esquina delantera-izquierda
            [200, 0, 250]     # Caja 4: esquina delantera-derecha
        ]
        
        cajas = []
        for i, posicion in enumerate(posiciones_cajas):
            caja = Caja(dim_board=dim_board, scale=scale, position=posicion)
            cajas.append(caja)
     #       print(f"Caja {i+1} creada en posición: ({posicion[0]}, {posicion[1]}, {posicion[2]})")
        
        return cajas