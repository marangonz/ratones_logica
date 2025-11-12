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

class Raton:
    modelo = None
    patastraseras = None
    patasdelanteras = None
    cabeza = None
    cola = None

    def __init__(self, dim_board=450, vel=0.5, scale=1.0):
        self.DimBoard = dim_board
        self.dimension = dim_board
        self.velocity = vel
        self.scale = scale
        self.llantastraseras = 0.0  # Ángulo de rotación de las llantas traseras
        self.radio_llanta = 1.0
        self.distancia_recorrida = 0.0
        self.llantasdelanteras = 0.0  # Ángulo de rotación de las llantas delanteras
        self.angulo_direccion = 0.0  # Ángulo de dirección de las llantas delanteras
        self.anim_angle = 0.0  # ángulo base de animación de patas
        self.anim_speed = 10.0

        # Nueva variable: controla si el ratón está avanzando
        self.en_movimiento = False
        
        if Raton.modelo is None:
            Raton.modelo = OBJ("raton/mouse_body.obj", swapyz=False)
            Raton.modelo.generate()
        if Raton.patastraseras is None:
            # corregido: cargar patas traseras en la variable patastraseras
            Raton.patastraseras = OBJ("raton/patas_traseras.obj", swapyz=False)
            Raton.patastraseras.generate()
        if Raton.patasdelanteras is None:
            # corregido: cargar patas delanteras en la variable patasdelanteras
            Raton.patasdelanteras = OBJ("raton/patas_delanteras.obj", swapyz=False)
            Raton.patasdelanteras.generate()
        if Raton.cabeza is None:
            Raton.cabeza = OBJ("raton/mouse_head.obj", swapyz=False)
            Raton.cabeza.generate()
        if Raton.cola is None:
            Raton.cola = OBJ("raton/mouse_tail.obj", swapyz=False)
            Raton.cola.generate()

        # inicializa posición
        self.Position = [
            random.randint(-1 * self.DimBoard, self.DimBoard),
            0.0,  # Posición Y en el piso
            random.randint(-1 * self.DimBoard, self.DimBoard)
        ]

        # vector dirección en XZ (y no se usa para avanzar)
        dx = random.uniform(-1.0, 1.0)
        dz = random.uniform(-1.0, 1.0)
        m = math.sqrt(dx*dx + dz*dz) or 1.0
        self.Direction = [dx / m, 0.0, dz / m]

    def update(self):
        if self.en_movimiento:
            self.anim_angle += self.anim_speed
        if self.anim_angle > 360:
            self.anim_angle -= 360
        # Asegurar que el ratón siempre esté en el piso
        self.Position[1] = 0.0
                
    def move_forward(self):
        self.en_movimiento = True
        dx = self.Direction[0] * self.velocity
        dz = self.Direction[2] * self.velocity
        self.Position[0] += dx
        self.Position[2] += dz
        # Mantener al ratón siempre en el piso
        self.Position[1] = 0.0
        avance = math.sqrt(dx*dx + dz*dz)
        self.distancia_recorrida += avance
        vueltas = self.distancia_recorrida / (2 * math.pi * self.radio_llanta)
        self.llantastraseras = (vueltas * 360) % 360

    def move_backward(self):
        self.en_movimiento = True
        dx = self.Direction[0] * self.velocity
        dz = self.Direction[2] * self.velocity
        # mover en sentido opuesto
        self.Position[0] -= dx
        self.Position[2] -= dz
        # Mantener al ratón siempre en el piso
        self.Position[1] = 0.0
        avance = math.sqrt(dx*dx + dz*dz)
        self.distancia_recorrida += avance
        vueltas = self.distancia_recorrida / (2 * math.pi * self.radio_llanta)
        self.llantastraseras = (vueltas * 360) % 360

    def turn(self, angle_radians):
        # actualizar angulo de cabeza en grados 
        self.angulo_direccion += math.degrees(angle_radians)
        self.angulo_direccion = max(-45.0, min(45.0, self.angulo_direccion))

        # rotación correcta alrededor de Y para (x,z):
        c = math.cos(angle_radians)
        s = math.sin(angle_radians)
        dx = self.Direction[0]
        dz = self.Direction[2]
        # matriz de rotación Y: x' = c*x + s*z ; z' = -s*x + c*z
        new_x = c * dx + s * dz
        new_z = -s * dx + c * dz
        # normaliza para evitar errores por acumulación
        norm = math.hypot(new_x, new_z) or 1.0
        self.Direction[0] = new_x / norm
        self.Direction[2] = new_z / norm
        
    def centrar_direccion(self, velocidad_centrado=1.0):
        # velocidad de centrado por llamada; idealmente multiplica por delta time en el main loop
        if self.angulo_direccion > 0:
            self.angulo_direccion -= velocidad_centrado
            if self.angulo_direccion < 0:
                self.angulo_direccion = 0
        elif self.angulo_direccion < 0:
            self.angulo_direccion += velocidad_centrado
            if self.angulo_direccion > 0:
                self.angulo_direccion = 0

    def draw(self):
      
        def mat_body(tx, ty, tz, sx, sy, sz, theta_rad):
            c = math.cos(theta_rad)
            s = math.sin(theta_rad)
            M = np.array([
                [ c * sx,    0.0,      s * sx,   tx],
                [ 0.0,       sy,       0.0,      ty],
                [-s * sx,    0.0,      c * sx,   tz],
                [ 0.0,       0.0,      0.0,      1.0]
            ], dtype=np.float32)
            return M

        def mat_T_Ry(tx, ty, tz, theta_rad):
            c = math.cos(theta_rad)
            s = math.sin(theta_rad)
            M = np.array([
                [ c,   0.0,  s,   tx],
                [ 0.0, 1.0,  0.0, ty],
                [-s,   0.0,  c,   tz],
                [ 0.0, 0.0,  0.0, 1.0]
            ], dtype=np.float32)
            return M

        def mat_T_Rx(tx, ty, tz, theta_rad):
            c = math.cos(theta_rad)
            s = math.sin(theta_rad)
            M = np.array([
                [1.0, 0.0, 0.0, tx],
                [0.0,  c,  -s,  ty],
                [0.0,  s,   c,  tz],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float32)
            return M

        angle = math.degrees(math.atan2(-self.Direction[0], -self.Direction[2]))
        theta = math.radians(angle)
        sx = sy = sz = self.scale
        tx, ty, tz = self.Position
       
        # animaciones
        if self.en_movimiento:
            self.anim_angle += self.anim_speed
        if self.anim_angle > 360.0:
            self.anim_angle -= 360.0
        movimiento_pata = math.sin(math.radians(self.anim_angle)) * 15.0
        movimiento_cola = math.sin(math.radians(self.anim_angle * 2.0)) * 10.0 if self.en_movimiento else 0.0

        head_offset = (0.0, 0.0, 0.0)       
        patas_tras_offset = (0.0, 0.0, 0.0)
        patas_del_offset = (0.0, 0.0,  0.0)
        cola_offset = (0.0, 0.0, 0.0)
        

        # cuerpo: M_world = T * Ry * S  
        M_world = mat_body(tx, ty, tz, sx, sy, sz, theta)

        glPushMatrix()
        glMultMatrixf(M_world.T)
        Raton.modelo.render()

        # cabeza = T(head_offset) * Ry(self.angulo_direccion) 
        local_head = mat_T_Ry(head_offset[0], head_offset[1], head_offset[2],
                            math.radians(self.angulo_direccion))
        glPushMatrix()
        glMultMatrixf(local_head.T)
        Raton.cabeza.render()
        glPopMatrix()

        # patas traseras = T(patas_tras_offset) * Rx(movimiento_pata) 
        local_ptras = mat_T_Rx(patas_tras_offset[0], patas_tras_offset[1], patas_tras_offset[2],
                            math.radians(movimiento_pata))
        glPushMatrix()
        glMultMatrixf(local_ptras.T)
        Raton.patastraseras.render()
        glPopMatrix()

        # patas delanteras: local = T(patas_del_offset) * Rx(-movimiento_pata)
        local_pdel = mat_T_Rx(patas_del_offset[0], patas_del_offset[1], patas_del_offset[2],
                            math.radians(-movimiento_pata))
        glPushMatrix()
        glMultMatrixf(local_pdel.T)
        Raton.patasdelanteras.render()
        glPopMatrix()

        # cola = T(cola_offset) * Ry(movimiento_cola)
        local_cola = mat_T_Ry(cola_offset[0], cola_offset[1], cola_offset[2],
                            math.radians(movimiento_cola))
        glPushMatrix()
        glMultMatrixf(local_cola.T)
        Raton.cola.render()
        glPopMatrix()

        glPopMatrix()