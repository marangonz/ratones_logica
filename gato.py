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

class Gato:
    cuerpo = None
    cabeza = None
    cola = None
    pata_delantera_derecha = None
    pata_delantera_izquierda = None
    pata_trasera_derecha = None
    pata_trasera_izquierda = None
    distancia_recorrida = 0.0
    radio_llanta = 1.0

    def __init__(self, dim_board=450, vel=0.7, scale=1.0):
        self.DimBoard = dim_board
        self.dimension = dim_board
        self.velocity = vel
        self.scale = scale
        
        # Collision detection properties
        self.collision_radius = 25.0  # Cat collision radius
        
        # Animaciones
        self.anim_angle = 0.0  # ángulo base de animación
        self.anim_speed = 22.0  # Ultra-fast animation for maximum fluidity 
        self.cabeza_anim = 0.0  # animación específica de cabeza
        
        # Control de movimiento
        self.en_movimiento = False
    # Dirección de movimiento: 1 = adelante, -1 = atrás, 0 = parado
        self.moving_direction = 0
        self.angulo_direccion = 0.0  # Ángulo de dirección de la cabeza
        
        # Variables para animaciones más complejas
        self.tiempo_idle = 0.0  # tiempo sin moverse para animaciones idle
        self.fase_patas = 0.0   # desfase entre patas delanteras y traseras, para que se intentara ver mas natural pero no funciono del todo
        
        if Gato.cuerpo is None:
            Gato.cuerpo = OBJ("gato/body.obj", swapyz=False)
            Gato.cuerpo.generate()
        if Gato.cabeza is None:
            Gato.cabeza = OBJ("gato/head.obj", swapyz=False)
            Gato.cabeza.generate()
        if Gato.cola is None:
            Gato.cola = OBJ("gato/cola_gato.obj", swapyz=False)
            Gato.cola.generate()
        if Gato.pata_delantera_derecha is None:
            Gato.pata_delantera_derecha = OBJ("gato/p_d_derecha.obj", swapyz=False)
            Gato.pata_delantera_derecha.generate()
        if Gato.pata_delantera_izquierda is None:
            Gato.pata_delantera_izquierda = OBJ("gato/p_d_izquierda.obj", swapyz=False)
            Gato.pata_delantera_izquierda.generate()
        if Gato.pata_trasera_derecha is None:
            Gato.pata_trasera_derecha = OBJ("gato/p__t_derecha.obj", swapyz=False)
            Gato.pata_trasera_derecha.generate()
        if Gato.pata_trasera_izquierda is None:
            Gato.pata_trasera_izquierda = OBJ("gato/pata_trasera_izquierda.obj", swapyz=False)
            Gato.pata_trasera_izquierda.generate()
       

        # Posición inicial específica (no aleatoria)
        self.Position = [100.0, 0.0, 100.0]  # Posición fija para diferenciar del ratón

        # Smooth movement for simulation-controlled cat
        self.target_position = self.Position.copy()
        self.interpolation_speed = 0.95  # Near-instant movement for aggressive cat behavior
        
        # Collision system
        self.obstacles = []  # Will be set by main.py

        # Dirección inicial
        self.Direction = [0.0, 0.0, -1.0]

    def set_target_position(self, target_pos):
        """Set target position for smooth simulation movement"""
        self.target_position = target_pos.copy()
    
    def set_obstacles(self, obstacles):
        """Set obstacle list for collision detection"""
        self.obstacles = obstacles
    
    def check_collision(self, position):
        """Check if a position would cause collision with obstacles"""
        for obstacle in self.obstacles:
            dx = position[0] - obstacle.position[0]
            dz = position[2] - obstacle.position[2]
            distance = math.sqrt(dx*dx + dz*dz)
            
            if distance < (self.collision_radius + obstacle.radius):
                return True
        return False
    
    def find_safe_move(self, target_pos):
        """Find a safe position to move towards that avoids obstacles"""
        if not self.check_collision(target_pos):
            return target_pos
        
        # If direct path is blocked, try alternative paths
        current_x, current_y, current_z = self.Position
        target_x, target_y, target_z = target_pos
        
        # Try moving around obstacles by testing different angles
        for angle_offset in [math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, 3*math.pi/4, -3*math.pi/4]:
            # Calculate direction to target
            dx = target_x - current_x
            dz = target_z - current_z
            distance = math.sqrt(dx*dx + dz*dz)
            
            if distance > 0:
                # Apply rotation to direction
                rotated_dx = dx * math.cos(angle_offset) - dz * math.sin(angle_offset)
                rotated_dz = dx * math.sin(angle_offset) + dz * math.cos(angle_offset)
                
                # Move a small step in the rotated direction
                step_size = 30.0  # Adjusted step size for navigation
                new_x = current_x + (rotated_dx / distance) * step_size
                new_z = current_z + (rotated_dz / distance) * step_size
                
                test_pos = [new_x, current_y, new_z]
                
                # Keep within bounds
                test_pos[0] = max(-300, min(300, test_pos[0]))
                test_pos[2] = max(-300, min(300, test_pos[2]))
                
                if not self.check_collision(test_pos):
                    return test_pos
        
        # If no safe path found, stay in place
        return self.Position.copy()

    def update(self):
        # Handle simulation smooth movement interpolation with collision detection
        
        # Find safe target position (avoiding obstacles)
        safe_target = self.find_safe_move(self.target_position)
        
        # Calculate distance to safe target
        dx = safe_target[0] - self.Position[0]
        dz = safe_target[2] - self.Position[2]
        distance = math.sqrt(dx*dx + dz*dz)
        
        if distance > 0.1:  # Minimal threshold for precise positioning
            # Calculate next position with interpolation
            next_x = self.Position[0] + dx * self.interpolation_speed
            next_z = self.Position[2] + dz * self.interpolation_speed
            next_pos = [next_x, self.Position[1], next_z]
            
            # Double-check that next position is safe
            if not self.check_collision(next_pos):
                self.Position[0] = next_x
                self.Position[2] = next_z
                
                # Update direction to face movement direction
                norm = math.sqrt(dx*dx + dz*dz) or 1.0
                self.Direction[0] = dx / norm
                self.Direction[2] = dz / norm
                
                self.en_movimiento = True
            else:
                self.en_movimiento = False
        else:
            self.en_movimiento = False
        
        # Animation updates
        if self.en_movimiento:
            # Animaciones de caminar - similar al ratón
            self.anim_angle += self.anim_speed
            self.fase_patas += self.anim_speed * 0.8  # Desfase para patas
            self.cabeza_anim += self.anim_speed * 0.6  # Cabeza más suave
            self.tiempo_idle = 0.0
        else:
            # Animaciones idle (cuando está parado)
            self.tiempo_idle += 1.0
            self.cabeza_anim += 2.0  # Movimiento suave de cabeza
            
        # Mantener ángulos en rango
        if self.anim_angle > 360:
            self.anim_angle -= 360
        if self.fase_patas > 360:
            self.fase_patas -= 360
        if self.cabeza_anim > 360:
            self.cabeza_anim -= 360
        
        # Mantener siempre en el piso y dentro de límites de cámara
        self.Position[1] = 0.0
        # Keep within camera bounds (±300)
        self.Position[0] = max(-300, min(300, self.Position[0]))
        self.Position[2] = max(-300, min(300, self.Position[2]))
                
    def move_forward(self):
        dx = self.Direction[0] * self.velocity
        dz = self.Direction[2] * self.velocity
        self.Position[0] += dx
        self.Position[2] += dz
        # Mantener al gato siempre en el piso
        self.Position[1] = 0.0
        avance = math.sqrt(dx*dx + dz*dz)
        self.distancia_recorrida += avance
        vueltas = self.distancia_recorrida / (2 * math.pi * self.radio_llanta)
        self.llantastraseras = (vueltas * 360) % 360

    def move_backward(self):
        dx = self.Direction[0] * self.velocity
        dz = self.Direction[2] * self.velocity
        # mover en sentido opuesto
        self.Position[0] -= dx
        self.Position[2] -= dz
        # Mantener al gato siempre en el piso
        self.Position[1] = 0.0
        avance = math.sqrt(dx*dx + dz*dz)
        self.distancia_recorrida += avance
        vueltas = self.distancia_recorrida / (2 * math.pi * self.radio_llanta)
        self.llantastraseras = (vueltas * 360) % 360

        
    def centrar_direccion(self, velocidad_centrado=1.0):
        if self.angulo_direccion > 0:
            self.angulo_direccion -= velocidad_centrado
            if self.angulo_direccion < 0:
                self.angulo_direccion = 0
        elif self.angulo_direccion < 0:
            self.angulo_direccion += velocidad_centrado
            if self.angulo_direccion > 0:
                self.angulo_direccion = 0

    def turn(self, angle_radians):
        # actualizar angulo de cabeza en grados (similar al ratoncito)
        self.angulo_direccion += math.degrees(angle_radians)
        self.angulo_direccion = max(-6.0, min(6.0, self.angulo_direccion))  # Reducido de -30/30 a -15/15, se supone que eso ayuda

        # Rotar vector de dirección
        c = math.cos(angle_radians)
        s = math.sin(angle_radians)
        dx = self.Direction[0]
        dz = self.Direction[2]
        new_x = c * dx + s * dz
        new_z = -s * dx + c * dz
        norm = math.hypot(new_x, new_z) or 1.0
        self.Direction[0] = new_x / norm
        self.Direction[2] = new_z / norm
        
    def centrar_direccion(self, velocidad_centrado=2.0): 
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
                [ sx * c,    0.0,    sz * s,   tx],
                [ 0.0,       sy,     0.0,      ty],
                [-sx * s,    0.0,    sz * c,   tz],
                [ 0.0,       0.0,    0.0,      1.0]
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

        def mat_Rx(theta_rad):
            c = math.cos(theta_rad)
            s = math.sin(theta_rad)
            M = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0,  c,  -s,  0.0],
                [0.0,  s,   c,  0.0],
                [0.0, 0.0, 0.0, 1.0]
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

        angle = math.degrees(math.atan2(self.Direction[0], self.Direction[2]))
        theta = math.radians(angle)
        sx = sy = sz = self.scale
        tx, ty, tz = self.Position

        # animaciones
        if self.en_movimiento:
            self.anim_angle += self.anim_speed
        if self.anim_angle > 360.0:
            self.anim_angle -= 360.0
        movimiento_pata = math.sin(math.radians(self.anim_angle)) * 8.0  # amplitud menor para gato
        movimiento_cola = math.sin(math.radians(self.anim_angle * 1.5)) * 5.0 if self.en_movimiento else 0.0
        cabeza_caminar = math.sin(math.radians(self.cabeza_anim)) * 1.0

        head_offset = (0.0, 0.0, 0.0)        
        patas_del_derecha_offset = (0.0, 0.0, 0.0)
        patas_del_izquierda_offset = (0.0, 0.0, 0.0)
        patas_tras_derecha_offset = (0.0, 0.0, 0.0)
        patas_tras_izquierda_offset = (0.0, 0.0, 0.0)
        cola_offset = (0.0, 0.0, 0.0)

        # cuerpo: M_world = T * Ry * S  
        M_world = mat_body(tx, ty, tz, sx, sy, sz, theta)

        glPushMatrix()
        glMultMatrixf(M_world.T)
        Gato.cuerpo.render()

        # cabeza= T(head_offset) * Ry(self.angulo_direccion) * Rx(cabeza_caminar)
        local_head = mat_T_Ry(head_offset[0], head_offset[1], head_offset[2],
                              math.radians(self.angulo_direccion)) @ mat_Rx(math.radians(cabeza_caminar))
        glPushMatrix()
        glMultMatrixf(local_head.T)
        Gato.cabeza.render()
        glPopMatrix()
        
        
        #pata delantera derecha
        local_p_dd = mat_T_Rx(patas_del_derecha_offset[0], patas_del_derecha_offset[1], patas_del_derecha_offset[2],
                              math.radians(movimiento_pata))
        glPushMatrix()
        glMultMatrixf(local_p_dd.T)
        Gato.pata_delantera_derecha.render()
        glPopMatrix()

        # delantera izquierda 
        local_p_di = mat_T_Rx(patas_del_izquierda_offset[0], patas_del_izquierda_offset[1], patas_del_izquierda_offset[2],
                              math.radians(-movimiento_pata))
        glPushMatrix()
        glMultMatrixf(local_p_di.T)
        Gato.pata_delantera_izquierda.render()
        glPopMatrix()

        # trasera derecha )
        local_p_td = mat_T_Rx(patas_tras_derecha_offset[0], patas_tras_derecha_offset[1], patas_tras_derecha_offset[2],
                              math.radians(-movimiento_pata))
        glPushMatrix()
        glMultMatrixf(local_p_td.T)
        Gato.pata_trasera_derecha.render()
        glPopMatrix()

        # trasera izquierda 
        local_p_ti = mat_T_Rx(patas_tras_izquierda_offset[0], patas_tras_izquierda_offset[1], patas_tras_izquierda_offset[2],
                              math.radians(movimiento_pata))
        glPushMatrix()
        glMultMatrixf(local_p_ti.T)
        Gato.pata_trasera_izquierda.render()
        glPopMatrix()

        # Cola: T(cola_offset) * Ry(movimiento_cola) 
        local_cola = mat_T_Ry(cola_offset[0], cola_offset[1], cola_offset[2], math.radians(movimiento_cola))
        glPushMatrix()
        glMultMatrixf(local_cola.T)
        Gato.cola.render()
        glPopMatrix()

        glPopMatrix()