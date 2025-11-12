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
        self.anim_speed = 20.0  # Ultra-fast animation for maximum fluidity

        # Nueva variable: controla si el ratón está avanzando
        self.en_movimiento = False
        
        # Collision detection properties
        self.collision_radius = 15.0  # Mouse collision radius (smaller than cat)
        
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
            0.0,  # Posicionar en el piso (Y=0)
            random.randint(-1 * self.DimBoard, self.DimBoard)
        ]

        # Smooth movement for simulation-controlled mice
        self.target_position = self.Position.copy()
        self.interpolation_speed = 0.8  # Maximum interpolation speed for real-time sync
        
        # Capture state - when caught by cat
        self.captured = False
        
        # Collision system
        self.obstacles = []  # Will be set by main.py

        # vector dirección en XZ (y no se usa para avanzar)
        dx = random.uniform(-1.0, 1.0)
        dz = random.uniform(-1.0, 1.0)
        m = math.sqrt(dx*dx + dz*dz) or 1.0
        # usa el vector aleatorio normalizado (si prefieres siempre mirar +Z, pon [0,0,1])
        self.Direction = [dx / m, 0.0, dz / m]

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
        for angle_offset in [math.pi/6, -math.pi/6, math.pi/3, -math.pi/3, math.pi/2, -math.pi/2]:
            # Calculate direction to target
            dx = target_x - current_x
            dz = target_z - current_z
            distance = math.sqrt(dx*dx + dz*dz)
            
            if distance > 0:
                # Apply rotation to direction
                rotated_dx = dx * math.cos(angle_offset) - dz * math.sin(angle_offset)
                rotated_dz = dx * math.sin(angle_offset) + dz * math.cos(angle_offset)
                
                # Move a small step in the rotated direction
                step_size = 20.0  # Smaller step size for mice navigation
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
        # Don't update captured mice
        if self.captured:
            return
            
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
            # Use frame-based animation for smoother movement
            self.anim_angle += self.anim_speed
        else:
            # Slow down animation when not moving to create a natural stop
            if self.anim_speed > 1.0:
                self.anim_speed *= 0.95  # Gradual slowdown
            else:
                self.anim_speed = 1.0
            self.anim_angle += self.anim_speed * 0.3  # Slower idle animation
            
        # Keep angle in reasonable range
        if self.anim_angle > 360:
            self.anim_angle -= 360
        elif self.anim_angle < 0:
            self.anim_angle += 360
            
        # Asegurar que el ratón siempre esté en el piso y dentro de límites de cámara
        self.Position[1] = 0.0
        # Keep within camera bounds (±300)
        self.Position[0] = max(-300, min(300, self.Position[0]))
        self.Position[2] = max(-300, min(300, self.Position[2]))
                
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
        # Don't draw captured mice
        if self.captured:
            return
            
        global llantaTraseraRoll
        # calculamos ángulo de orientación a partir del vector dirección
        angle = math.degrees(math.atan2(self.Direction[0], self.Direction[2])) 
       
        angle += 90.0
        
        t = math.radians(angle)
        sx = sy = sz = self.scale
        tx, ty, tz = self.Position
        s = math.sin(t)
        c = math.cos(t)
       
        glPushMatrix()
        glTranslatef(tx, ty, tz)
        glRotatef(angle, 0.0, 1.0, 0.0)
        glScalef(sx, sy, sz)
        Raton.modelo.render()

        # cabeza
        glPushMatrix()
        glTranslatef(0.0, 0.0, 0.0)
        glRotatef(self.angulo_direccion, 0.0, 0.3, 0.0)
        Raton.cabeza.render()
        glPopMatrix()

        # patas traseras (animadas)
        glPushMatrix()
        # Enhanced leg animation - more pronounced when moving
        if self.en_movimiento:
            movimiento_pata = math.sin(math.radians(self.anim_angle)) * 20  # Increased range
        else:
            movimiento_pata = math.sin(math.radians(self.anim_angle)) * 5   # Subtle idle movement
        glTranslatef(0.0, 0.0, 0.0)
        glRotatef(movimiento_pata, 1.0, 0.0, 0.0)
        Raton.patastraseras.render()
        glPopMatrix()

        # patas delanteras
        glPushMatrix()
        glTranslatef(0.0, 0.0, 0.0)
        # Front legs move opposite to back legs for realistic gait
        if self.en_movimiento:
            movimiento_pata_delantera = math.sin(math.radians(self.anim_angle + 180)) * 20  # 180° phase shift
        else:
            movimiento_pata_delantera = math.sin(math.radians(self.anim_angle + 180)) * 5
        glRotatef(movimiento_pata_delantera, 1.0, 0.0, 0.0)
        Raton.patasdelanteras.render()
        glPopMatrix()

        # cola 
        glPushMatrix()
        glTranslatef(0.0, 0.0, 0.0)  
        if self.en_movimiento:
            # More dynamic tail movement when walking
            movimiento_cola = math.sin(math.radians(self.anim_angle * 1.5)) * 15  # Increased range and frequency
        else:
            # Gentle tail sway when idle
            movimiento_cola = math.sin(math.radians(self.anim_angle * 0.5)) * 5
        glRotatef(movimiento_cola, 0.0, 1.0, 0.0)  
        Raton.cola.render()
        glPopMatrix()

        glPopMatrix()