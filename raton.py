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
        
        # Powerup/Slowdown states
        self.powered_up = False
        self.slowed_down = False
        
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
        self.interpolation_speed = 0.25  
        self.max_speed = 20.0 
        self.move_speed = 0.0
        
        # Estado capturado 
        self.captured = False
        
        # Poderes magicos
        self.powered_up = False
        
        # Colisiones
        self.obstacles = [] 

        # vector dirección en XZ (y no se usa para avanzar)
        dx = random.uniform(-1.0, 1.0)
        dz = random.uniform(-1.0, 1.0)
        m = math.sqrt(dx*dx + dz*dz) or 1.0
        # usa el vector aleatorio normalizado (si prefieres siempre mirar +Z, pon [0,0,1])
        self.Direction = [dx / m, 0.0, dz / m]

    def set_target_position(self, target_pos):
        self.target_position = target_pos.copy()
        
        # Calcular velocidad requerida para alcanzar el objetivo en aprox 9 a 10 frames
        dx = self.target_position[0] - self.Position[0]
        dz = self.target_position[2] - self.Position[2]
        dist = math.sqrt(dx*dx + dz*dz)
        
        if dist > 200: 
            self.move_speed = dist / 5.0 
        else:
            self.move_speed = dist / 9.0 
            
        if self.move_speed < 0.5: self.move_speed = 0.5
    
    def set_obstacles(self, obstacles):
        self.obstacles = obstacles
    
    def check_collision(self, position):
        for obstacle in self.obstacles:
            dx = position[0] - obstacle.position[0]
            dz = position[2] - obstacle.position[2]
            distance = math.sqrt(dx*dx + dz*dz)
            
            if distance < (self.collision_radius + obstacle.radius):
                return True
        return False
    
    def find_safe_move(self, target_pos):
        if not self.check_collision(target_pos):
            return target_pos
        
        current_x, current_y, current_z = self.Position
        target_x, target_y, target_z = target_pos
        
        # Intentar moverse alrededor de los obstáculos probando diferentes ángulos
        for angle_offset in [math.pi/6, -math.pi/6, math.pi/3, -math.pi/3, math.pi/2, -math.pi/2]:
            # Calcular dirección hacia el objetivo
            dx = target_x - current_x
            dz = target_z - current_z
            distance = math.sqrt(dx*dx + dz*dz)
            
            if distance > 0:
                # Aplicar rotación a la dirección
                rotated_dx = dx * math.cos(angle_offset) - dz * math.sin(angle_offset)
                rotated_dz = dx * math.sin(angle_offset) + dz * math.cos(angle_offset)
                
                # Mover un pequeño paso en la dirección rotada
                step_size = 20.0  # Tamaño de paso más pequeño para la navegación de ratones
                new_x = current_x + (rotated_dx / distance) * step_size
                new_z = current_z + (rotated_dz / distance) * step_size
                
                test_pos = [new_x, current_y, new_z]
                
                # Limitar al dimboard
                test_pos[0] = max(-300, min(300, test_pos[0]))
                test_pos[2] = max(-300, min(300, test_pos[2]))
                
                if not self.check_collision(test_pos):
                    return test_pos
        
        return self.Position.copy()

    def update(self):
        if self.captured:
            return
        
        dx = self.target_position[0] - self.Position[0]
        dz = self.target_position[2] - self.Position[2]
        distance = math.sqrt(dx*dx + dz*dz)
        
        if distance > 100.0:
            self.Position[0] = self.target_position[0]
            self.Position[2] = self.target_position[2]
            return

        if distance > 0.1:
            desired_dx = dx / distance
            desired_dz = dz / distance
            
            step = self.move_speed
            if step > distance: step = distance 
            
            move_x = desired_dx * step
            move_z = desired_dz * step
            
            move_dist = math.sqrt(move_x*move_x + move_z*move_z)
            if move_dist > self.max_speed:
                scale = self.max_speed / move_dist
                move_x *= scale
                move_z *= scale
            
            test_x = self.Position[0] + move_x
            test_z = self.Position[2] + move_z
            
            actual_move_x = move_x
            actual_move_z = move_z
            
            # Checa por colisiones
            collision_obstacle = None
            for obstacle in self.obstacles:
                ox = test_x - obstacle.position[0]
                oz = test_z - obstacle.position[2]
                dist = math.sqrt(ox*ox + oz*oz)
                if dist < (self.collision_radius + obstacle.radius):
                    collision_obstacle = obstacle
                    break
            
            if collision_obstacle is None:
                self.Position[0] = test_x
                self.Position[2] = test_z
            else:
                normal_x = self.Position[0] - collision_obstacle.position[0]
                normal_z = self.Position[2] - collision_obstacle.position[2]
                norm_len = math.sqrt(normal_x*normal_x + normal_z*normal_z)
                
                if norm_len > 0:
                    normal_x /= norm_len
                    normal_z /= norm_len
                    
                    dot_prod = move_x * normal_x + move_z * normal_z
                    
                    if dot_prod < 0:
                        slide_x = move_x - dot_prod * normal_x
                        slide_z = move_z - dot_prod * normal_z
                        
                        self.Position[0] += slide_x
                        self.Position[2] += slide_z
                        actual_move_x = slide_x
                        actual_move_z = slide_z
                    else:
                        self.Position[0] += move_x
                        self.Position[2] += move_z

                    overlap = (self.collision_radius + collision_obstacle.radius) - norm_len
                    if overlap > 0:
                        self.Position[0] += normal_x * overlap * 0.2
                        self.Position[2] += normal_z * overlap * 0.2
            
            target_norm = math.sqrt(actual_move_x*actual_move_x + actual_move_z*actual_move_z)
            if target_norm > 0.001:
                target_dx = actual_move_x / target_norm
                target_dz = actual_move_z / target_norm
                
                current_angle = math.atan2(self.Direction[0], self.Direction[2])
                target_angle = math.atan2(target_dx, target_dz)
                
                diff = target_angle - current_angle
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
    
                turn_speed = 0.2
                if abs(diff) > math.pi / 2:
                    turn_speed = 0.4
                
                new_angle = current_angle + diff * turn_speed
                
                self.Direction[0] = math.sin(new_angle)
                self.Direction[2] = math.cos(new_angle)
            
            self.en_movimiento = True
        else:
            self.en_movimiento = False
            
        #Animacion
        if self.en_movimiento:
            self.anim_angle += self.anim_speed
        else:
            if self.anim_speed > 1.0:
                self.anim_speed *= 0.95 
            else:
                self.anim_speed = 1.0
            self.anim_angle += self.anim_speed * 0.3 
            
        if self.anim_angle > 360:
            self.anim_angle -= 360
        elif self.anim_angle < 0:
            self.anim_angle += 360
            
        # Asegurar que el ratón siempre esté en el piso y dentro de límites de cámara
        self.Position[1] = 0.0
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
       
        # Cambio de color al recibir efecto del queso mágico
        if self.powered_up:
            glDisable(GL_TEXTURE_2D)
            glColor3f(0.0, 0.5, 1.0) # tinta azul
        elif self.slowed_down:
            glDisable(GL_TEXTURE_2D)
            glColor3f(0.0, 1.0, 0.0) # tinta verde

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
        if self.en_movimiento:
            movimiento_pata = math.sin(math.radians(self.anim_angle)) * 20  
        else:
            movimiento_pata = math.sin(math.radians(self.anim_angle)) * 5   
        glTranslatef(0.0, 0.0, 0.0)
        glRotatef(movimiento_pata, 1.0, 0.0, 0.0)
        Raton.patastraseras.render()
        glPopMatrix()

        glPushMatrix()
        glTranslatef(0.0, 0.0, 0.0)
        if self.en_movimiento:
            movimiento_pata_delantera = math.sin(math.radians(self.anim_angle + 180)) * 20  
        else:
            movimiento_pata_delantera = math.sin(math.radians(self.anim_angle + 180)) * 5
        glRotatef(movimiento_pata_delantera, 1.0, 0.0, 0.0)
        Raton.patasdelanteras.render()
        glPopMatrix()

        # cola 
        glPushMatrix()
        glTranslatef(0.0, 0.0, 0.0)  
        if self.en_movimiento:
            movimiento_cola = math.sin(math.radians(self.anim_angle * 1.5)) * 15 
        else:
            movimiento_cola = math.sin(math.radians(self.anim_angle * 0.5)) * 5
        glRotatef(movimiento_cola, 0.0, 1.0, 0.0)  
        Raton.cola.render()
        glPopMatrix()

        glPopMatrix()
        
        # Regresa a color y texturas normales cuando se va el poder
        if self.powered_up or self.slowed_down:
            glColor3f(1.0, 1.0, 1.0)
            glEnable(GL_TEXTURE_2D)