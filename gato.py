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
        self.anim_speed = 6.0  # Reduced speed for calmer movement
        self.cabeza_anim = 0.0  # animación específica de cabeza
        
        # Control de movimiento
        self.en_movimiento = False
    # Dirección de movimiento: 1 = adelante, -1 = atrás, 0 = parado
        self.moving_direction = 0
        self.angulo_direccion = 0.0  # Ángulo de dirección de la cabeza
        
        # Variables para animaciones más complejas
        self.tiempo_idle = 0.0  # tiempo sin moverse para animaciones idle
        self.fase_patas = 0.0   # desfase entre patas delanteras y traseras, para que se intentara ver mas natural pero no funciono del todo
        self.tail_timer = 0.0   # Continuous timer for tail animation
        
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

        # Movimiento suave en la simulación
        self.target_position = self.Position.copy()
        self.interpolation_speed = 0.25  
        self.max_speed = 20.0 # DEBUG: evita el lagging extremo
        self.move_speed = 0.0 
        
        # Colisión con obstáculos
        self.obstacles = []

        # Dirección inicial
        self.Direction = [0.0, 0.0, -1.0]
        
        # Animación agregada - Comiendo los ratones 
        self.is_eating = False
        self.eating_timer = 0.0
        self.eating_duration = 1.0 
        self.eating_phase = 0.0 

    def trigger_eating(self):
        """Empieza la animación de comer"""
        self.is_eating = True
        self.eating_timer = self.eating_duration
        self.eating_phase = 0.0

    def set_target_position(self, target_pos):
        """Establece la posición objetivo para el movimiento suave en la simulación"""
        self.target_position = target_pos.copy()
        
        # Calcular la velocidad requerida para alcanzar el objetivo en 0.15s (aprox de 9 a 10 frames)
        dx = self.target_position[0] - self.Position[0]
        dz = self.target_position[2] - self.Position[2]
        dist = math.sqrt(dx*dx + dz*dz)
        
        if dist > 200: 
            self.move_speed = dist / 5.0 
        else:
            self.move_speed = dist / 9.0 
            
        # Ensure minimum speed to prevent stalling on small adjustments
        if self.move_speed < 0.5: self.move_speed = 0.5
    
    def set_obstacles(self, obstacles):
        """Establece la lista de obstáculos para la detección de colisiones"""
        self.obstacles = obstacles
    
    def check_collision(self, position):
        """Verifica si una posición causaría colisión con obstáculos"""
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
        
        # Lógica para ir alrededor de los obstaculos 
        current_x, current_y, current_z = self.Position
        target_x, target_y, target_z = target_pos
        
        # Testeo de angulos diferentes para romper bloqueos de colisiones
        for angle_offset in [math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, 3*math.pi/4, -3*math.pi/4]:
            # Calculate direction to target
            dx = target_x - current_x
            dz = target_z - current_z
            distance = math.sqrt(dx*dx + dz*dz)
            
            if distance > 0:
                # Aplica las rotaciones que hacen que gire alrededor del obstáculo
                rotated_dx = dx * math.cos(angle_offset) - dz * math.sin(angle_offset)
                rotated_dz = dx * math.sin(angle_offset) + dz * math.cos(angle_offset)
                
                step_size = 30.0  
                new_x = current_x + (rotated_dx / distance) * step_size
                new_z = current_z + (rotated_dz / distance) * step_size
                
                test_pos = [new_x, current_y, new_z]
                
                test_pos[0] = max(-300, min(300, test_pos[0]))
                test_pos[2] = max(-300, min(300, test_pos[2]))
                
                if not self.check_collision(test_pos):
                    return test_pos
        
        # If no safe path found, stay in place
        return self.Position.copy()

    def update(self):
        """Actualiza la posición y animaciones del gato"""
        dx = self.target_position[0] - self.Position[0]
        dz = self.target_position[2] - self.Position[2]
        distance = math.sqrt(dx*dx + dz*dz)
        
        if distance > 0.1:
            desired_dx = dx / distance
            desired_dz = dz / distance
            
       
            alignment = self.Direction[0] * desired_dx + self.Direction[2] * desired_dz
            
            turn_factor = 1.0
            if alignment < -0.1:
                turn_factor = 0.05
                
            step = self.move_speed * turn_factor
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
                
                self.Direction[0] += (target_dx - self.Direction[0]) * 0.15
                self.Direction[2] += (target_dz - self.Direction[2]) * 0.15
                
                d_norm = math.sqrt(self.Direction[0]**2 + self.Direction[2]**2)
                if d_norm > 0:
                    self.Direction[0] /= d_norm
                    self.Direction[2] /= d_norm
            
            self.en_movimiento = True
        else:
            self.en_movimiento = False
            
        if self.is_eating:
            self.eating_timer -= 0.05
            if self.eating_timer <= 0:
                self.is_eating = False
                self.eating_phase = 0.0
            else:
                t = 1.0 - (self.eating_timer / self.eating_duration)
                self.eating_phase = math.sin(t * math.pi)
        
        # Actualizar animaciones
        self.tail_timer += 1.0 
        
        if self.en_movimiento:
            # Animaciones de caminar - similar al ratón
            self.anim_angle += self.anim_speed
            self.fase_patas += self.anim_speed * 0.8  
            self.cabeza_anim += self.anim_speed * 0.6 
            self.tiempo_idle = 0.0
        else:
            # Animación de espera (cuando está parado)
            self.tiempo_idle += 1.0
            self.cabeza_anim += 2.0  
            
        # Mantener ángulos en rango
        if self.anim_angle > 360:
            self.anim_angle -= 360
        if self.fase_patas > 360:
            self.fase_patas -= 360
        if self.cabeza_anim > 360:
            self.cabeza_anim -= 360
        
        # Mantener siempre en el piso y dentro de límites de cámara
        self.Position[1] = 0.0
        self.Position[0] = max(-300, min(300, self.Position[0]))
        self.Position[2] = max(-300, min(300, self.Position[2]))
                
    def move_forward(self):
        dx = self.Direction[0] * self.velocity
        dz = self.Direction[2] * self.velocity
        self.Position[0] += dx
        self.Position[2] += dz
        self.Position[1] = 0.0
        avance = math.sqrt(dx*dx + dz*dz)
        self.distancia_recorrida += avance
        vueltas = self.distancia_recorrida / (2 * math.pi * self.radio_llanta)
        self.llantastraseras = (vueltas * 360) % 360

    def move_backward(self):
        dx = self.Direction[0] * self.velocity
        dz = self.Direction[2] * self.velocity
        self.Position[0] -= dx
        self.Position[2] -= dz
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
        self.angulo_direccion = max(-45.0, min(45.0, self.angulo_direccion))

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

    # DIBUJADO
    def draw(self):
        def get_part_matrix(px, py, pz, rx, ry, rz, ox=0, oy=0, oz=0):
            
            # 1. Traaslado
            T = np.identity(4, dtype=np.float32)
            T[0,3], T[1,3], T[2,3] = px, py, pz
            
            # 2. Rotaciones
            cx, sx = math.cos(rx), math.sin(rx)
            Rx = np.array([[1,0,0,0], [0,cx,-sx,0], [0,sx,cx,0], [0,0,0,1]], dtype=np.float32)
            
            cy, sy = math.cos(ry), math.sin(ry)
            Ry = np.array([[cy,0,sy,0], [0,1,0,0], [-sy,0,cy,0], [0,0,0,1]], dtype=np.float32)
            
            cz, sz = math.cos(rz), math.sin(rz)
            Rz = np.array([[cz,-sz,0,0], [sz,cz,0,0], [0,0,1,0], [0,0,0,1]], dtype=np.float32)
            
            R = Ry @ Rx @ Rz
            
            # 3. Aplicar desplazamiento de malla (para que el pivote esté en el origen)
            To = np.identity(4, dtype=np.float32)
            To[0,3], To[1,3], To[2,3] = ox, oy, oz
            
            return T @ R @ To

        # Transformación principal del cuerpo
        def mat_body(tx, ty, tz, sx, sy, sz, theta_rad):
            c = math.cos(theta_rad)
            s = math.sin(theta_rad)
            
            # Rotación (eje Y)
            R = np.array([
                [c, 0, s, 0],
                [0, 1, 0, 0],
                [-s, 0, c, 0],
                [0, 0, 0, 1]
            ], dtype=np.float32)
            
            # Escalado
            S = np.identity(4, dtype=np.float32)
            S[0,0], S[1,1], S[2,2] = sx, sy, sz
            
            # Traslación
            T = np.identity(4, dtype=np.float32)
            T[0,3], T[1,3], T[2,3] = tx, ty, tz
            
            return T @ R @ S

        angle = math.degrees(math.atan2(self.Direction[0], self.Direction[2]))
        theta = math.radians(angle)
        sx = sy = sz = self.scale
        tx, ty, tz = self.Position

        if self.anim_angle > 360.0:
            self.anim_angle -= 360.0
            
        leg_swing = 15.0  
        tail_wag = 8.0
        head_bob = 3.0   
        
        sin_walk = math.sin(math.radians(self.anim_angle))
        cos_walk = math.cos(math.radians(self.anim_angle))
        
        bob_y = abs(sin_walk) * 0.5 
        ty += bob_y

        #Render (orden de importancia)
        
        #CUERPO
        M_world = mat_body(tx, ty, tz, sx, sy, sz, theta)
        glPushMatrix()
        glMultMatrixf(M_world.T)
        Gato.cuerpo.render()
        
        #Cabeza
        p_head = (0.0, 1.5, -1.0) 
        off_head = (0.0, -0.5, 0.0) 
        
        # Cola
        p_tail = (0.0, 4.0, 5.0)
        off_tail = (0.6, -0.5, 1.0) 
        
        # Correccion de posición de las patas para que queden bien alineadas
        p_fl = (-0.45, 3.0, -2.5)
        p_fr = ( 0.45, 3.0, -2.5)
        p_bl = (-0.45, 3.0,  2.5)
        p_br = ( 0.45, 3.0,  2.5)
        
        off_leg = (0.0, -0.5, 0.0) 

        # Dibujado de las animaciones

        # Cabeza
        head_ry = math.radians(self.angulo_direccion)
        head_rx = math.radians(-sin_walk * head_bob) if self.en_movimiento else 0
        
        # Animacion de comer, la cabeza se mueve hacia abajo y tiembla un poco
        if self.is_eating:
            eat_angle = 45.0 * self.eating_phase
            head_rx += math.radians(eat_angle)
            
            shake = math.sin(self.eating_phase * 20.0) * 10.0 * self.eating_phase
            head_ry += math.radians(shake)
            
        M_head = get_part_matrix(*p_head, head_rx, head_ry, 0, *off_head)
        glPushMatrix()
        glMultMatrixf(M_head.T)
        Gato.cabeza.render()
        glPopMatrix()
        
        # Cola
        tail_wag = math.sin(self.tail_timer * 0.05) * 5.0
        tail_lift = -15.0
            
        tail_ry = math.radians(tail_wag)
        tail_rx = math.radians(tail_lift)
        
        M_tail = get_part_matrix(*p_tail, tail_rx, tail_ry, 0, *off_tail)
        glPushMatrix()
        glMultMatrixf(M_tail.T)
        Gato.cola.render()
        glPopMatrix()
        
        rot_a = math.radians(sin_walk * leg_swing)
        rot_b = math.radians(-sin_walk * leg_swing)
        
        # Animación para las piernas
        lift_amp = 0.8 
        lift_a = max(0, sin_walk) * lift_amp
        lift_b = max(0, -sin_walk) * lift_amp
        
        # Patas delanteras izquierda (A)
        M_fl = get_part_matrix(p_fl[0], p_fl[1] + lift_a, p_fl[2], rot_a, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_fl.T)
        Gato.pata_delantera_izquierda.render()
        glPopMatrix()
        
        # Patas delanteras derecha (B)
        M_fr = get_part_matrix(p_fr[0], p_fr[1] + lift_b, p_fr[2], rot_b, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_fr.T)
        Gato.pata_delantera_derecha.render()
        glPopMatrix()
        
        # Patas traseras izquierda (B)
        M_bl = get_part_matrix(p_bl[0], p_bl[1] + lift_b, p_bl[2], rot_b, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_bl.T)
        Gato.pata_trasera_izquierda.render()
        glPopMatrix()
        
        # Patas traseras derecha (A)
        M_br = get_part_matrix(p_br[0], p_br[1] + lift_a, p_br[2], rot_a, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_br.T)
        Gato.pata_trasera_derecha.render()
        glPopMatrix()

        glPopMatrix()