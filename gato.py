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
        self.interpolation_speed = 0.2  # Faster interpolation
        self.max_speed = 20.0 # High speed limit to prevent lagging behind
        
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
        # Handle simulation smooth movement interpolation
        # With advanced collision sliding and SMOOTH turning
        
        # Calculate distance to target
        dx = self.target_position[0] - self.Position[0]
        dz = self.target_position[2] - self.Position[2]
        distance = math.sqrt(dx*dx + dz*dz)
        
        if distance > 0.1:  # Minimal threshold for precise positioning
            # Calculate desired direction normalized
            desired_dx = dx / distance
            desired_dz = dz / distance
            
            # Check alignment with current direction to prevent "reversing"
            # self.Direction is normalized
            alignment = self.Direction[0] * desired_dx + self.Direction[2] * desired_dz
            
            turn_factor = 1.0
            if alignment < -0.1: # Facing away (more than 90 degrees)
                turn_factor = 0.05 # Turn in place (very slow movement)
            
            # Calculate proposed movement
            move_x = dx * self.interpolation_speed * turn_factor
            move_z = dz * self.interpolation_speed * turn_factor
            
            # Cap movement speed to prevent teleporting/glitching
            move_dist = math.sqrt(move_x*move_x + move_z*move_z)
            if move_dist > self.max_speed:
                scale = self.max_speed / move_dist
                move_x *= scale
                move_z *= scale
            
            # Predict next position
            test_x = self.Position[0] + move_x
            test_z = self.Position[2] + move_z
            
            # Track actual movement for rotation
            actual_move_x = move_x
            actual_move_z = move_z
            
            # Check for collision
            collision_obstacle = None
            for obstacle in self.obstacles:
                ox = test_x - obstacle.position[0]
                oz = test_z - obstacle.position[2]
                dist = math.sqrt(ox*ox + oz*oz)
                if dist < (self.collision_radius + obstacle.radius):
                    collision_obstacle = obstacle
                    break
            
            if collision_obstacle is None:
                # No collision, move normally
                self.Position[0] = test_x
                self.Position[2] = test_z
            else:
                # Collision detected! Slide around it (Tangent motion).
                # Vector from obstacle center to agent (Normal vector)
                normal_x = self.Position[0] - collision_obstacle.position[0]
                normal_z = self.Position[2] - collision_obstacle.position[2]
                norm_len = math.sqrt(normal_x*normal_x + normal_z*normal_z)
                
                if norm_len > 0:
                    normal_x /= norm_len
                    normal_z /= norm_len
                    
                    # Project movement vector onto tangent plane
                    dot_prod = move_x * normal_x + move_z * normal_z
                    
                    # Only subtract the component if it's moving INTO the obstacle
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

                    # Soft push out to prevent sinking
                    overlap = (self.collision_radius + collision_obstacle.radius) - norm_len
                    if overlap > 0:
                        self.Position[0] += normal_x * overlap * 0.2
                        self.Position[2] += normal_z * overlap * 0.2
            
            # Update direction to face ACTUAL movement direction (Smooth Turn)
            target_norm = math.sqrt(actual_move_x*actual_move_x + actual_move_z*actual_move_z)
            if target_norm > 0.001:
                target_dx = actual_move_x / target_norm
                target_dz = actual_move_z / target_norm
                
                # Interpolate current direction towards target direction (Smoothness factor 0.15)
                self.Direction[0] += (target_dx - self.Direction[0]) * 0.15
                self.Direction[2] += (target_dz - self.Direction[2]) * 0.15
                
                # Re-normalize
                d_norm = math.sqrt(self.Direction[0]**2 + self.Direction[2]**2)
                if d_norm > 0:
                    self.Direction[0] /= d_norm
                    self.Direction[2] /= d_norm
            
            self.en_movimiento = True
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
        self.angulo_direccion = max(-45.0, min(45.0, self.angulo_direccion))  # Increased range for better looking behavior

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
        # Helper for part transformation
        def get_part_matrix(px, py, pz, rx, ry, rz, ox=0, oy=0, oz=0):
            # px,py,pz: Position of the joint on the body (Body Space)
            # rx,ry,rz: Rotation angles
            # ox,oy,oz: Offset of the mesh relative to the joint (Mesh Space)
            
            # 1. Translate to Joint Position
            T = np.identity(4, dtype=np.float32)
            T[0,3], T[1,3], T[2,3] = px, py, pz
            
            # 2. Rotate
            cx, sx = math.cos(rx), math.sin(rx)
            Rx = np.array([[1,0,0,0], [0,cx,-sx,0], [0,sx,cx,0], [0,0,0,1]], dtype=np.float32)
            
            cy, sy = math.cos(ry), math.sin(ry)
            Ry = np.array([[cy,0,sy,0], [0,1,0,0], [-sy,0,cy,0], [0,0,0,1]], dtype=np.float32)
            
            cz, sz = math.cos(rz), math.sin(rz)
            Rz = np.array([[cz,-sz,0,0], [sz,cz,0,0], [0,0,1,0], [0,0,0,1]], dtype=np.float32)
            
            R = Ry @ Rx @ Rz
            
            # 3. Apply Mesh Offset (so pivot is at origin)
            To = np.identity(4, dtype=np.float32)
            To[0,3], To[1,3], To[2,3] = ox, oy, oz
            
            return T @ R @ To

        # Main body transform
        def mat_body(tx, ty, tz, sx, sy, sz, theta_rad):
            c = math.cos(theta_rad)
            s = math.sin(theta_rad)
            
            # Rotation (Y-axis)
            R = np.array([
                [c, 0, s, 0],
                [0, 1, 0, 0],
                [-s, 0, c, 0],
                [0, 0, 0, 1]
            ], dtype=np.float32)
            
            # Scale
            S = np.identity(4, dtype=np.float32)
            S[0,0], S[1,1], S[2,2] = sx, sy, sz
            
            # Translation
            T = np.identity(4, dtype=np.float32)
            T[0,3], T[1,3], T[2,3] = tx, ty, tz
            
            return T @ R @ S

        angle = math.degrees(math.atan2(self.Direction[0], self.Direction[2]))
        theta = math.radians(angle)
        sx = sy = sz = self.scale
        tx, ty, tz = self.Position

        # Animation State
        # NOTE: Animation update is handled in update(), removing it here to prevent double-speed
        # if self.en_movimiento:
        #    self.anim_angle += self.anim_speed
        if self.anim_angle > 360.0:
            self.anim_angle -= 360.0
            
        # --- Animation Parameters ---
        # Reduced amplitudes to prevent parts from clipping out of the body
        leg_swing = 8.0   # Drastically reduced to keep legs "closed" and under body
        tail_wag = 8.0
        head_bob = 2.0    
        
        # Sine waves
        sin_walk = math.sin(math.radians(self.anim_angle))
        cos_walk = math.cos(math.radians(self.anim_angle))
        
        # Body Bob (Vertical bounce)
        bob_y = abs(sin_walk) * 0.3 
        ty += bob_y

        # --- Render Hierarchy ---
        
        # 1. Body
        M_world = mat_body(tx, ty, tz, sx, sy, sz, theta)
        glPushMatrix()
        glMultMatrixf(M_world.T)
        Gato.cuerpo.render()
        
        # --- Joint Configuration ---
        # Coordinates are relative to Body Center (0,0,0)
        
        # Head: Pivot raised to spine height (3.0) and moved back (-3.3)
        # Mesh lowered (-0.8) to bury the neck connection
        p_head = (0.0, 1.5, -1.0) 
        off_head = (0.0, -0.5, 0.0) 
        
        # Tail: Pivot at spine end, mesh pushed IN
        p_tail = (0.0, 4.0, 5.0)
        # Shifted mesh RIGHT (X=0.6) to correct visual centering
        off_tail = (0.6, -0.5, 1.0) 
        
        # Legs: Pivots moved inward (0.65) to keep legs under the torso
        p_fl = (-0.45, 3.0, -2.5)
        p_fr = ( 0.45, 3.0, -2.5)
        p_bl = (-0.45, 3.0,  2.5)
        p_br = ( 0.45, 3.0,  2.5)
        
        # Leg Offset: Reduced downward shift so legs sit higher
        off_leg = (0.0, -0.5, 0.0) 

        # --- Draw Parts ---

        # Head (Counter-phase bobbing for natural compensation)
        head_ry = math.radians(self.angulo_direccion)
        # Head dips slightly when body rises (counter-motion)
        head_rx = math.radians(-sin_walk * head_bob) if self.en_movimiento else 0
        M_head = get_part_matrix(*p_head, head_rx, head_ry, 0, *off_head)
        glPushMatrix()
        glMultMatrixf(M_head.T)
        Gato.cabeza.render()
        glPopMatrix()
        
        # Tail
        # "Cute" slow wag: Left-Right motion only, stable height to avoid circular motion
        if self.en_movimiento:
            # Slower frequency (0.2) for a lazy/cute wag
            tail_wag = math.sin(math.radians(self.anim_angle * 0.2)) * 15.0
            # Stable lift, slightly up (negative is usually up in this rig)
            tail_lift = -10.0 
        else:
            # Idle: Very slow sway
            tail_wag = math.sin(self.tiempo_idle * 0.05) * 5.0
            tail_lift = -15.0
            
        tail_ry = math.radians(tail_wag)
        tail_rx = math.radians(tail_lift)
        
        M_tail = get_part_matrix(*p_tail, tail_rx, tail_ry, 0, *off_tail)
        glPushMatrix()
        glMultMatrixf(M_tail.T)
        Gato.cola.render()
        glPopMatrix()
        
        # Legs (Trot Gait with Lift)
        # FL & BR move together (Group A)
        # FR & BL move together (Group B)
        
        rot_a = math.radians(sin_walk * leg_swing)
        rot_b = math.radians(-sin_walk * leg_swing)
        
        # Leg Lift: Raise the leg slightly during its forward swing to simulate knee bending
        # Group A moves forward when sin_walk > 0
        lift_amp = 0.2 # Reduced lift to match smaller swing
        lift_a = max(0, sin_walk) * lift_amp
        lift_b = max(0, -sin_walk) * lift_amp
        
        # Front Left (A)
        M_fl = get_part_matrix(p_fl[0], p_fl[1] + lift_a, p_fl[2], rot_a, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_fl.T)
        Gato.pata_delantera_izquierda.render()
        glPopMatrix()
        
        # Front Right (B)
        M_fr = get_part_matrix(p_fr[0], p_fr[1] + lift_b, p_fr[2], rot_b, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_fr.T)
        Gato.pata_delantera_derecha.render()
        glPopMatrix()
        
        # Back Left (B)
        M_bl = get_part_matrix(p_bl[0], p_bl[1] + lift_b, p_bl[2], rot_b, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_bl.T)
        Gato.pata_trasera_izquierda.render()
        glPopMatrix()
        
        # Back Right (A)
        M_br = get_part_matrix(p_br[0], p_br[1] + lift_a, p_br[2], rot_a, 0, 0, *off_leg)
        glPushMatrix()
        glMultMatrixf(M_br.T)
        Gato.pata_trasera_derecha.render()
        glPopMatrix()

        glPopMatrix()