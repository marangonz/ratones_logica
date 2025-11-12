import os
import pygame
from OpenGL.GL import *

class OBJ:
    generate_on_init = True

    @classmethod
    def loadTexture(cls, imagefile):
        # imagefile ya debe ser una ruta completa o relativa válida
        if not os.path.exists(imagefile):
            print(f"[OBJ] Advertencia: textura no encontrada: {imagefile}")
            return 0
        surf = pygame.image.load(imagefile)
        # Asegurarse del formato correcto (RGBA si tiene alpha)
        if surf.get_alpha() is None:
            image = pygame.image.tostring(surf, 'RGB', 1)
            fmt = GL_RGB
            internal = GL_RGB
        else:
            image = pygame.image.tostring(surf, 'RGBA', 1)
            fmt = GL_RGBA
            internal = GL_RGBA
        ix, iy = surf.get_rect().size
        texid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texid)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, internal, ix, iy, 0, fmt, GL_UNSIGNED_BYTE, image)
        return texid

    @classmethod
    def loadMaterial(cls, filename):
        contents = {}
        mtl = None

        # intentar abrir el archivo mtl en varias ubicaciones
        tried = []
        mtl_path = filename
        if not os.path.isabs(mtl_path) and not os.path.exists(mtl_path):
            # probar ruta tal cual
            tried.append(mtl_path)
            # probar en cwd
            maybe = os.path.join(os.getcwd(), filename)
            if os.path.exists(maybe):
                mtl_path = maybe
            else:
                tried.append(maybe)
                # probar en directorio del script (donde está el .obj)
                # si filename fue pasado como ruta completa, dirname será correcta
                dirname = os.path.dirname(filename)
                if dirname:
                    maybe2 = os.path.join(dirname, os.path.basename(filename))
                    tried.append(maybe2)
                    if os.path.exists(maybe2):
                        mtl_path = maybe2

        if not os.path.exists(mtl_path):
            # no encontrado: informar y devolver dict vacío para no romper la carga
            print(f"[OBJ] ERROR: .mtl no encontrado. Intentados: {tried} -> archivo solicitado: {filename}")
            return contents

        dirname = os.path.dirname(mtl_path)

        try:
            with open(mtl_path, "r") as f:
                for line in f:
                    if line.startswith('#'):
                        continue
                    values = line.strip().split()
                    if not values:
                        continue
                    if values[0] == 'newmtl':
                        mtl = contents[values[1]] = {}
                    elif mtl is None:
                        raise ValueError("mtl file doesn't start with newmtl stmt")
                    elif values[0] in ('map_Kd', 'bump', 'map_Ns', 'refl'):
                        # almacenar el nombre tal cual y luego intentar localizar la ruta
                        mtl[values[0]] = values[1]
                        if values[0] == 'map_Kd':
                            # buscar la textura en lugares razonables
                            candidates = [
                                os.path.join(dirname, values[1]),
                                os.path.join(dirname, "textures", values[1]),
                                os.path.join(os.getcwd(), values[1]),
                                values[1]
                            ]
                            found = None
                            for c in candidates:
                                if os.path.exists(c):
                                    found = c
                                    break
                            if found:
                                mtl['texture_Kd'] = cls.loadTexture(found)
                            else:
                                print(f"[OBJ] Advertencia: map_Kd no encontrada para {values[1]}. Intentados: {candidates}")
                    else:
                        try:
                            mtl[values[0]] = list(map(float, values[1:]))
                        except ValueError:
                            mtl[values[0]] = values[1:]  # Guardar como strings si no son floats
        except Exception as e:
            print(f"[OBJ] Error leyendo {mtl_path}: {e}")
            return contents

        return contents

    def __init__(self, filename, swapyz=False):
        """Loads a Wavefront OBJ file. """
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []
        self.gl_list = 0
        self.mtl = {}
        dirname = os.path.dirname(filename)

        try:
            with open(filename, "r") as f:
                for line in f:
                    if line.startswith('#'):
                        continue
                    values = line.strip().split()
                    if not values:
                        continue
                    if values[0] == 'v':
                        v = list(map(float, values[1:4]))
                        if swapyz:
                            v = v[0], v[2], v[1]
                        self.vertices.append(v)
                    elif values[0] == 'vn':
                        v = list(map(float, values[1:4]))
                        if swapyz:
                            v = v[0], v[2], v[1]
                        self.normals.append(v)
                    elif values[0] == 'vt':
                        self.texcoords.append(list(map(float, values[1:3])))
                    elif values[0] in ('usemtl', 'usemat'):
                        material = values[1]
                    elif values[0] == 'mtllib':
                        # construir posibles rutas y usar la que exista
                        mtl_fname = values[1]
                        candidates = []
                        if dirname:
                            candidates.append(os.path.join(dirname, mtl_fname))
                            candidates.append(os.path.join(dirname, "textures", mtl_fname))
                        candidates.append(os.path.join(os.getcwd(), mtl_fname))
                        candidates.append(mtl_fname)
                        found_mtl = None
                        for c in candidates:
                            if os.path.exists(c):
                                found_mtl = c
                                break
                        if found_mtl:
                            self.mtl = self.loadMaterial(found_mtl)
                        else:
                            print(f"[OBJ] Advertencia: mtllib '{mtl_fname}' no encontrado. Intentados: {candidates}")
                            self.mtl = {}
                    elif values[0] == 'f':
                        face = []
                        texcoords = []
                        norms = []
                        material = locals().get('material', None)
                        for v in values[1:]:
                            w = v.split('/')
                            face.append(int(w[0]))
                            texcoords.append(int(w[1]) if len(w) > 1 and w[1] else 0)
                            norms.append(int(w[2]) if len(w) > 2 and w[2] else 0)
                        self.faces.append((face, norms, texcoords, material))
        except FileNotFoundError:
            raise FileNotFoundError(f"[OBJ] Archivo .obj no encontrado: {filename}")
        except Exception as e:
            raise RuntimeError(f"[OBJ] Error leyendo {filename}: {e}")

        if self.generate_on_init:
            self.generate()

    def generate(self):
        if self.gl_list:
            glDeleteLists(self.gl_list, 1)
        self.gl_list = glGenLists(1)
        glNewList(self.gl_list, GL_COMPILE)
        glEnable(GL_TEXTURE_2D)
        glFrontFace(GL_CCW)
        for face in self.faces:
            vertices, normals, texture_coords, material = face

            mtl = self.mtl.get(material, {}) if self.mtl else {}
            if 'texture_Kd' in mtl and mtl['texture_Kd']:
                glBindTexture(GL_TEXTURE_2D, mtl['texture_Kd'])
            else:
                kd = mtl.get('Kd', [1.0, 1.0, 1.0])
                # glColor espera 3 o 4 floats; asegurar tuple
                if len(kd) == 3:
                    glColor3f(kd[0], kd[1], kd[2])
                elif len(kd) >= 4:
                    glColor4f(kd[0], kd[1], kd[2], kd[3])
                else:
                    glColor3f(1.0, 1.0, 1.0)

            glBegin(GL_POLYGON)
            for i in range(len(vertices)):
                if normals[i] > 0 and len(self.normals) >= normals[i]:
                    glNormal3fv(self.normals[normals[i] - 1])
                if texture_coords[i] > 0 and len(self.texcoords) >= texture_coords[i]:
                    glTexCoord2fv(self.texcoords[texture_coords[i] - 1])
                glVertex3fv(self.vertices[vertices[i] - 1])
            glEnd()
        glDisable(GL_TEXTURE_2D)
        glEndList()

    def render(self):
        glCallList(self.gl_list)

    def free(self):
        try:
            glDeleteLists(self.gl_list, 1)
        except Exception:
            pass
