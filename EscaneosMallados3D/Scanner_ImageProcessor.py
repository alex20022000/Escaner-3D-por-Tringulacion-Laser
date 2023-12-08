"""
Universidad Nacional de Lomas de Zamora
Facultad de Ingeniería
Proyecto Final - Ingeniería Mecatrónica - Escaner 3D por Triangulación Láser

Integrantes: 
- Alexander Dalle Rive Prietto
- Matías Nicolás Labreñiuk

Introducción al script:

En el presente script, se encuentra la programación de la interfaz gráfica para la ejecución del scanner.
Se crean los diferentes widgets (botones, entrada de datos, etiquetas, estilos, etc.)


"""


# --------------------------------
# Importamos librerías

import numpy as np
import cv2
import time
import os
from pyntcloud import PyntCloud
import pandas as pd

# --------------------------------
# Creamos la Clase de Procesamiento de Imagen


class ImagePro:
    # --------------------------------
    # Función para inicializar los parámetros

    def __init__(self):
        self.npoints = 0
        self.file_name = ""
        self.point_cloud = []
        self.folder_name = ""
        # Matriz de referencia con posiciones espaciales de los pixeles en X
        self.mtx_xi = np.loadtxt("ScriptScanner/mtx_x.txt")
        # Matriz de referencia con posiciones espaciales de los pixeles en Y
        self.mtx_yi = np.loadtxt("ScriptScanner/mtx_y.txt")
        # Matriz de referencia con posiciones espaciales de los pixeles en Z
        self.mtx_zi = -np.loadtxt("ScriptScanner/mtx_z.txt")
        # Escalas X,Y y Z
        self.scale_factor_x = 1.138507462686567
        self.scale_factor_y = 1.138507462686567
        self.scale_factor_z = 1

    # --------------------------------
    # Función para filtrar los pixeles en Rojo de imagen

    def HSV_Filter(self, image=None):
        result = image.copy()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        ####
        lower = np.array([155, 25, 0])
        upper = np.array([179, 255, 255])
        ####
        lower = np.array([0, 200, 125])
        upper = np.array([180, 255, 255])
        ####
        mask = cv2.inRange(image, lower, upper)
        result = cv2.bitwise_and(result, result, mask=mask)
        img_gris = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        return img_gris

    # --------------------------------
    # Función para calibrar imagen

    def Calibrate(self, imagen, h, w):
        "PROPIEDADES INTRÍNSECAS DE CÁMARA"
        # --------------------------------
        # Matriz intrínseca de Cámara
        M_int = np.array(
            [
                [1.42368362e03, 0.00000000e00, 9.50043678e02],
                [0.00000000e00, 1.42475207e03, 5.81229847e02],
                [0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )

        # --------------------------------
        # Matríz de Distorsión
        D = np.array([0.06048523, -0.27447534, -0.00033107, -0.00219491, 0.22280941])

        # --------------------------------
        # Matriz Transformación de Cámara a Pixel

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            M_int, D, (w, h), 1, (w, h), centerPrincipalPoint=False
        )
        # --------------------------------
        # Matriz Imagen calibrada

        dst = cv2.undistort(imagen, M_int, D, None, newcameramtx)

        # --------------------------------
        # Se ajusta tamaño
        x, y, w, h = roi
        dst = dst[y : y + h, x : x + w]
        return dst

    # --------------------------------
    # Función para crear archivo .ply

    def CREATE_PLY(self, pts, write_text):
        # ref: https://pyntcloud.readthedocs.io/en/latest/io.html
        # the doc is scarce and not complete

        # --------------------------------
        # Se toma la cantidad de puntos relevados
        n = len(pts)

        # --------------------------------
        # Se escriben los datos como DataFrame
        if n > 0:
            data = {
                "x": pts[:, 0],
                "y": pts[:, 1],
                "z": pts[:, 2],
                "red": np.uint8(pts[:, 3]),
                "green": np.uint8(pts[:, 4]),
                "blue": np.uint8(pts[:, 5]),
            }

            # --------------------------------
            # Se crea la nube de puntos
            cloud = PyntCloud(pd.DataFrame(data))

            # --------------------------------
            # Se crea archivo .ply
            cloud.to_file(
                "EscaneosNubeDePuntos/" + str(write_text) + str(".ply"),
                as_text=write_text,
            )
        return "EscaneosNubeDePuntos/" + str(write_text) + str(".ply")

    # --------------------------------
    # Función para procesar imagen desde Secuencia 1

    def PROCESAR_IMAGEN_SEC1(self, image_laser, image_led, angulo):
        # lista de posiciones de pixeles x e y
        pos_pix = []
        # Obtenemos la imagen y los datos de altura y ancho en pixeles
        scanner_image = image_laser  # cv2.cvtColor(image_laser, cv2.COLOR_BGR2GRAY)
        h, w = scanner_image.shape[:2]

        clr_image = np.array(image_led)
        clr_image = self.Calibrate(clr_image, h, w)

        # Procesamos la imagen
        scanner_image = self.Calibrate(scanner_image, h, w)
        scanner_image = self.HSV_Filter(scanner_image)

        # define a threshold, 128 is the middle of black and white in grey scale
        thresh = 15

        # assign blue channel to zeros
        img_binary = cv2.threshold(scanner_image, thresh, 255, cv2.THRESH_BINARY)[1]
        h_fila = 0

        for fila in img_binary:
            wwhite = np.where(np.logical_and(fila != 0, fila == max(fila)))
            if len(wwhite[0]) > 0:
                if len(wwhite[0]) % 2 == 0:
                    pos_pix.append([h_fila, wwhite[0][int(len(wwhite[0]) / 2)]])
                else:
                    pos_pix.append(
                        [h_fila, wwhite[0][int(round(len(wwhite[0]) / 2, 0))]]
                    )

            h_fila += 1

        for px in pos_pix:
            x = self.mtx_xi[px[0], px[1]] * self.scale_factor_x
            y = self.mtx_yi[px[0], px[1]] * self.scale_factor_y
            z = self.mtx_zi[px[0], px[1]] * self.scale_factor_z
            rgb = clr_image[px[0], px[1]]
            r = rgb[2]
            g = rgb[1]
            b = rgb[0]

            if z >= -69 and 180 > abs(x) and 180 > abs(y):
                punto = [x, y, z]

                # Creamos la matriz de rotación
                ang_rad = -np.deg2rad(angulo)

                m_rot = [
                    [np.cos(ang_rad), -np.sin(ang_rad), 0],
                    [np.sin(ang_rad), np.cos(ang_rad), 0],
                    [0, 0, 1],
                ]
                m_inv_rot = np.linalg.inv(m_rot)
                pos_xyz = np.dot(m_inv_rot, punto)
                self.point_cloud.append([pos_xyz[0], pos_xyz[1], pos_xyz[2], r, g, b])
                self.npoints += 1

    # --------------------------------
    # Función para procesar imagen desde Secuencia 2
    
    def PROCESAR_IMAGEN_SEC2(self, image_laser, angulo):
        # lista de posiciones de pixeles x e y
        pos_pix = []
        # Obtenemos la imagen y los datos de altura y ancho en pixeles
        scanner_image = image_laser  # cv2.cvtColor(image_laser, cv2.COLOR_BGR2GRAY)
        h, w = scanner_image.shape[:2]

        # Procesamos la imagen
        scanner_image = self.Calibrate(scanner_image, h, w)
        scanner_image = self.HSV_Filter(scanner_image)

        # define a threshold, 128 is the middle of black and white in grey scale
        thresh = 15

        # assign blue channel to zeros
        img_binary = cv2.threshold(scanner_image, thresh, 255, cv2.THRESH_BINARY)[1]
        h_fila = 0

        for fila in img_binary:
            wwhite = np.where(np.logical_and(fila != 0, fila == max(fila)))
            if len(wwhite[0]) > 0:
                if len(wwhite[0]) % 2 == 0:
                    pos_pix.append([h_fila, wwhite[0][int(len(wwhite[0]) / 2)]])
                else:
                    pos_pix.append(
                        [h_fila, wwhite[0][int(round(len(wwhite[0]) / 2, 0))]]
                    )

            h_fila += 1

        for px in pos_pix:
            x = self.mtx_xi[px[0], px[1]] * self.scale_factor_x
            y = self.mtx_yi[px[0], px[1]] * self.scale_factor_y
            z = self.mtx_zi[px[0], px[1]] * self.scale_factor_z
            r = 255
            g = 0
            b = 0

            if z >= -69 and 180 > abs(x) and 180 > abs(y):
                punto = [x, y, z]

                # Creamos la matriz de rotación
                ang_rad = -np.deg2rad(angulo)

                m_rot = [
                    [np.cos(ang_rad), -np.sin(ang_rad), 0],
                    [np.sin(ang_rad), np.cos(ang_rad), 0],
                    [0, 0, 1],
                ]
                m_inv_rot = np.linalg.inv(m_rot)
                pos_xyz = np.dot(m_inv_rot, punto)
                self.point_cloud.append([pos_xyz[0], pos_xyz[1], pos_xyz[2], r, g, b])
                self.npoints += 1
