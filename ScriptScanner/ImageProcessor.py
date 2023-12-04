# Importamos librerías
import numpy as np
import cv2
import time
import os
from pyntcloud import PyntCloud
import pandas as pd


class ImagePro:
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

    # Filtro HSV para tomar los pixeles en rojo

    def HSV_Filter(self, image=None):
        # img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # lower_red = np.array([0, 0, 25])
        # upper_red = np.array([200, 150, 255])

        # mask = cv2.inRange(img_hsv, lower_red, upper_red)

        # res = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

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

    # Calibración de Imagen

    def Calibrate(self, imagen, h, w):
        "PROPIEDADES INTRÍNSECAS DE CÁMARA"
        # Matriz intrínseca de Cámara
        M_int = np.array(
            [
                [1.42368362e03, 0.00000000e00, 9.50043678e02],
                [0.00000000e00, 1.42475207e03, 5.81229847e02],
                [0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )

        # Matríz de Distorsión
        D = np.array([0.06048523, -0.27447534, -0.00033107, -0.00219491, 0.22280941])

        # Matriz Transformación de Cámara a Pixel

        # U = np.append(M_int, np.zeros(3).resRes_verape(3, 1))
        U = M_int

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            M_int, D, (w, h), 1, (w, h), centerPrincipalPoint=False
        )
        dst = cv2.undistort(imagen, M_int, D, None, newcameramtx)

        return dst

    # Función para crear archivo .ply

    def CREATE_PLY(self, pts, write_text):
        # ref: https://pyntcloud.readthedocs.io/en/latest/io.html
        # the doc is scarce and not complete

        n = len(pts)

        # The points must be written as a dataframe,
        # ref: https://stackoverflow.com/q/70304087/6064933
        if n > 0:
            data = {
                "x": pts[:, 0],
                "y": pts[:, 1],
                "z": pts[:, 2],
                "red": np.uint8(pts[:, 3]),
                "green": np.uint8(pts[:, 4]),
                "blue": np.uint8(pts[:, 5]),
            }

            # build a cloud
            cloud = PyntCloud(pd.DataFrame(data))

            # the argument for writing ply file can be found in
            # https://github.com/daavoo/pyntcloud/blob/7dcf5441c3b9cec5bbbfb0c71be32728d74666fe/pyntcloud/io/ply.py#L173
            cloud.to_file(
                "EscaneosNubeDePuntos/" + str(write_text) + str(".ply"),
                as_text=write_text,
            )
        return "EscaneosNubeDePuntos/" + str(write_text) + str(".ply")

    """ --- GENERACIÓN DE NUBE DE PUNTOS --- """

    def GENERAR_POINT_CLOUD(self, images_path, pcloud_name):
        # Listamos todas las imagenes

        files_names = os.listdir(images_path)

        # Iniciamos contador de tiempo para consultar tiempo total de procesamiento

        start_time = time.time()

        # Por cada imagen del directorio, procesamos y guardamos en la nube de puntos

        # Lista de puntos espaciales para la nube de puntos

        puntos_espaciales = []

        for file_name in files_names:
            if file_name[:3] == "Izq":
                lado = file_name[:3]
                angulo = float(file_name[4:-4])
                # lista de posiciones de pixeles x e y
                pos_pix = []
                image_path = images_path + "/" + file_name
                print(
                    "Imagen correspondiente al ángulo -> "
                    + str(np.around(angulo, 1))
                    + " - "
                    + lado
                )
                # Obtenemos la imagen y los datos de altura y ancho en pixeles

                scanner_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

                clr_image = cv2.imread(images_path + "/Color " + str(file_name[4:]))
                clr_image = np.array(clr_image)
                if scanner_image is None:
                    print("No existe imagen de ángulo -> " + str(np.round(angulo, 1)))
                    continue
                h, w = scanner_image.shape[:2]

                # Procesamos la imagen
                scanner_image = self.Calibrate(scanner_image, h, w)

                # define a threshold, 128 is the middle of black and white in grey scale
                thresh = 15

                # assign blue channel to zeros
                img_binary = cv2.threshold(
                    scanner_image, thresh, 255, cv2.THRESH_BINARY
                )[1]
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
                    x = self.mtx_xi[px[0], px[1]] * 1.5256
                    y = self.mtx_yi[px[0], px[1]] * 1.5256
                    z = self.mtx_zi[px[0], px[1]]
                    rgb = clr_image[px[0], px[1]]
                    r = rgb[2]
                    g = rgb[1]
                    b = rgb[0]

                    if z >= -66 and 180 > abs(x) and 180 > abs(y):
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
                        puntos_espaciales.append(
                            [pos_xyz[0], pos_xyz[1], pos_xyz[2], r, g, b]
                        )

        puntos_espaciales = np.array(puntos_espaciales)

        file_path = self.use_pyntcloud(np.array(puntos_espaciales), "pcloud_name")

        return file_path

    def PROCESAR_IMAGEN_SEC1(self, image_laser, image_led, angulo):
        # lista de posiciones de pixeles x e y
        pos_pix = []
        # Obtenemos la imagen y los datos de altura y ancho en pixeles
        scanner_image = cv2.cvtColor(image_laser, cv2.COLOR_BGR2GRAY)
        h, w = scanner_image.shape[:2]
        clr_image = np.array(image_led)

        # Procesamos la imagen
        scanner_image = self.Calibrate(scanner_image, h, w)

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

    def PROCESAR_IMAGEN_SEC2(self, image_laser, angulo):
        # lista de posiciones de pixeles x e y
        pos_pix = []
        # Obtenemos la imagen y los datos de altura y ancho en pixeles
        scanner_image = cv2.cvtColor(image_laser, cv2.COLOR_BGR2GRAY)
        h, w = scanner_image.shape[:2]

        # Procesamos la imagen
        scanner_image = self.Calibrate(scanner_image, h, w)

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

    def PROCESS(self, image_laser, image_led, angulo):
        # lista de posiciones de pixeles x e y
        pos_pix = []
        print("Imagen correspondiente al ángulo -> " + str(np.around(angulo, 1)))
        # Obtenemos la imagen y los datos de altura y ancho en pixeles

        image_led = np.array(image_led)

        h, w = image_laser.shape[:2]

        # Procesamos la imagen
        image_laser = self.Calibrate(image_laser, h, w)
        image_laser = self.HSV_Filter(image_laser)

        # define a threshold, 128 is the middle of black and white in grey scale
        thresh = 25

        # assign blue channel to zeros
        img_binary = cv2.threshold(image_laser, thresh, 255, cv2.THRESH_BINARY)[1]
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
            rgb = image_led[px[0], px[1]]
            r = rgb[2]
            g = rgb[1]
            b = rgb[0]

            if z >= -66 and 180 > abs(x) and 180 > abs(y):
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
