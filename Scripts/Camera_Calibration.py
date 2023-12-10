"""
Universidad Nacional de Lomas de Zamora
Facultad de Ingeniería
Proyecto Final - Ingeniería Mecatrónica - Escaner 3D por Triangulación Láser

Integrantes: 
- Alexander Dalle Rive Prieto
- Matías Nicolás Labreñiuk

Introducción al script:

En el presente script, se encuentra la programación para obtener las matrices de calibración de la cámara a utilizar en el scanner.

"""

# Importamos Liberías

import cv2
import numpy as np
import glob


# --------------------------------
# Definimos las dimensiones de la tabla de ajedrez de prueba

CHECKERBOARD = (6, 8)  # 6 puntos verticales y 8 puntos horizontales

# --------------------------------
# Se establece criterio máximo de iteración y error

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.0001)

# --------------------------------
# Vector para almacenar cada vector 3D en la proyección de la imagen

objpoints = []

# --------------------------------
# Se establece criterio máximo de iteración y error
# Vector para almacenar cada vector 2D en la proyección de la imagen

imgpoints = []


# --------------------------------
# Se definen las coordenadas del espacio para los puntos en 3D

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# --------------------------------
# Carpeta donde se encuentran las fotos de calibración

images = glob.glob("Fotos/*.jpg")
cont = 0

# --------------------------------
# Se itera cada imagen

for fname in images:
    # --------------------------------
    # Se lee imagen
    img = cv2.imread(fname)
    # --------------------------------
    # Se convierte a escalas de grises
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # --------------------------------
    # Se identifican las esquinas de la tabla de ajedrez
    # Si las encuentra, devuelve True

    ret, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_FAST_CHECK
        + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )

    if ret == True:
        # --------------------------------
        # Agrega los puntos al vector

        objpoints.append(objp)

        # --------------------------------
        # Refina los puntos para los vectores 2D
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # --------------------------------
        # Se agregan a los puntos de imagen
        imgpoints.append(corners2)

        # --------------------------------
        # Dibuja las esquinas sobre la imagen

        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        # --------------------------------
        # Guarda las imagenes en la carpeta nuevamente
        # cv2.imwrite(str("Fotos/") + str(cont) + ".jpg", img)

        cont += 1

        # --------------------------------
        # Muestra las imagenes

        cv2.imshow("img", img)
        cv2.waitKey(10)


# --------------------------------
# Cierra todas las ventanas

cv2.destroyAllWindows()

# --------------------------------
# Obtiene el tamaño de imagen

h, w = img.shape[:2]

"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""

# --------------------------------
# Calibra la cámara pasando como parámetros, los diferentes vectores 3D y 2D de las imágenes procesadas

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# --------------------------------
# Calcula el error

mean_error = 0
tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

# --------------------------------
# Muestra los datos

print("total error: " + str(mean_error / len(objpoints)))

print("Camera matrix : \n")
print(mtx)

print("dist : \n")
print(dist)

print("rvecs : \n")
print(rvecs)

print("tvecs : \n")
print(tvecs)

# --------------------------------
# Calibra todas las imagenes de la carpeta para obtener el resultado final

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
    mtx, dist, (w, h), 1, (w, h), centerPrincipalPoint=False
)
cont = 0
for fname in images:
    img = cv2.imread(fname)
    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    dst = dst[y : y + h, x : x + w]
    cont += 1
    cv2.imwrite(str(cont) + ".jpg", dst)
