"""
Universidad Nacional de Lomas de Zamora
Facultad de Ingeniería
Proyecto Final - Ingeniería Mecatrónica - Escaner 3D por Triangulación Láser

Integrantes: 
- Alexander Dalle Rive Prieto
- Matías Nicolás Labreñiuk

Introducción al script:

En el presente script, se encuentra la programación de la interfaz gráfica para la ejecución del scanner.
Se crean los diferentes widgets (botones, entrada de datos, etiquetas, estilos, etc.)


"""

# --------------------------------
# Importamos librearías a utilizar

import cv2


# --------------------------------
# Creamos la Clase de la Cámara


class Camera:
    # --------------------------------
    # Función de inicialización de la clase Cámara, se declaran las variables iniciales

    def __init__(self, resol_w, resol_h, camera_id=0):
        # --------------------------------
        # Resolución ancho y alto de cámara

        self.resol_w = resol_w
        self.resol_h = resol_h

        # --------------------------------
        # Index de Cámara

        self.camera_id = camera_id

        # --------------------------------
        # Creamos la cámara y parametrizamos
        try:
            self.camera = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, resol_w)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resol_h)
            self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            self.camera.set(cv2.CAP_PROP_FOCUS, 30)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)

            if self.camera.isOpened():
                print("Cámara conectada.")
            else:
                print("Cámara no se pudo abrir.")

        except:
            print("Error: Conectando cámara.")
        # self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)
        # self.camera.set(cv2.CAP_PROP_SATURATION, 255)

    def Take_Photo(self, folder_name, name):
        # --------------------------------
        # Leemos la imagen

        do_read, frame = self.camera.read()
        self.camera.grab()
        frame = self.camera.retrieve()[1]

        # --------------------------------
        # Declaramos el nombre del archivo

        file_name = str(name) + ".jpg"

        # Verificamos que realizó la captura de la imagen
        # print("Foto  ----- > " + name)
        # print(str(folder_name) + "/" + file_name)

        # --------------------------------
        # Si la cámara leyó correctamente la imagen, ejecuta

        if do_read == True:
            # --------------------------------
            # Realizamos la escritura en el folder indicado
            try:
                cv2.imwrite(str(folder_name) + "/" + file_name, frame)
                return frame
            except:
                print("Error when save photo")
        else:
            print("Error al acceder a la cámara")

    # --------------------------------
    # Esta función cierra la cámara
    def Close_Camera(self):
        try:
            # --------------------------------
            # Si la cámara se encuentra en uso, ejecuta el cierre
            if self.camera.isOpened():
                self.camera.release()
                print("Cámara desconectada.")
        except:
            print("Erro: Desconectando cámara.")
