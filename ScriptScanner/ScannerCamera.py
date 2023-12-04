import cv2


class Camera:
    def __init__(self, resol_w, resol_h, camera_id=0):
        # Resoluciones en Widht y Height

        self.resol_w = resol_w
        self.resol_h = resol_h

        # Id de cámara

        self.camera_id = camera_id

        # Generamos la cámara y parametrizamos
        try:
            self.camera = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, resol_w)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resol_h)
            self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            self.camera.set(cv2.CAP_PROP_FOCUS, 30)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)

            if self.camera.isOpened():
                print("Cámara desconectada.")
            else:
                print("Cámara no se pudo abrir.")

        except:
            print("Erro: Conectando cámara.")
        # self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)
        # self.camera.set(cv2.CAP_PROP_SATURATION, 255)

    def Take_Photo(self, folder_name, name):
        # Leemos la imagen
        do_read, frame = self.camera.read()
        self.camera.grab()
        frame = self.camera.retrieve()[1]
        file_name = str(name) + ".jpg"

        # Verificamos que realizó la captura de la imagen
        # print("Foto  ----- > " + name)
        # print(str(folder_name) + "/" + file_name)

        if do_read == True:
            # Generamos nombre del archivo

            # Realizamos escritura en el Path Indicado
            try:
                cv2.imwrite(str(folder_name) + "/" + file_name, frame)
                return frame
            except:
                print("Error when save photo")
        else:
            print("Error al acceder a la cámara")

    def Close_Camera(self):
        try:
            if self.camera.isOpened():
                self.camera.release()
                print("Cámara desconectada.")
        except:
            print("Erro: Desconectando cámara.")
