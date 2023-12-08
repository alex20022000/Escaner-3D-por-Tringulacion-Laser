"""
Universidad Nacional de Lomas de Zamora
Facultad de Ingeniería
Proyecto Final - Ingeniería Mecatrónica - Escaner 3D por Triangulación Láser

Integrantes: 
- Alexander Dalle Rive Prietto
- Matías Nicolás Labreñiuk

Introducción al script:

En el presente script, se encuentra la programación de las secuencias de escaneo

"""

# --------------------------------
# Importamos librerías

import time
import os
import errno
import Scanner_ImageProcessor
import numpy as np

# --------------------------------
# Creamos la Clase del Secuenciador


class Secuenciadores:
    def __init__(self, scanner):
        # --------------------------------
        # Variables para ambas imágenes -> laser y color

        self.frame_laser = []
        self.frame_color = []

        # --------------------------------
        # Objeto para procesar las imágenes

        self.procesador_imagenes = Scanner_ImageProcessor.ImagePro()

        # --------------------------------
        # Objeto Scanner

        self.scanner = scanner

        # --------------------------------
        # Variables auxiliares para la ejecución

        self.ang_actual = 0  # Angulo de barrido actual
        self.pasos = 0  # Pasos a realizar en la secuencia
        self.n_imgs = 0  # Cantidad de imágenes recopiladas en la ejecución

    # --------------------------------
    # Esta función ejecuta la secuencia 1
    def SECUENCIA1(self):
        # --------------------------------
        # Se calculan los pasos a realizar por la base para el escaneado completo de la pieza

        self.pasos = int(360 / self.scanner.scanner.actual_anglestep)

        # --------------------------------
        # Se setean las variables

        self.n_imgs = 0
        self.procesador_imagenes.npoints = 0
        self.procesador_imagenes.point_cloud = []

        # --------------------------------
        # Se obtienen los datos de las opciones de escaneo
        if self.scanner.inpt_scan_scalex.text() != "":
            self.procesador_imagenes.scale_factor_x = float(
                self.scanner.inpt_scan_scalex.text()
            )
        else:
            self.procesador_imagenes.scale_factor_x = 1.138507462686567

        if self.scanner.inpt_scan_scaley.text() != "":
            self.procesador_imagenes.scale_factor_y = float(
                self.scanner.inpt_scan_scaley.text()
            )
        else:
            self.procesador_imagenes.scale_factor_y = 1.138507462686567

        if self.scanner.inpt_scan_scalez.text() != "":
            self.procesador_imagenes.scale_factor_z = float(
                self.scanner.inpt_scan_scalez.text()
            )
        else:
            self.procesador_imagenes.scale_factor_z = 1.0

        print("Escalado en x -> " + str(self.procesador_imagenes.scale_factor_x))
        print("Escalado en y -> " + str(self.procesador_imagenes.scale_factor_y))
        print("Escalado en z -> " + str(self.procesador_imagenes.scale_factor_z))

        # --------------------------------
        # Se establece inicialmente el laser y led apagados

        self.scanner.scanner.LASER_OFF()
        self.scanner.scanner.LED_OFF()

        # --------------------------------
        # Se ejecuta el ciclo for, recorriendo los pasos completos

        for i in range(0, self.pasos):
            # --------------------------------
            # Si los hilos se están ejecutando, entonces continúa con el escanneo

            if (
                self.scanner.Thread_1.hilo_corriendo == True
                and self.scanner.Thread_2.hilo_corriendo == True
            ):
                # --------------------------------
                # Limpiamos le buffer del arduino

                self.scanner.scanner.arduino.flush()

                # --------------------------------
                # Obtenemos el ángulo actual de barrido

                self.ang_actual = round(self.scanner.scanner.actual_anglestep * i, 8)

                # --------------------------------
                # Mostramos en pantalla el ángulo actual + desfase

                self.scanner.lcd_nang.display(
                    str(
                        round(
                            self.ang_actual + self.scanner.scanner.actual_anglestep, 2
                        )
                    )
                )

                print("\nFoto: " + str(i) + " | Angulo: " + str(self.ang_actual))

                # --------------------------------
                # Se enciende laser y se apaga led

                self.scanner.scanner.LASER_ON()
                self.scanner.scanner.LED_OFF()
                time.sleep(0.5)

                # --------------------------------
                # Se toma imagen con laser

                image_laser = self.scanner.camera.Take_Photo(
                    folder_name=self.scanner.file_folder,
                    name="Izq " + str(self.ang_actual),
                )

                # --------------------------------
                # Se agrega al contado de imagenes
                self.n_imgs += 1

                # --------------------------------
                # Se apaga laser y se prende led

                self.scanner.scanner.LASER_OFF()
                self.scanner.scanner.LED_ON()
                time.sleep(0.5)

                # --------------------------------
                # Se toma imagen con led

                image_led = self.scanner.camera.Take_Photo(
                    folder_name=self.scanner.file_folder,
                    name="Color " + str(self.ang_actual),
                )

                # --------------------------------
                # Se agrega al contado de imagenes

                self.n_imgs += 1

                # --------------------------------
                # Se muestra en pantalla la cantidad de imagenes obtenidas

                self.scanner.lcd_nimages.display(str(self.n_imgs))

                # --------------------------------
                # Se procesa la imagen y se obtienen los puntos

                self.procesador_imagenes.PROCESAR_IMAGEN_SEC1(
                    image_laser, image_led, self.ang_actual
                )

                # --------------------------------
                # Se muestra en pantalla la cantidad de puntos obtenidos

                self.scanner.lcd_npoints.display(str(self.procesador_imagenes.npoints))

                # --------------------------------
                # Se apaga LED y se gira base

                self.scanner.scanner.LED_OFF()
                self.scanner.scanner.MOTOR_DO_STEP()
            else:
                # --------------------------------
                # Si los hilos no están corriendo, se prende el led y se apaga laser

                self.scanner.scanner.LED_ON()
                self.scanner.scanner.LASER_OFF()
                break

        # --------------------------------
        # Si se termina la ejecución, se enciende led y se apaga laser

        self.scanner.scanner.LED_ON()
        self.scanner.scanner.LASER_OFF()

        # --------------------------------
        # Se crea el archivo .ply para la obtención de la nube de puntos
        try:
            self.scanner.ply_file_folder = self.procesador_imagenes.CREATE_PLY(
                pts=np.array(self.procesador_imagenes.point_cloud),
                write_text=self.scanner.file_name,
            )

        except Exception as e:
            print("Error: Guardar .ply Point Cloud -> " + str(e))
        return True

    # --------------------------------
    # Esta función ejecuta la secuencia 2
    def SECUENCIA2(self):
        # --------------------------------
        # Se calculan los pasos a realizar por la base para el escaneado completo de la pieza

        self.pasos = int(360 / self.scanner.scanner.actual_anglestep)

        # --------------------------------
        # Se setean las variables

        self.n_imgs = 0
        self.procesador_imagenes.npoints = 0
        self.procesador_imagenes.point_cloud = []

        # --------------------------------
        # Se obtienen los datos de las opciones de escaneo
        if self.scanner.inpt_scan_scalex.text() != "":
            self.procesador_imagenes.scale_factor_x = float(
                self.scanner.inpt_scan_scalex.text()
            )
        else:
            self.procesador_imagenes.scale_factor_x = 1.138507462686567

        if self.scanner.inpt_scan_scaley.text() != "":
            self.procesador_imagenes.scale_factor_y = float(
                self.scanner.inpt_scan_scaley.text()
            )
        else:
            self.procesador_imagenes.scale_factor_y = 1.138507462686567

        if self.scanner.inpt_scan_scalez.text() != "":
            self.procesador_imagenes.scale_factor_z = float(
                self.scanner.inpt_scan_scalez.text()
            )
        else:
            self.procesador_imagenes.scale_factor_z = 1.0

        print("Escalado en x -> " + str(self.procesador_imagenes.scale_factor_x))
        print("Escalado en y -> " + str(self.procesador_imagenes.scale_factor_y))
        print("Escalado en z -> " + str(self.procesador_imagenes.scale_factor_z))

        # --------------------------------
        # Se establece incialmente el laser y led apagados

        self.scanner.scanner.LASER_ON()
        self.scanner.scanner.LED_OFF()

        # --------------------------------
        # Se ejecuta el ciclo for, recorriendo los pasos completos

        for i in range(0, self.pasos):
            # --------------------------------
            # Si los hilos se están ejecutando, entonces continúa con el escanneo
            if (
                self.scanner.Thread_1.hilo_corriendo == True
                and self.scanner.Thread_2.hilo_corriendo == True
            ):
                # --------------------------------
                # Limpiamos le buffer del arduino

                self.scanner.scanner.arduino.flush()

                # --------------------------------
                # Obtenemos el ángulo actual de barrido

                self.ang_actual = round(self.scanner.scanner.actual_anglestep * i, 8)

                # --------------------------------
                # Mostramos en pantalla el ángulo actual + desfase

                self.scanner.lcd_nang.display(
                    str(
                        round(
                            self.ang_actual + self.scanner.scanner.actual_anglestep, 2
                        )
                    )
                )

                print("\nFoto: " + str(i) + " | Angulo: " + str(self.ang_actual))

                # --------------------------------
                # Se toma imagen con laser

                image_laser = self.scanner.camera.Take_Photo(
                    folder_name=self.scanner.file_folder,
                    name="Izq " + str(self.ang_actual),
                )

                # --------------------------------
                # Se agrega al contado de imagenes
                self.n_imgs += 1

                # --------------------------------
                # Se procesa la imagen y se obtienen los puntos

                self.procesador_imagenes.PROCESAR_IMAGEN_SEC2(
                    image_laser, self.ang_actual
                )

                # --------------------------------
                # Se muestra en pantalla la cantidad de puntos obtenidos

                self.scanner.lcd_npoints.display(str(self.procesador_imagenes.npoints))

                # --------------------------------
                # Se muestra en pantalla la cantidad de imagenes obtenidas

                self.scanner.lcd_nimages.display(str(self.n_imgs))

                # --------------------------------
                # Se apaga LED y se gira base

                self.scanner.scanner.MOTOR_DO_STEP()
            else:
                # --------------------------------
                # Si los hilos no están corriendo, se prende el led y se apaga laser

                self.scanner.scanner.LED_ON()
                self.scanner.scanner.LASER_OFF()
                break

        # --------------------------------
        # Si se termina la ejecución, se enciende led y se apaga laser

        self.scanner.scanner.LED_ON()
        self.scanner.scanner.LASER_OFF()

        # --------------------------------
        # Se crea el archivo .ply para la obtención de la nube de puntos

        try:
            self.scanner.ply_file_folder = self.procesador_imagenes.CREATE_PLY(
                pts=np.array(self.procesador_imagenes.point_cloud),
                write_text=self.scanner.file_name,
            )

        except Exception as e:
            print("Error: Guardar .ply Point Cloud -> " + str(e))
        return True

    # --------------------------------
    # Esta función crea la carpeta para almacenar las imágenes de escaneo
    def CREAR_CARPETA(self):
        # --------------------------------
        # Creamos carpeta para guardar las imágenes
        cond = True
        cont = 1

        while cond:
            try:
                # --------------------------------
                # Busca las carpetas con el mismo nombre
                carpeta = "Imagenes Scanneo - " + str(cont)
                # --------------------------------
                # Si al crear la carpeta, verifica que existe, entonces agrega a la variable "cont" +1 y vuelve a validar si existe o no
                os.mkdir(carpeta)
                cond = False
                # --------------------------------
                # Si al crear la carpeta, se ejecuta Ok, entonces devuelve la dirección y la variable "cont"
                return carpeta, cont
            except OSError as e:
                # --------------------------------
                # Si al crear la carpeta, verifica que existe, entonces agrega a la variable "cont" +1 y vuelve a validar si existe o no
                cont += 1
                if e.errno != errno.EEXIST:
                    raise
