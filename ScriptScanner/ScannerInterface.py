# --------------------------------
# Importamos librearías a utilizar

from PyQt5.QtWidgets import *
from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5 import QtGui
from PyQt5.QtCore import *
import sys
import time as tt
import serial.tools.list_ports
import datetime
from pygrabber.dshow_graph import FilterGraph

import ScannerCamera
import Scanner
import ScannerFunctions
import SurfaceProcess

# --------------------------------
# Creamos la Clase de la Interfaz


class Window(QMainWindow):
    def __init__(self):
        # --------------------------------
        # Esta línea es importante para que funcione el programa
        QMainWindow.__init__(self)
        super(QMainWindow, self).__init__(parent=None)

        # --------------------------------
        # Propiedades de la ventana

        self.setFixedHeight(650)
        self.setFixedWidth(800)
        self.setWindowTitle("Escaner 3D")
        self.setWindowIcon(QtGui.QIcon("ScriptScanner/icono_intefaz.png"))

        # --------------------------------
        # Variables auxiliares

        self.scanner = False
        self.scanner_led_on = False
        self.scanner_laser_on = False
        self.camera_name = "Logitech Webcam C925e"
        self.camera_index = 0
        self.ply_file_folder = ""

        # --------------------------------
        # Se crea el objeto scanner y cámara y secuenciador

        self.ScannerScanner_create()
        self.secuenciador = ScannerFunctions.Secuenciadores(self)
        self.surface_proc = SurfaceProcess.SurfaceGenerator(self)

        # --------------------------------
        # Se agregan todos los widgets

        self.add_tabs()
        self.add_boxes()
        self.add_buttons()
        self.add_labels()
        self.add_inputs()
        self.add_lcds()
        self.add_pbar()
        self.apply_styles()
        # --------------------------------
        # Se crean los hilos para la ejecución en paralelo

        self.Thread_2 = Thread_Scan(self)
        self.Thread_1 = Thread_Timer(self)

    def ScannerScanner_create(self):
        # --------------------------------
        # Se listan los puertos serial y se obtiene el puerto COM del arduino

        try:
            for port in list(serial.tools.list_ports.comports()):
                # Busca el hardware por VID-PID

                if port[2].startswith("USB VID:PID=2341:0043"):
                    com = port[0]  # Se obtiene el puerto COM
            # Creamos el objeto Scanner y luego conectamos con el arduino
            self.scanner = Scanner.ScannerProperties(port=com)
        except:
            print(
                "Error: Conexión/Desconxión a puerto serie no se ejecuta correctamente."
            )

    def ScannerScanner_Connect(self):
        try:
            # --------------------------------
            # Si el scanner no está conectado, realiza la conexión.

            if self.scanner.conectado == False:
                # Realizamos la conexión con el arduino, en caso de ser True la respuesta, setea LED y LASER en OFF y dispone el label en verde de conexión

                resp = self.scanner.CONECTAR_ARDUINO()

                # --------------------------------
                # Si la conexión es exitosa, devuelve TRUE

                if resp:
                    # --------------------------------
                    # Inicio de Cámara

                    self.ScannerCamera_Connect()

                    # --------------------------------
                    # Se apaga LED y Laser

                    self.scanner.LED_OFF()
                    self.scanner.LASER_OFF()

                    # --------------------------------
                    # Cambio de texto de botón de conexión

                    self.btn_Connect.setText("Desconectar Escaner")

                    # --------------------------------
                    # Se establece default el paso microstep=7

                    (
                        self.mstp_actual,
                        self.angular_resolution,
                    ) = self.scanner.SET_MICROPASOS(microstep=7)

                    self.angular_resolution = round(self.angular_resolution, 5)

                    # --------------------------------
                    # Se habilitan los demás widgets

                    self.btn_ScanStart.setEnabled(True)
                    self.btn_ScanStop.setEnabled(False)
                    self.btn_OnOffLed.setEnabled(True)
                    self.btn_OnOffLaser.setEnabled(True)
                    self.btn_StepBase.setEnabled(True)
                    self.btn_UnLockBase.setEnabled(True)
                    self.btn_DoMesh.setEnabled(False)
                    self.btn_SaveMesh.setEnabled(False)
                    self.lcd_clock.setEnabled(True)
                    self.lcd_nimages.setEnabled(True)
                    self.lcd_nang.setEnabled(True)
                    self.lcd_npoints.setEnabled(True)
                    self.inpt_reconstruction_depth.setEnabled(True)
                    self.inpt_octree_depth.setEnabled(True)
                    self.inpt_gradients_depth.setEnabled(True)
                    self.inpt_scale.setEnabled(True)
                    self.inpt_nsamples.setEnabled(True)
                    self.inpt_interp_weight.setEnabled(True)
                    self.inpt_gauss_seidel_relaxations.setEnabled(True)
                    self.inpt_scan_scalex.setEnabled(True)
                    self.inpt_scan_scaley.setEnabled(True)
                    self.inpt_scan_scalez.setEnabled(True)
                    self.inpt_normals_k.setEnabled(True)
                    self.inpt_normals_smoothiter.setEnabled(True)
                    self.list_scan_resolution.setEnabled(True)
                    self.list_scan_secuenc.setEnabled(True)
                    self.pbar.setEnabled(True)

            elif self.scanner.conectado == True:
                # --------------------------------
                # Si el scanner está conectado, se desconecta

                resp = self.scanner.DESCONECTAR_ARDUINO()

                if resp:
                    # --------------------------------
                    # Se cierra la cámara

                    self.camera.Close_Camera()

                    # --------------------------------
                    # Se cambia el texto del botón de conexión

                    self.btn_Connect.setText("Conectar Escaner")

                    # --------------------------------
                    # Se inhabilitan los demás widgets

                    self.btn_ScanStart.setEnabled(False)
                    self.btn_ScanStop.setEnabled(False)
                    self.btn_OnOffLed.setEnabled(False)
                    self.btn_OnOffLaser.setEnabled(False)
                    self.btn_StepBase.setEnabled(False)
                    self.btn_UnLockBase.setEnabled(False)
                    self.btn_DoMesh.setEnabled(False)
                    self.btn_SaveMesh.setEnabled(False)
                    self.lcd_clock.setEnabled(False)
                    self.lcd_nimages.setEnabled(False)
                    self.lcd_nang.setEnabled(False)
                    self.lcd_npoints.setEnabled(False)
                    self.inpt_reconstruction_depth.setEnabled(False)
                    self.inpt_octree_depth.setEnabled(False)
                    self.inpt_gradients_depth.setEnabled(False)
                    self.inpt_scale.setEnabled(False)
                    self.inpt_scan_scalex.setEnabled(False)
                    self.inpt_scan_scaley.setEnabled(False)
                    self.inpt_scan_scalez.setEnabled(False)
                    self.inpt_nsamples.setEnabled(False)
                    self.inpt_interp_weight.setEnabled(False)
                    self.inpt_gauss_seidel_relaxations.setEnabled(False)
                    self.list_scan_resolution.setEnabled(False)
                    self.list_scan_secuenc.setEnabled(False)
                    self.inpt_normals_k.setEnabled(False)
                    self.inpt_normals_smoothiter.setEnabled(False)
                    self.pbar.setEnabled(False)
        except Exception as e:
            print(
                "Error: Conexión/Desconxión a Scanner no se ejecuta correctamente -> "
                + str(e)
            )

    def ScannerCamera_Connect(self):
        # --------------------------------
        # Inicializamos Camara

        try:
            # --------------------------------
            # Se obtiene el index de la cámara correspondiente

            devices = FilterGraph().get_input_devices()

            # --------------------------------
            # Se obtiene el index de la cámara correspondiente

            for device_index, device_name in enumerate(devices):
                # --------------------------------
                # Si coincide con el nombre de la cámara, obtiene el index

                if device_name == self.camera_name:
                    self.camera_index = device_index
                    break
            # --------------------------------
            # Se inicia cámara

            self.camera = ScannerCamera.Camera(1920, 1080, camera_id=self.camera_index)

        except Exception as e:
            print("Error: Con/Desc cámara no se ejecuta correctamente -> " + str(e))

    def Scanner_StopScan(self):
        try:
            # --------------------------------
            # Si se está ejecutando la secuencia, detiene todo

            if self.Thread_2.isRunning():
                self.Thread_2.stop()
                self.Thread_1.stop()

                # --------------------------------
                # Habilitamso botón de start

                self.btn_ScanStart.setEnabled(True)
                self.btn_ScanStop.setEnabled(False)
                self.btn_Connect.setEnabled(True)
                self.btn_ScanStart.setEnabled(True)
                self.btn_UnLockBase.setEnabled(True)
                self.btn_DoMesh.setEnabled(False)
                self.btn_SaveMesh.setEnabled(False)
                self.btn_OnOffLaser.setEnabled(True)
                self.btn_OnOffLed.setEnabled(True)
                self.btn_ScanStop.setEnabled(False)
                self.btn_StepBase.setEnabled(True)
        except Exception as e:
            print("Error: " + str(e))

    def Scanner_StartScan(self):
        try:
            # --------------------------------
            # Si el scanner está conectado, ejecuta

            if self.scanner.conectado:
                # --------------------------------
                # Se crea la carpeta para la nube de puntos y el nombre del archivo
                self.file_folder, self.file_name = self.secuenciador.CREAR_CARPETA()

                # --------------------------------
                # Se toma el parámetro de resolución
                if self.list_scan_resolution.currentIndex() == 0:
                    self.scanner.SET_MICROPASOS(microstep=3)
                elif self.list_scan_resolution.currentIndex() == 1:
                    self.scanner.SET_MICROPASOS(microstep=4)
                elif self.list_scan_resolution.currentIndex() == 2:
                    self.scanner.SET_MICROPASOS(microstep=5)
                elif self.list_scan_resolution.currentIndex() == 3:
                    self.scanner.SET_MICROPASOS(microstep=6)
                elif self.list_scan_resolution.currentIndex() == 4:
                    self.scanner.SET_MICROPASOS(microstep=7)
                elif self.list_scan_resolution.currentIndex() == 5:
                    self.scanner.SET_MICROPASOS(microstep=8)

                # --------------------------------
                # Se inhabilitan los widgets para evitar ejecuciones

                self.btn_ScanStart.setEnabled(False)
                self.btn_StepBase.setEnabled(False)
                self.btn_DoMesh.setEnabled(False)
                self.btn_OnOffLaser.setEnabled(False)
                self.btn_OnOffLed.setEnabled(False)
                self.btn_ScanStop.setEnabled(True)
                self.btn_UnLockBase.setEnabled(False)
                self.btn_Connect.setEnabled(False)
                self.btn_SaveMesh.setEnabled(False)
                self.btn_ScanStop.setEnabled(True)

                # --------------------------------
                # Se inicia tiempo de ejecución de escaneo

                self.start_time = tt.time()
                # --------------------------------
                # Se obtiene el tipo de secuencia que se desea realizar

                self.secuencia = self.list_scan_secuenc.currentIndex()

                # --------------------------------
                # Se inician los hilos

                self.Thread_1.start()
                self.Thread_1.update_m.connect(self.update_metrics)
                self.Thread_2.start()

        except Exception as e:
            print("Error: Inicio de escaneo -> " + str(e))

    def ScannerScanner_Exit(self, QCloseEvent):
        super().closeEvent(QCloseEvent)

        if self.scanner.conectado:
            try:
                self.scanner.LED_OFF()
                self.scanner.LASER_OFF()
                self.scanner.MOTOR_OFF()
                self.scanner.DESCONECTAR_ARDUINO()
                self.camera.Close_Camera()
            except:
                print("Error: Apagando scanner.")

    def add_tabs(self):
        # --------------------------------
        # Pestaña Principal - Escaneo de Pieza

        self.tab_Main = QWidget()
        self.lyt_Main = QGridLayout()
        self.tab_Main.setLayout(self.lyt_Main)

        # --------------------------------
        # Pestaña de Mallado Nube de Puntos

        self.tab_Mesh = QWidget()
        self.lyt_Mesh = QGridLayout()
        self.tab_Mesh.setLayout(self.lyt_Mesh)

        # --------------------------------
        # Contenedor de Pestañas

        self.tab_Container = QTabWidget(self)
        self.tab_Container.setGeometry(0, 0, self.width(), self.height())
        self.tab_Container.addTab(self.tab_Main, "Command Panel")
        self.tab_Container.addTab(self.tab_Mesh, "Mesh Panel")

    def add_boxes(self):
        # --------------------------------
        # Cuadro de Comandos Periféricos Scanner

        self.lyt_gb_Perifericos = QGridLayout()
        self.lyt_gb_Perifericos.setSpacing(5)

        self.gb_Perifericos = QGroupBox("Peripheral Commands")
        self.gb_Perifericos.setLayout(self.lyt_gb_Perifericos)

        # --------------------------------
        # Cuadro de Ejecución Scanner

        self.lyt_gb_ScanComand = QGridLayout()
        self.lyt_gb_ScanComand.setSpacing(5)

        self.gb_ScanComand = QGroupBox("Scan Commands")
        self.gb_ScanComand.setLayout(self.lyt_gb_ScanComand)

        # --------------------------------
        # Cuadro de Métricas escaneo

        self.lyt_gb_Metrics = QGridLayout()
        self.lyt_gb_Metrics.setSpacing(5)

        self.gb_Metrics = QGroupBox("Metrics")
        self.gb_Metrics.setLayout(self.lyt_gb_Metrics)

        # --------------------------------
        # Cuadro de Parámetros de Escaneo

        self.lyt_gb_Options = QGridLayout()
        self.lyt_gb_Options.setSpacing(5)

        self.gb_Options = QGroupBox("Scan Options")
        self.gb_Options.setLayout(self.lyt_gb_Options)

        # --------------------------------
        # Cuadro de Parámetros de Mallado

        self.lyt_gb_Mesh = QGridLayout()
        self.lyt_gb_Mesh.setSpacing(5)

        self.gb_Mesh = QGroupBox("Surface Reconstruction Options")
        self.gb_Mesh.setLayout(self.lyt_gb_Mesh)

        # --------------------------------
        # Cuadro de Parámetros de Mallado

        self.lyt_gb_NormalsMesh = QGridLayout()
        self.lyt_gb_NormalsMesh.setSpacing(5)

        self.gb_NormalsMesh = QGroupBox("Compute Normals")
        self.gb_NormalsMesh.setLayout(self.lyt_gb_NormalsMesh)

        # --------------------------------
        # Cuadro de Parámetros de Mallado

        self.lyt_gb_MeshFile = QGridLayout()
        self.lyt_gb_MeshFile.setSpacing(5)

        self.gb_MeshFile = QGroupBox("Mesh File Options")
        self.gb_MeshFile.setLayout(self.lyt_gb_MeshFile)

        # --------------------------------
        # Agregamos los boxes a los layouts correspondientes

        self.lyt_Main.addWidget(self.gb_Options, 0, 0, 3, 2)
        self.lyt_Main.addWidget(self.gb_Metrics, 0, 2, 3, 3)
        self.lyt_Main.addWidget(self.gb_Perifericos, 3, 0, 1, 2)
        self.lyt_Main.addWidget(self.gb_ScanComand, 3, 2, 1, 3)

        self.lyt_Mesh.addWidget(self.gb_Mesh, 0, 0, 5, 1)
        self.lyt_Mesh.addWidget(self.gb_NormalsMesh, 5, 0, 2, 1)
        self.lyt_Mesh.addWidget(self.gb_MeshFile, 7, 0, 2, 1)

    def add_buttons(self):
        # --------------------------------
        # Botón para conexión/desconexión con escaner

        self.btn_Connect = QPushButton(text="Conectar Escaner")
        self.btn_Connect.setEnabled(True)
        self.btn_Connect.clicked.connect(self.ScannerScanner_Connect)
        self.lyt_gb_ScanComand.addWidget(self.btn_Connect, 0, 0, 1, 1)

        # --------------------------------
        # Botón de inicio de escaneo

        self.btn_ScanStart = QPushButton(text="Iniciar Escaneo")
        self.btn_ScanStart.setEnabled(False)
        self.btn_ScanStart.clicked.connect(self.Scanner_StartScan)
        self.lyt_gb_ScanComand.addWidget(self.btn_ScanStart, 1, 0, 1, 1)

        # --------------------------------
        # Botón de parada de escaneo

        self.btn_ScanStop = QPushButton(text="Detener Escaneo")
        self.btn_ScanStop.setEnabled(False)
        self.btn_ScanStop.clicked.connect(self.Scanner_StopScan)
        self.lyt_gb_ScanComand.addWidget(self.btn_ScanStop, 2, 0, 1, 1)

        # --------------------------------
        # Botón ON_OFF Led

        self.btn_OnOffLed = QPushButton(text="Led Off")
        self.btn_OnOffLed.setEnabled(False)
        self.btn_OnOffLed.clicked.connect(self.turnLedOnOff)
        self.lyt_gb_Perifericos.addWidget(self.btn_OnOffLed, 0, 1, 1, 1)

        # --------------------------------
        # Botón de ON_OFF Laser

        self.btn_OnOffLaser = QPushButton(text="Laser Off")
        self.btn_OnOffLaser.setEnabled(False)
        self.btn_OnOffLaser.clicked.connect(self.turnLaserOnOff)
        self.lyt_gb_Perifericos.addWidget(self.btn_OnOffLaser, 0, 2, 1, 1)

        # --------------------------------
        # Botón avance angular Base

        self.btn_StepBase = QPushButton(text="Rotar Base")
        self.btn_StepBase.setEnabled(False)
        self.btn_StepBase.clicked.connect(self.moveBase)
        self.lyt_gb_Perifericos.addWidget(self.btn_StepBase, 0, 3, 1, 1)

        # --------------------------------
        # Botón de desbloqueo de Base

        self.btn_UnLockBase = QPushButton(text="Liberar Base")
        self.btn_UnLockBase.setEnabled(False)
        self.btn_UnLockBase.clicked.connect(self.scanner.MOTOR_OFF)
        self.lyt_gb_Perifericos.addWidget(self.btn_UnLockBase, 0, 4, 1, 1)

        # --------------------------------
        # Botón para Mallar

        self.btn_DoMesh = QPushButton(text="Generar Superficie")
        self.btn_DoMesh.setEnabled(False)
        self.btn_DoMesh.clicked.connect(self.surface_proc.generate_surface)
        self.lyt_gb_MeshFile.addWidget(self.btn_DoMesh, 0, 0, 1, 1)

        # --------------------------------
        # Botón para SalvarMallado

        self.btn_SaveMesh = QPushButton(text="Salvar Mallado")
        self.btn_SaveMesh.setEnabled(False)
        self.btn_SaveMesh.clicked.connect(self.surface_proc.save_surface)
        self.lyt_gb_MeshFile.addWidget(self.btn_SaveMesh, 1, 0, 1, 1)

    def add_labels(self):
        # --------------------------------
        # Label para contador de imágenes

        self.lbl_nimgs = QLabel()
        self.lbl_nimgs.setText("Imágenes recopiladas")

        # --------------------------------
        # Label para mostrar ángulo barrido

        self.lbl_bang = QLabel()
        self.lbl_bang.setText("Ángulo barrido")

        # --------------------------------
        # Label contabilizar tiempo

        self.lbl_time = QLabel()
        self.lbl_time.setText("Tiempo de escaneo")

        # --------------------------------
        # Label contabilizar la cantidad de puntos obtenidos

        self.lbl_npoints = QLabel()
        self.lbl_npoints.setText("Puntos obtenidos")

        # --------------------------------
        # Label contabilizar la cantidad de puntos obtenidos

        self.lbl_percent = QLabel()
        self.lbl_percent.setText("Escaneado")

        # --------------------------------
        # Se agregan los labels al layout

        self.lyt_gb_Metrics.addWidget(self.lbl_nimgs, 0, 0, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lbl_bang, 1, 0, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lbl_time, 2, 0, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lbl_npoints, 3, 0, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lbl_percent, 4, 0, 1, 1)

        # --------------------------------
        # Labels de parámetros para Mallado

        self.lbl_reconstruction_depth = QLabel(parent=self.gb_Mesh)
        self.lbl_reconstruction_depth.setText("Profundidad de reconstrucción")

        self.lbl_octree_depth = QLabel(parent=self.gb_Mesh)
        self.lbl_octree_depth.setText("Profundidad de octree adaptativa")

        self.lbl_gradients_depth = QLabel(parent=self.gb_Mesh)
        self.lbl_gradients_depth.setText("Profundidad de gradientes conjugados")

        self.lbl_scale = QLabel(parent=self.gb_Mesh)
        self.lbl_scale.setText("Factor de escala")

        self.lbl_nsamples = QLabel(parent=self.gb_Mesh)
        self.lbl_nsamples.setText("Número mínimo de muestras")

        self.lbl_interp_weight = QLabel(parent=self.gb_Mesh)
        self.lbl_interp_weight.setText("Peso de interpolación")

        self.lbl_gauss_seidel_relaxations = QLabel(parent=self.gb_Mesh)
        self.lbl_gauss_seidel_relaxations.setText("Relajaciones de Gauss-Seidel")

        self.lbl_normals_k = QLabel(parent=self.gb_NormalsMesh)
        self.lbl_normals_k.setText("Puntos vecinos")

        self.lbl_normals_smoothiter = QLabel(parent=self.gb_NormalsMesh)
        self.lbl_normals_smoothiter.setText("Iteración de suavizado")

        self.lyt_gb_Mesh.addWidget(self.lbl_reconstruction_depth, 0, 0, 1, 2)
        self.lyt_gb_Mesh.addWidget(self.lbl_octree_depth, 1, 0, 1, 2)
        self.lyt_gb_Mesh.addWidget(self.lbl_gradients_depth, 2, 0, 1, 2)
        self.lyt_gb_Mesh.addWidget(self.lbl_scale, 3, 0, 1, 2)
        self.lyt_gb_Mesh.addWidget(self.lbl_nsamples, 4, 0, 1, 2)
        self.lyt_gb_Mesh.addWidget(self.lbl_interp_weight, 5, 0, 1, 2)
        self.lyt_gb_Mesh.addWidget(self.lbl_gauss_seidel_relaxations, 6, 0, 1, 2)

        self.lyt_gb_NormalsMesh.addWidget(self.lbl_normals_k, 0, 0, 1, 2)
        self.lyt_gb_NormalsMesh.addWidget(self.lbl_normals_smoothiter, 1, 0, 1, 2)

        # --------------------------------
        # Labels para el seteo de configuración de scanneo

        self.lbl_scan_resolution = QLabel(parent=self.gb_Options)
        self.lbl_scan_resolution.setText("Resolución angular")

        self.lbl_scan_secuenc = QLabel(parent=self.gb_Options)
        self.lbl_scan_secuenc.setText("Secuencia de escaneo")

        self.lbl_scan_scalex = QLabel(parent=self.gb_Options)
        self.lbl_scan_scalex.setText("Factor de escala X")
        self.lbl_scan_scaley = QLabel(parent=self.gb_Options)
        self.lbl_scan_scaley.setText("Factor de escala Y")
        self.lbl_scan_scalez = QLabel(parent=self.gb_Options)
        self.lbl_scan_scalez.setText("Factor de escala Z")

        self.lyt_gb_Options.addWidget(self.lbl_scan_resolution, 0, 0, 1, 1)
        self.lyt_gb_Options.addWidget(self.lbl_scan_secuenc, 1, 0, 1, 1)
        self.lyt_gb_Options.addWidget(self.lbl_scan_scalex, 2, 0, 1, 1)
        self.lyt_gb_Options.addWidget(self.lbl_scan_scaley, 3, 0, 1, 1)
        self.lyt_gb_Options.addWidget(self.lbl_scan_scalez, 4, 0, 1, 1)

    def add_inputs(self):
        # --------------------------------
        # Inputs para las opciones de Escaneo

        self.list_scan_resolution = QComboBox()
        self.list_scan_resolution.addItems(
            ["1.8°", "0.9°", "0.45°", "0.225°", "0.1125°", "0.05625°"]
        )
        self.list_scan_resolution.setEnabled(False)

        self.list_scan_secuenc = QComboBox()
        self.list_scan_secuenc.addItems(["Secuencia 1", "Secuencia 2"])
        self.list_scan_secuenc.setEnabled(False)

        self.inpt_scan_scalex = QLineEdit()
        self.inpt_scan_scalex.setValidator(QDoubleValidator())
        self.inpt_scan_scalex.setMaxLength(16)
        self.inpt_scan_scalex.setAlignment(Qt.AlignRight)
        self.inpt_scan_scalex.setEnabled(False)
        self.inpt_scan_scalex.setPlaceholderText("1.138")

        self.inpt_scan_scaley = QLineEdit()
        self.inpt_scan_scaley.setValidator(QDoubleValidator())
        self.inpt_scan_scaley.setMaxLength(16)
        self.inpt_scan_scaley.setAlignment(Qt.AlignRight)
        self.inpt_scan_scaley.setEnabled(False)
        self.inpt_scan_scaley.setPlaceholderText("1.138")

        self.inpt_scan_scalez = QLineEdit()
        self.inpt_scan_scalez.setValidator(QDoubleValidator())
        self.inpt_scan_scalez.setMaxLength(16)
        self.inpt_scan_scalez.setAlignment(Qt.AlignRight)
        self.inpt_scan_scalez.setEnabled(False)
        self.inpt_scan_scalez.setPlaceholderText("1.0")

        self.lyt_gb_Options.addWidget(self.list_scan_resolution, 0, 1, 1, 1)
        self.lyt_gb_Options.addWidget(self.list_scan_secuenc, 1, 1, 1, 1)
        self.lyt_gb_Options.addWidget(self.inpt_scan_scalex, 2, 1, 1, 1)
        self.lyt_gb_Options.addWidget(self.inpt_scan_scaley, 3, 1, 1, 1)
        self.lyt_gb_Options.addWidget(self.inpt_scan_scalez, 4, 1, 1, 1)

        # --------------------------------
        # Inputs de entrada para las opciones de Mallado

        self.inpt_reconstruction_depth = QLineEdit(parent=self.gb_Mesh)
        self.inpt_octree_depth = QLineEdit(parent=self.gb_Mesh)
        self.inpt_gradients_depth = QLineEdit(parent=self.gb_Mesh)
        self.inpt_scale = QLineEdit(parent=self.gb_Mesh)
        self.inpt_nsamples = QLineEdit(parent=self.gb_Mesh)
        self.inpt_interp_weight = QLineEdit(parent=self.gb_Mesh)
        self.inpt_gauss_seidel_relaxations = QLineEdit(parent=self.gb_Mesh)
        self.inpt_normals_k = QLineEdit(parent=self.gb_NormalsMesh)
        self.inpt_normals_smoothiter = QLineEdit(parent=self.gb_NormalsMesh)

        self.inpt_reconstruction_depth.setPlaceholderText("8")
        self.inpt_octree_depth.setPlaceholderText("5")
        self.inpt_gradients_depth.setPlaceholderText("0")
        self.inpt_scale.setPlaceholderText("1.1")
        self.inpt_nsamples.setPlaceholderText("1.5")
        self.inpt_interp_weight.setPlaceholderText("4")
        self.inpt_gauss_seidel_relaxations.setPlaceholderText("8")
        self.inpt_normals_k.setPlaceholderText("10")
        self.inpt_normals_smoothiter.setPlaceholderText("0")

        self.inpt_reconstruction_depth.setEnabled(False)
        self.inpt_octree_depth.setEnabled(False)
        self.inpt_gradients_depth.setEnabled(False)
        self.inpt_scale.setEnabled(False)
        self.inpt_nsamples.setEnabled(False)
        self.inpt_interp_weight.setEnabled(False)
        self.inpt_gauss_seidel_relaxations.setEnabled(False)
        self.inpt_normals_k.setEnabled(False)
        self.inpt_normals_smoothiter.setEnabled(False)

        self.inpt_reconstruction_depth.setValidator(QIntValidator())
        self.inpt_reconstruction_depth.setMaxLength(4)
        self.inpt_reconstruction_depth.setAlignment(Qt.AlignRight)

        self.inpt_octree_depth.setValidator(QIntValidator())
        self.inpt_octree_depth.setMaxLength(4)
        self.inpt_octree_depth.setAlignment(Qt.AlignRight)

        self.inpt_gradients_depth.setValidator(QIntValidator())
        self.inpt_gradients_depth.setMaxLength(4)
        self.inpt_gradients_depth.setAlignment(Qt.AlignRight)

        self.inpt_scale.setValidator(QDoubleValidator())
        self.inpt_scale.setMaxLength(4)
        self.inpt_scale.setAlignment(Qt.AlignRight)

        self.inpt_nsamples.setValidator(QDoubleValidator())
        self.inpt_nsamples.setMaxLength(4)
        self.inpt_nsamples.setAlignment(Qt.AlignRight)

        self.inpt_interp_weight.setValidator(QDoubleValidator())
        self.inpt_interp_weight.setMaxLength(4)
        self.inpt_interp_weight.setAlignment(Qt.AlignRight)

        self.inpt_gauss_seidel_relaxations.setValidator(QIntValidator())
        self.inpt_gauss_seidel_relaxations.setMaxLength(4)
        self.inpt_gauss_seidel_relaxations.setAlignment(Qt.AlignRight)

        self.inpt_normals_k.setValidator(QIntValidator())
        self.inpt_normals_k.setMaxLength(4)
        self.inpt_normals_k.setAlignment(Qt.AlignRight)

        self.inpt_normals_smoothiter.setValidator(QIntValidator())
        self.inpt_normals_smoothiter.setMaxLength(4)
        self.inpt_normals_smoothiter.setAlignment(Qt.AlignRight)

        self.lyt_gb_Mesh.addWidget(self.inpt_reconstruction_depth, 0, 2, 1, 1)
        self.lyt_gb_Mesh.addWidget(self.inpt_octree_depth, 1, 2, 1, 1)
        self.lyt_gb_Mesh.addWidget(self.inpt_gradients_depth, 2, 2, 1, 1)
        self.lyt_gb_Mesh.addWidget(self.inpt_scale, 3, 2, 1, 1)
        self.lyt_gb_Mesh.addWidget(self.inpt_nsamples, 4, 2, 1, 1)
        self.lyt_gb_Mesh.addWidget(self.inpt_interp_weight, 5, 2, 1, 1)
        self.lyt_gb_Mesh.addWidget(self.inpt_gauss_seidel_relaxations, 6, 2, 1, 1)

        self.lyt_gb_NormalsMesh.addWidget(self.inpt_normals_k, 0, 2, 1, 1)
        self.lyt_gb_NormalsMesh.addWidget(self.inpt_normals_smoothiter, 1, 2, 1, 1)

    def add_lcds(self):
        # --------------------------------
        # Se crean los widgets para las métricas

        # --------------------------------
        # LCD para contar imágenes

        self.lcd_nimages = QLCDNumber()
        self.lcd_nimages.setEnabled(False)
        self.lcd_nimages.setDigitCount(8)
        self.lcd_nimages.display("0000")

        # --------------------------------
        # LCD para ángulo actual

        self.lcd_nang = QLCDNumber()
        self.lcd_nang.setEnabled(False)
        self.lcd_nang.setDigitCount(8)
        self.lcd_nang.display("0000")

        # --------------------------------
        # LCD para timer

        self.lcd_clock = QLCDNumber()
        self.lcd_clock.setEnabled(False)
        self.lcd_clock.setDigitCount(8)
        self.lcd_clock.display("00:00:00")

        # --------------------------------
        # LCD para contar puntos

        self.lcd_npoints = QLCDNumber()
        self.lcd_npoints.setEnabled(False)
        self.lcd_npoints.setDigitCount(12)
        self.lcd_npoints.display("000000000000")

        # --------------------------------
        # Se agregan los LCD al Layout de Metricas
        self.lyt_gb_Metrics.addWidget(self.lcd_nimages, 0, 1, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lcd_nang, 1, 1, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lcd_clock, 2, 1, 1, 1)
        self.lyt_gb_Metrics.addWidget(self.lcd_npoints, 3, 1, 1, 1)

    def add_pbar(self):
        # --------------------------------
        # Barra de progeso del escaneo

        self.pbar = QProgressBar()
        self.pbar.setValue(0)
        self.pbar.setRange(0, 100)

        self.lyt_gb_Metrics.addWidget(self.pbar, 4, 1, 1, 1)

    def update_metrics(self, time):
        period_time = round(time - self.start_time, ndigits=0)
        # --------------------------------
        # Se pasa a formato string
        text = str(datetime.timedelta(seconds=period_time))

        # --------------------------------
        # Se muestra en pantalla
        self.lcd_clock.display(text)
        # --------------------------------
        # Se actualiza la barra de progreso -> Angulo actual / 360 * 100
        value = int(
            ((self.secuenciador.ang_actual + self.scanner.actual_anglestep) / 3.6)
        )
        self.pbar.setValue(value)

    def turnLaserOnOff(self):
        try:
            # --------------------------------
            # Si el laser está encendido, lo apaga
            if self.scanner.laser_on:
                self.scanner.LASER_OFF()
                self.btn_OnOffLaser.setText("Laser Off")
            # --------------------------------
            # Si el laser está encendido, lo apaga
            elif self.scanner.laser_on == False:
                self.scanner.LASER_ON()
                self.btn_OnOffLaser.setText("Laser On")
        except Exception as e:
            print("Error: Error ejecución on/off Laser -> " + str(e))

    def turnLedOnOff(self):
        try:
            # --------------------------------
            # Si el laser está encendido, lo apaga
            if self.scanner.led_on:
                self.scanner.LED_OFF()
                self.btn_OnOffLed.setText("Led Off")
            # --------------------------------
            # Si el laser está encendido, lo apaga
            elif self.scanner.led_on == False:
                self.scanner.LED_ON()
                self.btn_OnOffLed.setText("Led On")
        except Exception as e:
            print("Error: Error ejecución on/off Led -> " + str(e))

    def moveBase(self):
        try:
            # --------------------------------
            # Si el scanner está encendido, ejecuta
            if self.scanner.conectado:
                # --------------------------------
                # Configura el paso según el valor establecido en la interfaz
                if self.list_scan_resolution.currentIndex() == 0:
                    self.scanner.SET_MICROPASOS(microstep=3)
                elif self.list_scan_resolution.currentIndex() == 1:
                    self.scanner.SET_MICROPASOS(microstep=4)
                elif self.list_scan_resolution.currentIndex() == 2:
                    self.scanner.SET_MICROPASOS(microstep=5)
                elif self.list_scan_resolution.currentIndex() == 3:
                    self.scanner.SET_MICROPASOS(microstep=6)
                elif self.list_scan_resolution.currentIndex() == 4:
                    self.scanner.SET_MICROPASOS(microstep=7)
                elif self.list_scan_resolution.currentIndex() == 5:
                    self.scanner.SET_MICROPASOS(microstep=8)
                # --------------------------------
                # Mueve base
                self.scanner.MOTOR_DO_STEP()
        except Exception as e:
            print("Error: Error ejecución mover base -> " + str(e))

    def apply_styles(self):
        # --------------------------------
        # Se aplican estilos a los widgets de la interfaz

        gb_estilo_1 = """
            QGroupBox:title {
            font-size: 11px;  
            color: #4E342E;
            font-family: "Rockwell"; 
            }
            QGroupBox {
            font-size: 11px;  
            color: #4E342E;
            padding: 1px;
            border: 1px groove #D7CCC8; 
            font-family: "Rockwell"; 
            border-radius: 5px;
            }
            """
        btn_estilo_1 = """
            QPushButton { 
            background-color: #616161; 
            border-top-style: groove;
            border-bottom-style: groove;
            color: #CCCCCC; 
            text-align: center;
            font-size: 11px;  
            padding: 1px;
            border-radius: 4px;
            font-family: "Tahoma"; 
            }
            QPushButton::hover {
                color: #F4D03F; 
                background-color: #9E9E9E; 
            }
            QPushButton::pressed {
                color: #CCCCCC; 
                background-color: #795548; 
                font-size: 10px;  
            }
            QPushButton::disabled{
                color: #BCAAA4; 
                background-color: #D7CCC8; 
            }
            """

        lcd_estilo_1 = """
            QLCDNumber{
            background-color: #CCCCCC; 
            color: #145A32;
            text-align: center;  
            border: 0px solid black;
            }
            """

        lbl_estilo_1 = """
            QLabel
            {
            background-color: #ACACAC; 
            font-size: 14px;
            color: #212121;
            font-family: "Tw Cen MT"; 
            padding: 1px; 
            border-left: 1px groove black;
            }"""
        aux = """
            QProgressBar
            {
            text-align: center;
            font-size: 14px;
            color: #145A32;
            font-family: "Arial Unicode MS";  
            border: 0px;
            }"""

        pbar_estilo_1 = """
            QProgressBar { 
            text-align: center;
            padding: 1px; 
            background:#CCCCCC;
            border: 0px solid #1E8449;
            font-family: "Arial Unicode MS";  
            border-radius: 4px; 
            color: #145A32;
            }

            QProgressBar::chunk {
            background: #F0F4C3;
            font-family: "Arial Unicode MS";  
            border-radius: 4px; 
            color: #145A32;
            }
            
            """
        inpt_estilo_1 = (
            "QLineEdit {background-color: #CCCCCC;font-family: 'Arial Unicode MS'; }"
        )

        # Estilos aplicados a los Group Boxes

        self.gb_Metrics.setStyleSheet(gb_estilo_1)
        self.gb_MeshFile.setStyleSheet(gb_estilo_1)
        self.gb_Options.setStyleSheet(gb_estilo_1)
        self.gb_Perifericos.setStyleSheet(gb_estilo_1)
        self.gb_ScanComand.setStyleSheet(gb_estilo_1)
        self.gb_Mesh.setStyleSheet(gb_estilo_1)
        self.gb_NormalsMesh.setStyleSheet(gb_estilo_1)

        # Estilos aplicados a los Buttons

        self.btn_UnLockBase.setStyleSheet(btn_estilo_1)
        self.btn_Connect.setStyleSheet(btn_estilo_1)
        self.btn_DoMesh.setStyleSheet(btn_estilo_1)
        self.btn_OnOffLaser.setStyleSheet(btn_estilo_1)
        self.btn_OnOffLed.setStyleSheet(btn_estilo_1)
        self.btn_SaveMesh.setStyleSheet(btn_estilo_1)
        self.btn_StepBase.setStyleSheet(btn_estilo_1)
        self.btn_ScanStart.setStyleSheet(btn_estilo_1)
        self.btn_ScanStop.setStyleSheet(btn_estilo_1)
        self.btn_Connect.setWhatsThis(
            "this is a push button \  linked to clickme function"
        )

        self.lbl_bang.setStyleSheet(lbl_estilo_1)
        self.lbl_nimgs.setStyleSheet(lbl_estilo_1)
        self.lbl_npoints.setStyleSheet(lbl_estilo_1)
        self.lbl_scale.setStyleSheet(lbl_estilo_1)
        self.lbl_scan_secuenc.setStyleSheet(lbl_estilo_1)
        self.lbl_scan_resolution.setStyleSheet(lbl_estilo_1)
        self.lbl_scan_scalex.setStyleSheet(lbl_estilo_1)
        self.lbl_scan_scaley.setStyleSheet(lbl_estilo_1)
        self.lbl_scan_scalez.setStyleSheet(lbl_estilo_1)
        self.lbl_percent.setStyleSheet(lbl_estilo_1)
        self.lbl_time.setStyleSheet(lbl_estilo_1)
        self.lbl_normals_k.setStyleSheet(lbl_estilo_1)
        self.lbl_normals_smoothiter.setStyleSheet(lbl_estilo_1)

        self.lbl_gauss_seidel_relaxations.setStyleSheet(lbl_estilo_1)
        self.lbl_gradients_depth.setStyleSheet(lbl_estilo_1)
        self.lbl_interp_weight.setStyleSheet(lbl_estilo_1)
        self.lbl_nsamples.setStyleSheet(lbl_estilo_1)
        self.lbl_octree_depth.setStyleSheet(lbl_estilo_1)
        self.lbl_reconstruction_depth.setStyleSheet(lbl_estilo_1)

        self.lcd_nang.setStyleSheet(lcd_estilo_1)
        self.lcd_clock.setStyleSheet(lcd_estilo_1)
        self.lcd_nimages.setStyleSheet(lcd_estilo_1)
        self.lcd_npoints.setStyleSheet(lcd_estilo_1)

        self.lcd_nang.setFixedHeight(25)
        self.lcd_clock.setFixedHeight(25)
        self.lcd_nimages.setFixedHeight(25)
        self.lcd_npoints.setFixedHeight(25)

        self.list_scan_resolution.setFixedHeight(25)
        self.list_scan_secuenc.setFixedHeight(25)
        self.list_scan_resolution.setStyleSheet(
            "font-family: 'Arial Unicode MS'; background-color: #CCCCCC;"
        )
        self.list_scan_secuenc.setStyleSheet(
            "font-family: 'Arial Unicode MS'; background-color: #CCCCCC;"
        )

        self.inpt_scan_scalex.setFixedHeight(25)
        self.inpt_scan_scaley.setFixedHeight(25)
        self.inpt_scan_scalez.setFixedHeight(25)

        self.inpt_scan_scalex.setStyleSheet(inpt_estilo_1)
        self.inpt_scan_scaley.setStyleSheet(inpt_estilo_1)
        self.inpt_scan_scalez.setStyleSheet(inpt_estilo_1)

        self.inpt_reconstruction_depth.setStyleSheet(inpt_estilo_1)
        self.inpt_octree_depth.setStyleSheet(inpt_estilo_1)
        self.inpt_gradients_depth.setStyleSheet(inpt_estilo_1)
        self.inpt_scale.setStyleSheet(inpt_estilo_1)
        self.inpt_nsamples.setStyleSheet(inpt_estilo_1)
        self.inpt_interp_weight.setStyleSheet(inpt_estilo_1)
        self.inpt_gauss_seidel_relaxations.setStyleSheet(inpt_estilo_1)
        self.inpt_normals_k.setStyleSheet(inpt_estilo_1)
        self.inpt_normals_smoothiter.setStyleSheet(inpt_estilo_1)

        self.inpt_reconstruction_depth.setFixedWidth(250)
        self.inpt_octree_depth.setFixedWidth(250)
        self.inpt_gradients_depth.setFixedWidth(250)
        self.inpt_scale.setFixedWidth(250)
        self.inpt_nsamples.setFixedWidth(250)
        self.inpt_interp_weight.setFixedWidth(250)
        self.inpt_gauss_seidel_relaxations.setFixedWidth(250)
        self.inpt_normals_k.setFixedWidth(250)
        self.inpt_normals_smoothiter.setFixedWidth(250)

        self.inpt_reconstruction_depth.setFixedHeight(25)
        self.inpt_octree_depth.setFixedHeight(25)
        self.inpt_gradients_depth.setFixedHeight(25)
        self.inpt_scale.setFixedHeight(25)
        self.inpt_nsamples.setFixedHeight(25)
        self.inpt_interp_weight.setFixedHeight(25)
        self.inpt_gauss_seidel_relaxations.setFixedHeight(25)
        self.inpt_normals_k.setFixedHeight(25)
        self.inpt_normals_smoothiter.setFixedHeight(25)

        self.pbar.setStyleSheet(pbar_estilo_1)

        self.lcd_nang.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_clock.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_nimages.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_npoints.setSegmentStyle(QLCDNumber.Flat)

        self.setStyleSheet("background-color: #BDBDBD; font-size: 10px;")


# --------------------------------
# Hilo de ejecución


class Thread_Timer(QThread):
    def __init__(self, scanner):
        super().__init__()
        self.scanner = scanner

    update_m = pyqtSignal(float)

    def run(self):
        # --------------------------------
        # Hilo de ejecución activo
        self.hilo_corriendo = True
        # --------------------------------
        # Mientras el hilo esté activo, ejecuta

        while self.hilo_corriendo:
            # --------------------------------
            # Timer delay
            tt.sleep(0.1)

            # --------------------------------
            # Se obtiene el tiempo actual
            time = tt.time()
            # --------------------------------
            # Se obtiene el tiempo transcurrido
            self.update_m.emit(time)

    def stop(self):
        # --------------------------------
        # Al detenerse el hilo, cambia el estado de la variable y se quita hilo
        self.hilo_corriendo = False

        self.scanner.btn_ScanStart.setEnabled(True)
        self.scanner.btn_StepBase.setEnabled(True)
        self.scanner.btn_DoMesh.setEnabled(True)
        self.scanner.btn_OnOffLaser.setEnabled(True)
        self.scanner.btn_OnOffLed.setEnabled(True)
        self.scanner.btn_ScanStop.setEnabled(False)
        self.scanner.btn_UnLockBase.setEnabled(True)
        self.scanner.btn_Connect.setEnabled(True)
        self.scanner.btn_SaveMesh.setEnabled(True)
        self.quit()


class Thread_Scan(QThread):
    def __init__(self, scanner):
        super().__init__()
        self.scanner = scanner

    # scanning = pyqtSignal()

    def run(self):
        # --------------------------------
        # Hilo de ejecución activo

        self.hilo_corriendo = True
        # --------------------------------
        # Se ejecuta la secuencia indicada por el usuario
        if self.scanner.secuencia == 0:
            self.scanner.secuenciador.SECUENCIA1()
            # --------------------------------
            # Al finalizar la secuencia, frena el hilo del timer
            self.scanner.Thread_1.stop()
            self.stop()
        elif self.scanner.secuencia == 1:
            self.scanner.secuenciador.SECUENCIA2()
            # --------------------------------
            # Al finalizar la secuencia, frena el hilo del timer
            self.scanner.Thread_1.stop()
            self.stop()

    def stop(self):
        # --------------------------------
        # Al detenerse el hilo, se setean los widgets habilitados para poder ser usados por el usuario

        self.hilo_corriendo = False
        # --------------------------------
        # Se elimina hilo

        self.quit()


# --------------------------------
# Se ejecuta la interfaz

if __name__ == "__main__":
    App = QApplication(sys.argv)
    App.setStyle("Fusion")

    w = Window()

    w.show()  # Mostramos la ventana

    sys.stdout.flush()
    sys.exit(App.exec_())
