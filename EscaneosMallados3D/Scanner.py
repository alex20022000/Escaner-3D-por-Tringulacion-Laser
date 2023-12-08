# Importamos librerías

import serial
import struct
import time

# Control de Perifericos Scanner


class ScannerProperties:
    def __init__(self, baudrate=9600, rtscts=True, dsrdtr=True, port="COM3"):
        """
        Parámetros de conexión con el Módulo ARDUINO
        """
        self.baudrate = baudrate
        self.rtscts = rtscts
        self.dsrdtr = dsrdtr
        self.port = port
        self.conectado = False
        self.led_on = False
        self.laser_on = False

        """
        Configuración inicial de micropasos para el Motor
        """
        self.actual_microstep = "Full Step -> 1.8°"
        self.actual_anglestep = 1.8

    def CONECTAR_ARDUINO(self):
        # Se realiza conexión con arduino, caso de fallar, se explicita con un mensaje para que el usuario verifique la configuración

        try:
            self.arduino = serial.Serial(
                port=self.port, baudrate=9600, rtscts=True, dsrdtr=True
            )

            if self.arduino.is_open:
                print("Conexión establecida -> Ok")
                self.conectado = True
                time.sleep(2)
                self.SET_MICROPASOS(microstep=5)

                return True
        except serial.serialutil.SerialException as e:
            print("Error: " + str(e))
            self.conectado = False
            return False

    def DESCONECTAR_ARDUINO(self):
        if self.arduino.is_open:
            self.arduino.close()
            self.conectado = False
            print("ARDUINO -> Desconectado")

        return True

    def SET_MICROPASOS(self, microstep=6):
        if microstep == 3:
            self.SEND_TO_ARDUINO(msg=3)
            self.actual_microstep = "1 Step -> 1.8°"
            self.actual_anglestep = 1.8
        elif microstep == 4:
            self.SEND_TO_ARDUINO(msg=4)
            self.actual_microstep = "1/2 Step -> 0.9°"
            self.actual_anglestep = 0.9
        elif microstep == 5:
            self.SEND_TO_ARDUINO(msg=5)
            self.actual_microstep = "1/4 Step -> 0.45°"
            self.actual_anglestep = 0.45
        elif microstep == 6:
            self.SEND_TO_ARDUINO(msg=6)
            self.actual_microstep = "1/8 Step -> 0.225°"
            self.actual_anglestep = 0.225
        elif microstep == 7:
            self.SEND_TO_ARDUINO(msg=7)
            self.actual_microstep = "1/16 Step -> 0.1125°"
            self.actual_anglestep = 0.1125
        elif microstep == 8:
            self.SEND_TO_ARDUINO(msg=8)
            self.actual_microstep = "1/32 Step -> 0.05625°"
            self.actual_anglestep = 0.05625
        else:
            print(
                """\nLa opción a enviar no es válida, recuerde que las disponibles son:  
                    3 -> Indica paso 1    - Full Step (1.8°)
                    4 -> Indica paso 1/2  - Half Step
                    5 -> Indica paso 1/4
                    6 -> Indica paso 1/8
                    7 -> Indica paso 1/16
                    8 -> Indica paso 1/32              
                    """
            )

        return self.actual_microstep, self.actual_anglestep

    def MOTOR_OFF(self):
        if self.SEND_TO_ARDUINO(msg=9) == 1:
            print("MOTOR -> OFF")
        else:
            print("ERROR !!! MOTOR FAIL")

    # --------------------------------
    # Función para apagado/encendido del LASER

    def LASER_ONOFF(self):
        try:
            # --------------------------------
            # Si el LASER está encendido, lo apaga
            if self.laser_on == True:
                # --------------------------------
                # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
                if self.SEND_TO_ARDUINO(msg=10) == 1:
                    print("LASER -> OFF")
                    self.laser_on = False
                else:
                    print("Error: Apagado Laser.")
            # --------------------------------
            # Si el LASER está apagado, lo enciende
            elif self.laser_on == False:
                # --------------------------------
                # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
                if self.SEND_TO_ARDUINO(msg=11) == 1:
                    print("LASER -> ON")
                    self.laser_on = True
                else:
                    print("Error: Encendido Laser.")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para apagado del LASER

    def LASER_OFF(self):
        try:
            # --------------------------------
            # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
            if self.SEND_TO_ARDUINO(msg=10) == 1:
                print("LASER -> OFF")
                self.laser_on = False
            else:
                print("Error: Apagado Laser.")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para encendido del LASER

    def LASER_ON(self):
        try:
            # --------------------------------
            # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
            if self.SEND_TO_ARDUINO(msg=11) == 1:
                print("LASER -> ON")
                self.laser_on = True
            else:
                print("Error: Encendido Laser.")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para apagado/encendido del LED

    def LED_ONOFF(self):
        try:
            # --------------------------------
            # Si el LED está encendido, lo apaga

            if self.led_on == True:
                # --------------------------------
                # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
                if self.SEND_TO_ARDUINO(msg=13) == 1:
                    print("LED -> OFF")
                    self.led_on = False
                else:
                    print("Error: Apagado LED.")
            # --------------------------------
            # Si el LED está apagado, lo enciende
            elif self.led_on == False:
                # --------------------------------
                # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
                if self.SEND_TO_ARDUINO(msg=12) == 1:
                    print("LED -> ON")
                    self.led_on = True
                else:
                    print("Error: Encendido LED.")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para apagado del LED

    def LED_OFF(self):
        try:
            # --------------------------------
            # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
            if self.SEND_TO_ARDUINO(msg=13) == 1:
                print("LED -> OFF")
                self.led_on = False
            else:
                print("Error: Apagado LED.")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para encendido del LED

    def LED_ON(self):
        try:
            # --------------------------------
            # Envia el comando a arduino y si recibe como respuesta 1, se valida la ejecución.
            if self.SEND_TO_ARDUINO(msg=12) == 1:
                print("LED -> ON")
                self.led_on = True
            else:
                print("Error: Encendido LED.")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para mover base

    def MOTOR_DO_STEP(self):
        try:
            # --------------------------------
            # Se envia comando para mover la base, si devuelve 1, se valida ejecución
            if self.SEND_TO_ARDUINO(msg=1) == 1:
                print("Giro Base -> Ok")
            else:
                print("Giro Base -> Fallo")
        except Exception as e:
            print("Error: " + str(e))

    # --------------------------------
    # Función para enviar comandos a arduino

    def SEND_TO_ARDUINO(self, msg=0):
        try:
            if self.arduino.is_open:
                msg_control = 0

                # Enviamos mensaje

                self.arduino.write(str(msg).encode("utf-8"))
                print("Enviado -> " + str(str(msg).encode("utf-8")))
                # time.sleep(0.5)
                msg_control = self.arduino.readline().decode("ascii")

                print("Recibido --> " + str(msg_control))
                msg_control = int(msg_control)
                # print("Arduino : " + str(msg_control))
                if msg_control == 1:
                    print("Procesado correctamente.")
                    # Liberamos Buffer

                    self.arduino.flush()
                    return msg_control
            else:
                print(
                    "Error en comunicación con el Arduino, mensaje de confirmación no recibido."
                )
                # Liberamos Buffer

                self.arduino.flush()
                return msg_control
        except serial.serialutil.SerialException:
            print("Comunication Failed" + str(serial.serialutil.SerialException))
            # Liberamos Buffer

            # self.arduino.flush()
            return False


# Comunicación Arduino

""" -> Para Arduino Nano
#define DIR 5
#define STP 6
#define SLP 7
#define RST 8
#define M2 9
#define M1 10
#define M0 11
#define EN 12
"""


"""
Las opciones posibles para enviar serán:

- 1 -> Realizar paso
- 2 -> Establecer velocidad de giro
- 3 -> Indica paso 1    - Full Step (1.8°)
- 4 -> Indica paso 1/2  - Half Step
- 5 -> Indica paso 1/4
- 6 -> Indica paso 1/8
- 7 -> Indica paso 1/16
- 8 -> Indica paso 1/32
- 9 -> Deshabilitar motor
- 10 -> Apagar Laser
- 11 -> Encender Laser

Los mensajes a recibir serán: 

- 1 -> Paso ejecutado
- 2 -> Cambio realizado (ajustes de velocidad, tipo de paso, etc.)

"""

# Script Arduino:

"""

#define EN 2

#define M0 3
#define M1 4
#define M2 5

#define SLP 7
#define RST 6

#define DIR 9
#define STEP 8

#define LASERI_POS 10
#define LASERI_NEG 11

#define LED_POS 13
#define LED_NEG 12

String data_send;
int data_input = -1;
int vel = 200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(50);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(LASERI_POS, OUTPUT);
  pinMode(LASERI_NEG, OUTPUT);
  pinMode(LED_POS, OUTPUT);
  pinMode(LED_NEG, OUTPUT);

  digitalWrite(EN, LOW);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  digitalWrite(LASERI_POS, LOW);
  digitalWrite(LASERI_NEG, LOW);

  digitalWrite(LED_POS, LOW);
  digitalWrite(LED_NEG, LOW);

  digitalWrite(RST, HIGH);
  digitalWrite(SLP, HIGH);
  digitalWrite(DIR, HIGH);
  Serial.flush();
}

void loop() {

  /*"" Las opciones posibles a recibir serán :

    - 1->Realizar paso
    - 2->Establecer velocidad de giro
    - 3->Indica paso 1 - Full Step(1.8°)
    - 4->Indica paso 1 / 2 - Half Step
    - 5->Indica paso 1 / 4
    - 6->Indica paso 1 / 8
    - 7->Indica paso 1 / 16
    - 8->Indica paso 1 / 32
    - 9->Deshabilitar motor

        Los datos a enviar serán :

    -0->Error en ejecución.
    - 1->Ejecutado correctamente.
    - 2->Cambio realizado(ajustes de velocidad, tipo de paso, etc.) ""*/

  if (Serial.available() > 0) {
    /* "Leemos el dato enviado desde PC"*/

    data_input = Serial.read();


    switch (data_input) {
      case 1:
        digitalWrite(EN, LOW);
        digitalWrite(STEP, HIGH);
        delay(vel);
        digitalWrite(STEP, LOW);
        delay(vel);
        Serial.println(1);
        break;
      case 3:
        digitalWrite(M0, LOW);
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        Serial.println(1);
        break;
      case 4:
        digitalWrite(M0, HIGH);
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        Serial.println(1);
        break;
      case 5:
        digitalWrite(M0, LOW);
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        Serial.println(1);
        break;
      case 6:
        digitalWrite(M0, HIGH);
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        Serial.println(1);
        break;
      case 7:
        digitalWrite(M0, LOW);
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        Serial.println(1);
        break;
      case 8:
        digitalWrite(M0, HIGH);
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        Serial.println(1);
        break;
      case 9:
        digitalWrite(EN, HIGH);
        Serial.println(1);
        break;
      case 10:
        digitalWrite(LASERI_POS, LOW);
        Serial.println(1);
        break;
      case 11:
        digitalWrite(LASERI_POS, HIGH);
        Serial.println(1);
        break;
      case 12:
        digitalWrite(LED_POS, HIGH);
        Serial.println(1);
        break;
      case 13:
        digitalWrite(LED_POS, LOW);
        Serial.println(1);
        break;
      default:
        Serial.println(0);
        break;
    }
  }
  data_input = -1;
}
"""
