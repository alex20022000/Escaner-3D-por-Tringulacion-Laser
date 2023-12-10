#define LASERI_POS 2
#define LED_POS 3
#define DIR 5
#define STEP 6
#define SLP 7
#define RST 8
#define M2 9
#define M1 10
#define M0 11
#define EN 12

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
  pinMode(LED_POS, OUTPUT);

  digitalWrite(EN, LOW);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  digitalWrite(LASERI_POS, LOW);
  digitalWrite(LED_POS, LOW);

  digitalWrite(RST, HIGH);
  digitalWrite(SLP, HIGH);
  digitalWrite(DIR, HIGH);
  Serial.flush();
}

void loop() {
 
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