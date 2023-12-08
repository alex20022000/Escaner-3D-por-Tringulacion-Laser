 

void setup() { 
  Serial.begin(9600);
  Serial.setTimeout(50);
}

void loop() {

  if (Serial.available() > 0) { 
    int data_input = Serial.parseInt(); 


    switch (data_input) {
      case 1:
        Serial.println(1);
        break;
      case 3:
        Serial.println(1);
        break;
      case 4:
        Serial.println(1);
        break;
      case 5:
        Serial.println(1);
        break;
      case 6:
        Serial.println(1);
        break;
      case 7:
        Serial.println(1);
        break;
      case 8:
        Serial.println(1);
        break;
      case 9:
        Serial.println(1);
        break;
      case 10:
        Serial.println(1);
        break;
      case 11:
        Serial.println(1);
        break;
      case 12:
        Serial.println(1);
        break;
      case 13:
        Serial.println(1);
        break;
      default:
        Serial.println(0);
        break;
    }
  }
}
