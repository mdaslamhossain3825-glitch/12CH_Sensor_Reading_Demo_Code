// Define control pins for the multiplexer
#define selectsenA 33
#define selectsenB 32
//
#define inputsenA 34
#define inputsenB 35
#define inputsenC 25
#define inputsenD 26

#define sensorNumber 12
int sensorADC[sensorNumber];

void setup() {
  Serial.begin(115200);
  pinMode(selectsenA, OUTPUT);
  pinMode(selectsenB, OUTPUT);
}

void Read_ADC() {
  //00
  digitalWrite(selectsenA, LOW);
  digitalWrite(selectsenB, LOW);
  sensorADC[6] = analogRead(inputsenA);
  sensorADC[4] = analogRead(inputsenC);
  sensorADC[1] = analogRead(inputsenD);
  //01
  digitalWrite(selectsenA, LOW);
  digitalWrite(selectsenB, HIGH);
  sensorADC[7] = analogRead(inputsenA);
  sensorADC[11] = analogRead(inputsenB);
  sensorADC[2] = analogRead(inputsenC);
  sensorADC[0] = analogRead(inputsenD);
  //10
  digitalWrite(selectsenA, HIGH);
  digitalWrite(selectsenB, LOW);
  sensorADC[9] = analogRead(inputsenA);
  sensorADC[10] = analogRead(inputsenB);
  sensorADC[3] = analogRead(inputsenC);
  //11
  digitalWrite(selectsenA, HIGH);
  digitalWrite(selectsenB, HIGH);
  sensorADC[8] = analogRead(inputsenA);
  sensorADC[5] = analogRead(inputsenC);
}

void analogShow() {
  while (1) {
    Read_ADC();
    for (int i = 0; i < 12; i++) {  //display bitsensor data
      Serial.print(String(sensorADC[i]) + "  ");
    }
    Serial.println();
  }
}

void loop() {
  analogShow();
}
