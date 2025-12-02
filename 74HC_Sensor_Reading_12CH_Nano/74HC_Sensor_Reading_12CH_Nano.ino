//Arduino NANO

#define selectsenA 2
#define selectsenB 3

#define inputsenA A0
#define inputsenB A1
#define inputsenC A2
#define inputsenD A3

//Sensor Variables
#define sensorNumber 12
int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];
int bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };
int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
int sumOnSensor;
int sensorWeight;
int bitSensor;
int threshold = 1000;

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

void setup() {
  pinMode(selectsenA, OUTPUT);
  pinMode(selectsenB, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  bitSensorShow();
}

void bitSensorShow() {
  while (1) {
    read();
    for (int i = 0; i < sensorNumber; i++) {  //display bitsensor data
      Serial.print(String(sensorADC[i]) + "  ");
    }
    //
    for (int i = 11; i >= 0; i--) {  //display bitsensor data
      Serial.print(String(bitRead(bitSensor, i)));
    }
    Serial.println();
  }
}

void read() {
  sumOnSensor = 0;
  sensorWeight = 0;
  bitSensor = 0;

  Read_ADC();

  for (int i = 0; i < sensorNumber; i++) {
    //analog to digital
    if (sensorADC[i] > threshold) {
      sensorDigital[i] = 1;
    } else {
      sensorDigital[i] = 0;
    }

    sumOnSensor += sensorDigital[i];
    sensorWeight += sensorDigital[i] * WeightValue[i];
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}