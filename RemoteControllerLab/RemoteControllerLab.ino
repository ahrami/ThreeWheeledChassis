// stick 1 is connected to A0, A1
// calibrate button is connected to 2 pullup

#include "SPI.h" 
#include "RF24.h" 
#include "nRF24L01.h" 

RF24 radio(10, 9); 

const byte address[6] = "00069"; 

//NRF24L01 buffer limit is 32 bytes (max struct size)

struct Payload { 
	long stick_1[2] = {0, 0};
  long stick_2[2] = {0, 0};
  long stick_3[2] = {0, 0};
  bool button_S1 = false;
  bool button_S2 = false;
  bool button_S3 = false;
  bool button_1 = false;
  bool button_2 = false;
  bool button_3 = false;
};

Payload payload; 

long stick_1[] = {0, 0};
long stick_2[] = {0, 0};
long stick_3[] = {0, 0};

int zeros_1[] = {512, 512};
int zeros_2[] = {512, 512};
int zeros_3[] = {512, 512};

bool button_S1 = false;
bool button_S2 = false;
bool button_S3 = false;
bool button_1 = false;
bool button_2 = false;
bool button_3 = false;

void calibrate() {
  zeros_1[0] = analogRead(A0);
  zeros_1[1] = analogRead(A1);
  zeros_2[0] = analogRead(A2);
  zeros_2[1] = analogRead(A3);
  zeros_3[0] = analogRead(A4);
  zeros_3[1] = analogRead(A5);
}

void setup() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  Serial.begin(115200);
  radio.begin(); 
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(sizeof(payload));
  radio.openWritingPipe(address); 
	radio.stopListening();
}

void deadzone(long* stick, int* zeros, int zone) {
  for(int i = 0; i < 2; i++) {
    stick[i] = stick[i] - zeros[i];
    if (stick[i] > 0) {
      stick[i] = stick[i] >= zone ? stick[i] - zone : 0;
    } else if (stick[i] < 0) {
      stick[i] = stick[i] <= -zone ? stick[i] + zone : 0;
    }
    if (stick[i] > 0) {
      stick[i] *= 1024;
      stick[i] /= 1023 - zeros[i] - zone;
    } else if (stick[i] < 0) {
      stick[i] *= 1024;
      stick[i] /= zeros[i] - zone;
    }
  }
}

unsigned long button_wait_start = 0;
unsigned long button_wait_time = 100;

unsigned long transmission_interval = 50;
unsigned long last_transmission = 0;

void transmit() {
  payload.stick_1[0] = -stick_1[0];
  payload.stick_1[1] = -stick_1[1];
  payload.stick_2[0] = -stick_2[0];
  payload.stick_2[1] = -stick_2[1];
  payload.stick_3[0] = -stick_3[0];
  payload.stick_3[1] = -stick_3[1];
  payload.button_S1 = button_S1;
  payload.button_S2 = button_S2;
  payload.button_S3 = button_S3;
  payload.button_1 = button_1;
  payload.button_2 = button_2;
  payload.button_3 = button_3;
  radio.write(&payload, sizeof(payload));
}

void loop() {

  if (millis() - button_wait_start > button_wait_time && !digitalRead(2)) {
    button_wait_start = millis();
    calibrate();
  }

  stick_1[0] = analogRead(A0);
  stick_1[1] = analogRead(A1);
  stick_2[0] = analogRead(A2);
  stick_2[1] = analogRead(A3);
  stick_3[0] = analogRead(A4);
  stick_3[1] = analogRead(A5);
  button_S1 = !digitalRead(3);
  button_S2 = !digitalRead(4);
  button_S3 = !digitalRead(5);
  button_1 = !digitalRead(6);
  button_2 = !digitalRead(7);
  button_3 = !digitalRead(8);
  deadzone(stick_1, zeros_1, 50);
  deadzone(stick_2, zeros_2, 50);
  deadzone(stick_3, zeros_3, 50);
  //*
  Serial.print(" x_1: ");
  Serial.print(stick_1[0]);
  Serial.print(" y_1: ");
  Serial.print(stick_1[1]);
  Serial.print(" x_2: ");
  Serial.print(stick_2[0]);
  Serial.print(" y_2: ");
  Serial.print(stick_2[1]);
  Serial.print(" x_3: ");
  Serial.print(stick_3[0]);
  Serial.print(" y_3: ");
  Serial.print(stick_3[1]);
  Serial.print(" S1: ");
  Serial.print(button_S1);
  Serial.print(" S2: ");
  Serial.print(button_S2);
  Serial.print(" S3: ");
  Serial.print(button_S3);
  Serial.print(" B1: ");
  Serial.print(button_1);
  Serial.print(" B2: ");
  Serial.print(button_2);
  Serial.print(" B3: ");
  Serial.print(button_3);
  Serial.println();
  //*/
  if (millis() - last_transmission > transmission_interval) {
    last_transmission = millis();
    transmit();
    Serial.println("transmitted");
  }
}
