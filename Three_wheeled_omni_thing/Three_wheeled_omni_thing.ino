#include "SPI.h" 
#include "RF24.h" 
#include "nRF24L01.h"
#include <PPMReader.h>

byte interruptPin = 3;
byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);

// NRF THINGS START
RF24 radio(49, 48); 
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
// NRF THINGS END

int ppm_array[] = {1500, 1500, 1500, 1500, 1500, 1500};

float acceleration = 10;
float input_velocity[] = {0, 0, 0};
float target_input_velocity[] = {0, 0, 0};

float parallel_input_velocity[] = {0, 0};
float angular_input_velocity = 0;

float wheel_velocity[] = {0, 0, 0};

// consts for fast wheel speed calculation START
float sqrt3 = sqrt(3);
float one_over_sqrt3 = 1/sqrt3;
float one_over_3 = 1/3;
float two_over_3 = 2/3;
float sqrt3_over_two = 0.5 * sqrt3;
// consts for fast wheel speed calculation END

const int wheel[][2] = {{5, 23}, {6, 24}, {4, 22}};
const int button = 25;
const int kill_indicator = 32;
const int rc_indicator = 33;
const int makeshift_indicator = 34;
const int random_indicator = 35;

float last_payload = 0;
float payload_kill = 1;
bool unprocessed_payload = false;
bool killed = false;

float last_out_of_bounds = 0;

int nrf_or_rc = 1;

void kill() {
  digitalWrite(wheel[0][0], 0);
  digitalWrite(wheel[1][0], 0);
  digitalWrite(wheel[2][0], 0);
  digitalWrite(wheel[0][1], 0);
  digitalWrite(wheel[1][1], 0);
  digitalWrite(wheel[2][1], 0);
  payload.stick_1[0] = 0;
  payload.stick_1[1] = 0;
  payload.stick_2[0] = 0;
  payload.stick_2[1] = 0;
  payload.stick_3[0] = 0;
  payload.stick_3[1] = 0;
  payload.button_1 = false;
  payload.button_2 = false;
  payload.button_3 = false;
  payload.button_S1 = false;
  payload.button_S2 = false;
  payload.button_S3 = false;
  target_input_velocity[0] = 0;
  target_input_velocity[1] = 0;
  target_input_velocity[2] = 0;
}

float processPPM(float delta) {
  // Print latest valid values from all channels
  // value from 1000 to 2000 (1500 at rest)
  // value is 0 if no remote connected
  bool out_of_bounds = false;
  for (byte channel = 1; channel <= channelAmount; ++channel) {
      unsigned value = ppm.latestValidChannelValue(channel, 0);
      if (value < 1000 || value > 2000) {
        out_of_bounds = true;
      }
      ppm_array[channel-1] = value;
  }
  if (out_of_bounds) {
    last_out_of_bounds += delta;
  } else {
    last_out_of_bounds = 0;
  }
  return last_out_of_bounds < 1;
}

float processPayload(float delta) {
  float dead = false;
  if (unprocessed_payload) {
    last_payload = 0;
    unprocessed_payload = false;
  } else {
    last_payload += delta;
  }
  if (last_payload > payload_kill) {
    dead = true;
  }
  return !dead;
}

void calculateInputVelocities(float delta) {

  // calculating raw stick inputs
  if (nrf_or_rc == 0) {
    // from makeshift remote
    target_input_velocity[0] = float(payload.stick_1[1])/1024;
    target_input_velocity[1] = float(payload.stick_2[0])/1024;
    target_input_velocity[2] = float(payload.stick_3[0])/1024;
  }
  else if (nrf_or_rc == 1) {
    // from rx reciever
    target_input_velocity[0] = float(ppm_array[2]-1500)/500;
    target_input_velocity[1] = float(ppm_array[3]-1500)/500;
    target_input_velocity[2] = float(ppm_array[0]-1500)/500;
  }

  Serial.print(target_input_velocity[0]);
  Serial.print(" ");
  Serial.print(target_input_velocity[1]);
  Serial.print(" ");
  Serial.print(target_input_velocity[2]);
  Serial.println();

  for (int i = 0; i < 3; i++) {
    float dv = target_input_velocity[i] - input_velocity[i];
  
    if (dv > 0) {
      input_velocity[i] += acceleration * delta;
      if (input_velocity[i] > target_input_velocity[i]) {
        input_velocity[i] = target_input_velocity[i];
      }
    } else if (dv < 0) {
      input_velocity[i] -= acceleration * delta;
      if (input_velocity[i] < target_input_velocity[i]) {
        input_velocity[i] = target_input_velocity[i];
      }
    }
  }

  // normalizing stick inputs

  parallel_input_velocity[0] = input_velocity[0];
  parallel_input_velocity[1] = input_velocity[1];
  float length = sqrt(parallel_input_velocity[0]*parallel_input_velocity[0] + parallel_input_velocity[1]*parallel_input_velocity[1]);
  
  if (length > 1) {
    parallel_input_velocity[0] /= length;
    parallel_input_velocity[1] /= length;
    length = 1;
  }

  angular_input_velocity = input_velocity[2];

  float mult = length + abs(angular_input_velocity);

  if (mult > 1) {
    angular_input_velocity /= mult;
    parallel_input_velocity[0] /= mult;
    parallel_input_velocity[1] /= mult;
  }
}

void calculateWheelVelocities(float delta) {
  float v = parallel_input_velocity[0];
  float vn = -parallel_input_velocity[1];
  float w = -angular_input_velocity;

  float v_0 = 0;
  float v_1 = 0;
  float v_2 = 0;

  //v = one_over_sqrt3 * (v_2 - v_0);
  //vn = one_over_3 * (v_2 + v_0) - two_over_3 * v_1;
  //w = one_over_3 * (v_0 + v_1 + v_2);

  v_0 = w + 0.5 * vn - sqrt3_over_two * v;
  v_1 = w - vn;
  v_2 = w + 0.5 * vn + sqrt3_over_two * v;

  wheel_velocity[0] = v_0;
  wheel_velocity[1] = v_1;
  wheel_velocity[2] = v_2;
}

void writeWheelVelocities() {
  for (int i = 0; i < 3; i++) {
    float min = 0.05;

    if (wheel_velocity[i] > min) {
      digitalWrite(wheel[i][1], 0);
    } else if (wheel_velocity[i] < -min) {
      digitalWrite(wheel[i][1], 1);
    } else {
      digitalWrite(wheel[i][1], 0);
    }

    if (wheel_velocity[i] > min) {
      analogWrite(wheel[i][0], 255 * wheel_velocity[i]);
    } else if (wheel_velocity[i] < -min) {
      analogWrite(wheel[i][0], 255 + 255 * wheel_velocity[i]);
    } else {
      digitalWrite(wheel[i][0], 0);
    }

  }
}

float blink_timer = 0;
bool button_pressed = false;
float button_time = 0;

void updateRemoteIndicator() {
  digitalWrite(rc_indicator, nrf_or_rc);
  digitalWrite(makeshift_indicator, 1 - nrf_or_rc);
}

void update(float delta) {
  blink_timer += delta;
  button_time += delta;

  if (killed) {
    if (blink_timer < 0.25){
      digitalWrite(kill_indicator, true);
    } else if (blink_timer < 0.5){
      digitalWrite(kill_indicator, false);
    } else {
      blink_timer = 0;
    }
  } else {
    digitalWrite(kill_indicator, false);
  }

  if (!digitalRead(button) && !button_pressed && button_time > 0.5){
    button_pressed = true;
    nrf_or_rc = 1 - nrf_or_rc;
    killed = false;
    button_time = 0;
    updateRemoteIndicator();
  } else {
    button_pressed = false;
  }
  
  
  if (nrf_or_rc == 0) {
    killed = !processPayload(delta);
    digitalWrite(random_indicator, payload.button_S1); 
  }
  else if (nrf_or_rc == 1) {
    killed = !processPPM(delta);
    digitalWrite(random_indicator, ppm_array[5] < 1500); 
  }

  if (killed) {
    kill();
  }

  if (!killed) {
    calculateInputVelocities(delta);
    calculateWheelVelocities(delta);
    writeWheelVelocities();
  }
}

void setupRadio() {
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setPayloadSize(sizeof(payload)); 
	radio.openReadingPipe(0, address); 
	radio.startListening(); 
}

void printRadio() {
  Serial.print("recieved");
  Serial.print(" x_1: ");
  Serial.print(payload.stick_1[0]);
  Serial.print(" y_1: ");
  Serial.print(payload.stick_1[1]);
  Serial.print(" x_2: ");
  Serial.print(payload.stick_2[0]);
  Serial.print(" y_2: ");
  Serial.print(payload.stick_2[1]);
  Serial.print(" x_3: ");
  Serial.print(payload.stick_3[0]);
  Serial.print(" y_3: ");
  Serial.print(payload.stick_3[1]);
  Serial.print(" S1: ");
  Serial.print(payload.button_S1);
  Serial.print(" S2: ");
  Serial.print(payload.button_S2);
  Serial.print(" S3: ");
  Serial.print(payload.button_S3);
  Serial.print(" B1: ");
  Serial.print(payload.button_1);
  Serial.print(" B2: ");
  Serial.print(payload.button_2);
  Serial.print(" B3: ");
  Serial.print(payload.button_3);
  Serial.println();
}

void setup (void) {
  Serial.begin(115200);

  setupRadio();

  pinMode(wheel[0][0], OUTPUT);
  pinMode(wheel[0][1], OUTPUT);
  pinMode(wheel[1][0], OUTPUT);
  pinMode(wheel[1][1], OUTPUT);
  pinMode(wheel[2][0], OUTPUT);
  pinMode(wheel[2][1], OUTPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(kill_indicator, OUTPUT);
  pinMode(rc_indicator, OUTPUT);
  pinMode(makeshift_indicator, OUTPUT);
  pinMode(random_indicator, OUTPUT);

  updateRemoteIndicator();
}

unsigned long delta_time = 0;
unsigned long previous_time = 0;
void loop (void) {
  delta_time = micros() - previous_time;
  previous_time = micros();

  if (radio.available() > 0) { 
    radio.read(&payload, sizeof(payload));
    unprocessed_payload = true;
    //print_radio();
  }

  update(float(delta_time) / 1000000.0);
}