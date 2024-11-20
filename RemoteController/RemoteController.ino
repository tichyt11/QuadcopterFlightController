#include <SPI.h>
#include "printf.h"
#include <RF24.h>
#include <nRF24L01.h>

#define PRINT_STATE 1
#define PRINT_COMMANDS 0

RF24 radio(9,10); // CE, CSN

#define POT A0

#define JOYLB_PIN A1
#define JOYLX_PIN A2
#define JOYLY_PIN A3

#define JOYRX_PIN A6
#define JOYRY_PIN A7

#define BUTTON_R 2
#define BUTTON_L 3
#define SWITCH 4

#define LED1 5
#define LED2 6
#define LED3 7
#define WSLED 8

#include <FastLED.h>
CRGB leds[1];

const byte CMDADD[4] = "DRR";  // receive address
const byte STATADD[4] = "DRT";  // transmit address

int16_t Xstart;
int16_t Ystart;
int16_t X2start;
int16_t Y2start;

void setup() {
  pinMode(POT, INPUT);
  pinMode(JOYLB_PIN, INPUT);
  pinMode(JOYLY_PIN, INPUT);
  pinMode(JOYLX_PIN, INPUT);
  pinMode(JOYRY_PIN, INPUT);
  pinMode(JOYRX_PIN, INPUT);
  pinMode(BUTTON_L, INPUT);
  pinMode(BUTTON_R, INPUT);
  pinMode(SWITCH, INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(WSLED, OUTPUT);
  FastLED.addLeds<WS2812B, WSLED, GRB>(leds, 1);  // GRB ordering is typical

  Xstart = analogRead(JOYLX_PIN);
  Ystart = analogRead(JOYLY_PIN);
  X2start = analogRead(JOYRX_PIN);
  Y2start = analogRead(JOYRY_PIN);
  
  Serial.begin(115200);
  while (!Serial){}

  SPI.setClockDivider(SPI_CLOCK_DIV2);

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} 
  }
  
  radio.setPALevel(RF24_PA_MAX, 1);
  radio.setChannel(3);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_2MBPS);
  radio.enableDynamicAck();
  radio.setAutoAck(0);
  radio.openWritingPipe(CMDADD); 
  radio.openReadingPipe(1, STATADD); 
  radio.startListening();   
  
  printf_begin();
  radio.printPrettyDetails();
}

float Xval;
float Yval;
float X2val;
float Y2val;
float Tval;
uint8_t button1_state = 1;
uint8_t button2_state = 1;

uint8_t motors_mode = 1;
uint8_t motors_on = 0;
bool led_on = 0;

long last_cmd_t = 0;
long last_shown = 0;


void loop() {

  Xval = (analogRead(JOYLX_PIN) - Xstart)/20.0;  // pm 12.5 degs
  Yval = (analogRead(JOYLY_PIN) - Ystart)/20.0;
  X2val = -(analogRead(JOYRX_PIN) - X2start)/20.0;  // pm 12.5 degs
  Y2val = -(analogRead(JOYRY_PIN) - Y2start)/20.0;
  Tval = (analogRead(POT))/1024.0;

  if (digitalRead(BUTTON_L) && !button1_state){ 
    button1_state = 1;
  }
  if (!digitalRead(BUTTON_L) && button1_state){
    button1_state = 0;
    motors_mode = 1 + ((motors_mode) % 5);
    Serial.println("Left button pressed");
  }
  
  if (digitalRead(BUTTON_R) && !button2_state){ 
    button2_state = 1;
  }
  if (!digitalRead(BUTTON_R) && button2_state){
    button2_state = 0;
    motors_on = !motors_on;
    Serial.println("Right button pressed");
  }

  analogWrite(LED1, int(256*Xval/30));
  analogWrite(LED2, int(256*Yval/30));

  leds[0] = CHSV(0, 255, int(64*!(motors_on)));
  FastLED.show();

  if ( abs(Xval) < 0.5) Xval = 0; // less than 1 deg
  if ( abs(Yval) < 0.5) Yval = 0; // less than 1 deg
  if ( abs(X2val) < 0.5) X2val = 0; // less than 1 deg
  if ( abs(Y2val) < 0.5) Y2val = 0; // less than 1 deg
  
  ctrl_msg_t ctrl_msg = {X2val, Y2val, 0.0, Tval, motors_on*motors_mode};
  msg_t msg; 
  msg.type = 0;
  msg.data.ctrl_data = ctrl_msg;
  bool report = 0;

//  Serial.println(millis() - last_cmd_t);
  if ((millis() - last_cmd_t) >= 100){
    last_cmd_t = millis();
    radio.stopListening(); 
    radio.flush_tx();
    long start_time = micros();
//    report = radio.write(&msg, sizeof(msg_t), 0);       // 800 usec
//    bool report = radio.writeFast(&payload, sizeof(payload));       // 700 usec
    report = radio.startWrite(&msg, sizeof(msg_t), 0); 
    long end_time = micros();
    delayMicroseconds(250);
    radio.startListening(); 
    if (PRINT_COMMANDS){
    Serial.print(Xval);
    Serial.print(" ");
    Serial.print(Yval);
    Serial.print(" ");
    Serial.print(X2val);
    Serial.print(" ");
    Serial.print(Y2val);
    Serial.print(" ");
//    Serial.print(Tval);
//    Serial.print(" ");
//    Serial.print(motors_mode);
    Serial.println(" ");
    Serial.println(motors_on);
    }
//    
//    Serial.println("Transmit took " + String(end_time - start_time) + " microseconds");
  }

  if (Serial.available() > 0){  // received serial commands
    String msg = Serial.readStringUntil(' ');
    if (msg == "roll" || msg == "pitch" || msg == "yaw"){  // react only if pitch or roll or yaw is seen
      float P = Serial.parseFloat();
      float I = Serial.parseFloat();
      float D = Serial.parseFloat();
      float sat = Serial.parseFloat();
      float LPc = Serial.parseFloat();
      config_msg_t new_config;
      if (msg == "roll"){
        new_config = {P, I, D, sat, LPc, 0};
        String text = "Sending roll config: P " + String(P, 2) + ", I " + String(I, 2) + ", D " + String(D, 2) + ", sat " + String(sat, 2) + ", LPc " + String(LPc, 2);
        Serial.println(text);
      }
      else if (msg == "pitch"){
        new_config = {P, I, D, sat, LPc, 1};
        String text = "Sending pitch config: P " + String(P, 2) + ", I " + String(I, 2) + ", D " + String(D, 2) + ", sat " + String(sat, 2) + ", LPc " + String(LPc, 2);
        Serial.println(text);
      }
      else if (msg == "yaw"){
        new_config = {P, I, D, sat, LPc, 2};
        String text = "Sending yaw config: P " + String(P, 2) + ", I " + String(I, 2) + ", D " + String(D, 2) + ", sat " + String(sat, 2) + ", LPc " + String(LPc, 2);
        Serial.println(text);
      }
      // sent config message
      msg_t msg; 
      msg.type = 1;
      msg.data.config_data = new_config;
      bool report = 0;
      radio.stopListening(); 
      report = radio.write(&msg, sizeof(msg_t));
      if (report){
        Serial.print("Config sent");
      }
      radio.startListening(); 
    }
    
  }

  uint8_t pipe;
  telemetry_msg_t tele_msg;
  if(radio.available(&pipe)){
    radio.read(&tele_msg, sizeof(tele_msg)); 
    if (tele_msg.type == 0){  // state data
      if ((millis() - last_shown) > 33){
        if (PRINT_STATE){
          state_struct s_data = tele_msg.data.state_data;
          Serial.print("State: ");
  //        Serial.print(s_data.ms);  
  //        Serial.print(' ');   
          Serial.print(s_data.roll);  
          Serial.print(' ');   
          Serial.print(s_data.pitch);  
          Serial.print(' ');   
          Serial.print(s_data.yaw); 
          for (int i = 0; i < 4; i++){
            Serial.print(" " + String(s_data.motors[i]/65535.0));
          }
          Serial.println();
        }
        last_shown = millis();
      }
    }
    if (tele_msg.type == 1){  // sensor data
      sensor_struct s_data = tele_msg.data.sensor_data;
      Serial.print("Telemetry: ");
      Serial.print(s_data.battery);  
      Serial.print(' ');   
      Serial.println(s_data.height); 
    }
  }

//  delay(100);
}