// ===== LIBRARIES =====
#include <Arduino.h>
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include <Wire.h>
#include <SPI.h>

// ===== COMPONENTS =====
// TFT LCD
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// Relay
# define RELAY 22
volatile byte lamp = LOW;
// RCWL-0516
# define RCWL 34
# define LED 27
volatile byte motion = false;
// LM386
# define LM386 26
// Push buttons
# define BTN_EXPERIMENT 22
# define BTN_RELAY 23
// BARS AC
# define BAR1 31

// ===== EXPERIMENT PARAMETERS =====
String readString;
String toneFrequency;
String toneTime;
String stimulationTime;
String movementAnalysisTime;
String intervalTime;
String numberOfEvents;
byte status_experiment = LOW;
byte status_relay = LOW;

int ind1;
int ind2;
int ind3;
int ind4;
int ind5;
int ind6;

// ===== ESP32 UART =====
void ESP32UART(){

  if(Serial1.available()){
    readString = Serial1.readString();
    if(readString){
      tft.setCursor(0,0);
      tft.fillScreen(BLACK);
      tft.setTextColor(GREEN);
      tft.println("Conditioning Chamber");
      tft.setTextColor(WHITE);
      tft.println("");
      tft.println("Received Parameters");
      tft.println("");

      ind1 = readString.indexOf(','); 
      toneFrequency = readString.substring(0, ind1); 
      ind2 = readString.indexOf(',', ind1+1 );
      toneTime = readString.substring(ind1+1, ind2);
      ind3 = readString.indexOf(',', ind2+1 );
      stimulationTime = readString.substring(ind2+1, ind3);
      ind4 = readString.indexOf(',', ind3+1 );
      movementAnalysisTime = readString.substring(ind3+1, ind4);
      ind5 = readString.indexOf(',', ind4+1 );
      intervalTime = readString.substring(ind4+1, ind5);
      ind6 = readString.indexOf(',', ind5+1 );
      numberOfEvents = readString.substring(ind5+1);
      tft.println("Tone Freq.: " + toneFrequency + " Hz");
      tft.println("Tone Duration: " + toneTime + " s");
      tft.println("Stim. Duration: " + stimulationTime + " s");
      tft.println("Mov. Analysis: " + movementAnalysisTime + " s");
      tft.println("Int. Duration: " + intervalTime + " s");
      tft.println("Num. of Events: " + numberOfEvents);
    }
  }
}

// ===== RELAY SWITCH =====
void RelayActivation(){
    lamp = !lamp;
    if(lamp){ 
        digitalWrite(RELAY, HIGH);
    }
    else {
        digitalWrite(RELAY, LOW);
    }
}

// ===== TONE GENERATION =====
void ToneActivation(){
    unsigned int toneFreq = toneFrequency.toInt();
    int timeLimit = toneTime.toInt() * 1000;
    int startMillis = millis();
    tft.fillScreen(BLUE);
    while(millis() - startMillis < (timeLimit)){
      tone(LM386, toneFreq);
    }
    noTone(LM386);
}

// ===== STIMULUS =====
void Stimulus(){
  int timeLimit = stimulationTime.toInt() * 1000;
  int startMillis = millis();
  tft.fillScreen(RED);
  while(millis() - startMillis < (timeLimit)){
    digitalWrite(BAR1, HIGH);
    digitalWrite(LED, HIGH);
  }
  digitalWrite(BAR1, LOW);
  digitalWrite(LED, LOW);
}

// ===== MOTION DETECTION =====
void MotionDetection(){
  int timeLimit = movementAnalysisTime.toInt() * 1000;
  int startMillis = millis();
  tft.fillScreen(GREEN);
  while(millis() - startMillis < (timeLimit)){
    int val = digitalRead(RCWL);
    if (val == HIGH) {
      if (motion == false) {
        motion = true;
        digitalWrite(LED, HIGH);
      }
    }
    else {
      if (motion == true) {
        motion = false;
        digitalWrite(LED, LOW);
      }
    }
  }
  digitalWrite(LED, LOW);
}

// ===== WAITING INTERVAL =====
void Wait(){
  int timeLimit = intervalTime.toInt() * 1000;
  int startMillis = millis();
  while(millis() - startMillis < (timeLimit)){
    tft.fillScreen(CYAN);
  }
}

// ===== EXPERIMENT =====
void Experiment(){
  ToneActivation();
  Stimulus();
  MotionDetection();
  Wait();
}

// ===== PUSHED BUTTON =====
void PushedButton(){
  status_experiment = digitalRead(BTN_EXPERIMENT);
  status_relay = digitalRead(BTN_RELAY);
  if(status_experiment == HIGH){
    Experiment();
    tft.setCursor(0,0);
    tft.fillScreen(BLACK);
    tft.setTextColor(YELLOW);
    tft.println("Finished Experiment");
    status_experiment = LOW;
  }
  if(status_relay == HIGH){
    RelayActivation();
    status_relay = LOW;
  }
  
}

// ===== SETUP  =====
void setup() {

  // Serial Communication
  Serial.begin(115200);
  Serial1.begin(115200);

  // LCD Setup
  tft.reset();
  tft.begin(0x9341) ;
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setTextColor(GREEN);
  tft.setCursor(0,0);
  tft.println("Conditioning Chamber");
  tft.println("");
  tft.setTextColor(WHITE);
  tft.println("SSID: WebChamber");
  tft.println("PWD: helloworld");
  tft.println("IP: 192.168.4.1");
  
  // Pin Setup
  pinMode(RCWL, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LM386, OUTPUT);
  pinMode(BTN_EXPERIMENT, INPUT);
  pinMode(BTN_RELAY, INPUT);
  pinMode(BAR1, OUTPUT);
}

// ===== MAIN =====
void loop() {
  ESP32UART();
  PushedButton();
}