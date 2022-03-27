/* -------------------------------------------------------------------------- */
/*                                  Libraries                                 */
/* -------------------------------------------------------------------------- */
#include <Arduino.h>
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include <Wire.h>
#include <SPI.h>
#include "DHT.h"
#include <ds3231.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SD.h>

/* -------------------------------------------------------------------------- */
/*                                 Components                                 */
/* -------------------------------------------------------------------------- */

/* ---------------------------- TFT LCD (Screen) ---------------------------- */
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

/* ------------------------ DS3231 (Real Time Clock) ------------------------ */
struct ts t;
int hour=12; 
int min=30;
int sec=0;
int mday=25;
int mon=12;
int year=2019;

/* ------------------------- RELAY (AC Power Supply) ------------------------ */
#define RELAY 25
volatile byte power = true;

/* ----------------------- RCWL-0516 (Movement Sensor) ---------------------- */
#define RCWL 24
volatile byte motion = false;
int motionCounter = 0;

/* ------------------------ PAM8406 (Audio Amplifier) ----------------------- */
#define PAM8406 26

/* ----------------- DTH22 (Humidity and Temperature Sensor) ---------------- */
#define DHTPIN 27
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

/* --------------------- BARS GND (Floating Ground Bars) -------------------- */
#define firstBAR 34
#define lastBAR 47

/* --------------------- RESISTOR (Capacitor Discharge) --------------------- */
#define RESISTOR 18
int dischargeTime = 30;

/* ----------------- microSD (Micro SD Card Breakout Board) ----------------- */
# define PIN_SD_CS 53

/* -------------------------------------------------------------------------- */
/*                           Expertiment Parameters                           */
/* -------------------------------------------------------------------------- */
String readString;
String toneFrequencyStr;
String toneTimeStr;
String stimulationTimeStr;
String movementAnalysisTimeStr;
String intervalTimeStr;
String numberOfEventsStr;
String experimentAnimalStr;
String dayOfExperimentStr;
int toneFrequency;
int toneTime;
int stimulationTime;
int movementAnalysisTime;
int intervalTime;
int numberOfEvents;
int experimentAnimal; // Missing in HTML
int dayOfExperiment;  // Missing in HTML
float temperature;
float humidity;
volatile byte confirmed = false;

/* -------------------------------------------------------------------------- */
/*                          Temperature and Humidity                          */
/* -------------------------------------------------------------------------- */
void ReadTH(){
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}

/* -------------------------------------------------------------------------- */
/*                                Date and Time                               */
/* -------------------------------------------------------------------------- */
void GetClock(){
  DS3231_get(&t);
}

/* -------------------------------------------------------------------------- */
/*                               AC Power Supply                              */
/* -------------------------------------------------------------------------- */
void RelayActivation(){
    power = !power;
    if(power){ 
        digitalWrite(RELAY, HIGH);
    }
    else {
        digitalWrite(RELAY, LOW);
    }
}

/* -------------------------------------------------------------------------- */
/*                             Capacitor Discharge                            */
/* -------------------------------------------------------------------------- */
void Discharge(){
  tft.setCursor(0,0);
  tft.fillScreen(BLACK);
  tft.println("Discharging the capacitor");
}

/* -------------------------------------------------------------------------- */
/*                          Save Data to microSD card                         */
/* -------------------------------------------------------------------------- */
void SaveData(){
  tft.setCursor(0,0);
  tft.fillScreen(BLACK);
  tft.println("Saving Data into the microSD card");
}

/* -------------------------------------------------------------------------- */
/*                                Tone Stimulus                               */
/* -------------------------------------------------------------------------- */
void ToneActivation(){
  unsigned int toneFreq = toneFrequency;
  unsigned long timeLimit = toneTime * 1000;
  unsigned long startMillis = millis();
  tft.fillScreen(BLUE);
  while(millis() - startMillis < (timeLimit)){
    tone(PAM8406, toneFreq);
  }
  noTone(PAM8406);
}

/* -------------------------------------------------------------------------- */
/*                       Floating Ground Bars Activation                      */
/* -------------------------------------------------------------------------- */
void Stimulus(){
  unsigned long timeLimit = stimulationTime * 1000;
  unsigned long startMillis = millis();
  tft.fillScreen(RED);
  while(millis() - startMillis < (timeLimit)){
      for(int i=firstBAR; i<=lastBAR; i++)
        {
          digitalWrite(i, HIGH);
        }
  }
  for(int i=firstBAR; i<=lastBAR; i++)
  {
    digitalWrite(i, LOW);
  }
}

/* -------------------------------------------------------------------------- */
/*                              Motion Detection                              */
/* -------------------------------------------------------------------------- */
void MotionDetection(){
  unsigned long timeLimit = movementAnalysisTime * 1000;
  unsigned long startMillis = millis();
  tft.fillScreen(GREEN);
  while(millis() - startMillis < (timeLimit)){
    int val = digitalRead(RCWL);
    if (val == HIGH) {
      if (motion == false) {
        motion = true;
        motionCounter++;
      }
    }
    else {
      if (motion == true) {
        motion = false;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                         Waiting Time Between Events                        */
/* -------------------------------------------------------------------------- */
void Wait(){
  unsigned long timeLimit = intervalTime * 1000;
  unsigned long startMillis = millis();
  tft.fillScreen(CYAN);
  while(millis() - startMillis < (timeLimit)){
    motion = false;
  }
}
/* -------------------------------------------------------------------------- */
/*                    String to Integer variable conversion                   */
/* -------------------------------------------------------------------------- */
void VariableConversion(){
  toneFrequency = toneFrequencyStr.toInt();
  toneTime = toneTimeStr.toInt();
  stimulationTime = stimulationTimeStr.toInt();
  movementAnalysisTime = movementAnalysisTimeStr.toInt();
  intervalTime = intervalTimeStr.toInt();
  numberOfEvents = numberOfEventsStr.toInt();
}

/* -------------------------------------------------------------------------- */
/*                            Full Experiment Event                           */
/* -------------------------------------------------------------------------- */
void Experiment(){
  /* ----------------------- AC Power Supply activation ----------------------- */
  RelayActivation();
  /* -------------------------- Data Type Conversion -------------------------- */
  VariableConversion();
  delay(2000);
  /* --------------- Repetition of events during the experiment --------------- */
  for(int i= 0; i<numberOfEvents; i++){
    ToneActivation();
    Stimulus();
    MotionDetection();
    Wait();
    tft.setCursor(0,0);
    tft.fillScreen(BLACK);
    tft.println("Finished Event " + String(i+1));
    delay(2000);
  }
  /* ---------------------- AC Power Supply deactivation ---------------------- */
  RelayActivation();
  delay(2000);
  /* --------------------------- Capacitor Discharge -------------------------- */
  Discharge();
  delay(2000);
  /* --------------------- Save data into the microSD card -------------------- */
  SaveData();
  delay(2000);
}

/* -------------------------------------------------------------------------- */
/*                                 ESP32 UART                                 */
/* -------------------------------------------------------------------------- */
void ESP32UART(){
  int ind1;
  int ind2;
  int ind3;
  int ind4;
  int ind5;

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
      /* -------------------------- Split received string ------------------------- */
      ind1 = readString.indexOf(','); 
      toneFrequencyStr = readString.substring(0, ind1); 
      ind2 = readString.indexOf(',', ind1+1 );
      toneTimeStr = readString.substring(ind1+1, ind2);
      ind3 = readString.indexOf(',', ind2+1 );
      stimulationTimeStr = readString.substring(ind2+1, ind3);
      ind4 = readString.indexOf(',', ind3+1 );
      movementAnalysisTimeStr = readString.substring(ind3+1, ind4);
      ind5 = readString.indexOf(',', ind4+1 );
      intervalTimeStr = readString.substring(ind4+1, ind5);
      numberOfEventsStr = readString.substring(ind5+1);
      /* ----------------------- Display received parameters ---------------------- */
      tft.println("Tone Freq.: " + toneFrequencyStr + " Hz");
      tft.println("Tone Duration: " + toneTimeStr + " s");
      tft.println("Stim. Duration: " + stimulationTimeStr + " s");
      tft.println("Mov. Analysis: " + movementAnalysisTimeStr + " s");
      tft.println("Int. Duration: " + intervalTimeStr + " s");
      tft.println("Num. of Events: " + numberOfEventsStr);
    }
  confirmed = true; // Confirms the received parameters to initiate experiment
  }
}

/* -------------------------------------------------------------------------- */
/*                                Arduino Setup                               */
/* -------------------------------------------------------------------------- */
void setup() {

  /* -------------------------- Serial Communication -------------------------- */
  Serial.begin(115200);
  Serial1.begin(115200);
  /* ---------------------------- IC2 Communication --------------------------- */
  Wire.begin();
  /* -------------------------------- LCD Setup ------------------------------- */
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
  /* -------------------------------- Pin Setup ------------------------------- */
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  pinMode(RCWL, INPUT);
  pinMode(PAM8406, OUTPUT);
  pinMode(DHTPIN, INPUT);
  for(int i=firstBAR; i<=lastBAR; i++)
  {
    pinMode(i, OUTPUT);
  }
  /* ------------------------------ DS3231 Setup ------------------------------ */
  DS3231_init(DS3231_CONTROL_INTCN);
  t.hour=hour; 
  t.min=min;
  t.sec=sec;
  t.mday=mday;
  t.mon=mon;
  t.year=year;
  DS3231_set(t);
  /* ------------------------------ microSD Setup ----------------------------- */
  /*if (!SD.begin(PIN_SD_CS)) { // Inicializa la comunicación con la tarjeta SD con el pin 53 como selector del SPI
    Serial.println("SD initialization failed, or not present!");
    while (1); // don't do anything more:
  }else
  Serial.println("SD initialization done."); */
  /* ------------------------------- DTH22 Setup ------------------------------ */
  dht.begin();
}

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */
void loop() {
  ESP32UART();
  if(confirmed == true){
    Experiment();
    confirmed = false;
  }
}