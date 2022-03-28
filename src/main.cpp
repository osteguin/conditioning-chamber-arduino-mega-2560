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
int setHour=12; 
int setMin=30;
int setSec=0;
int setDay=25;
int setMonth=12;
int setYear=2019;

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
#define RESISTOR 30
unsigned int dischargeTime = 120;

/* ----------------- microSD (Micro SD Card Breakout Board) ----------------- */
#define PIN_SD_CS 53
File myFile;

/* ----------------------------------- LED ---------------------------------- */
#define LED 31

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
String experimentAnimalStr = "C-CM 1"; // Missing in HTML
String dayOfExperimentStr = "1"; // Missing in HTML
int toneFrequency;
unsigned int toneTime;
unsigned int stimulationTime;
unsigned int movementAnalysisTime;
unsigned int intervalTime;
int numberOfEvents;

float temperature;
float humidity;

unsigned long experimentStart;
unsigned long experimentEnd;
unsigned long experimentTotalTime;

int hour;
int minute;
int seconds;
int day;
int month;
int year;
int dayStart;
int monthStart;
int yearStart;
int hourStart;
int minuteStart;
int secondStart;
int dayEnd;
int monthEnd;
int yearEnd;
int hourEnd;
int minuteEnd;
int secondEnd;
volatile byte confirmed = false;

/* -------------------------------------------------------------------------- */
/*                          Temperature and Humidity                          */
/* -------------------------------------------------------------------------- */
void ReadDTH(){
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}

/* -------------------------------------------------------------------------- */
/*                                Date and Time                               */
/* -------------------------------------------------------------------------- */
void GetClock(){
  DS3231_get(&t);
  hour = t.hour;
  minute = t.min;
  seconds = t.sec;
  day = t.mday;
  month = t.mon;
  year = t.year;
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
  unsigned long timeLimit = dischargeTime * 1000;
  unsigned long startMillis = millis();
  tft.setCursor(0,0);
  tft.fillScreen(BLACK);
  tft.println("Discharging the capacitor");
  while(millis() - startMillis < (timeLimit)){
    digitalWrite(RESISTOR, HIGH);
  }
  digitalWrite(RESISTOR, LOW);
}

/* -------------------------------------------------------------------------- */
/*                          Save Data to microSD card                         */
/* -------------------------------------------------------------------------- */
void SaveData(){
  tft.setCursor(0,0);
  tft.fillScreen(BLACK);
  tft.println("Saving Data into the microSD card");
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    String dateStart = String(dayStart) + "/" +  String(monthStart) + "/" + String(yearStart);
    String timeStart = String(hourStart) + ":" + String(minuteStart) + ":" + String(secondStart);
    String dateEnd = String(dayEnd) + "/" + String(monthEnd) + "/" + String(yearEnd);
    String timeEnd = String(hourEnd) + ":" + String(minuteEnd) + ":" + String(secondEnd);
    String dataToSave = dayOfExperimentStr + "," + experimentAnimalStr + "," + dateStart + "," + timeStart + "," + dateEnd + "," + timeEnd + "," + experimentTotalTime + "," + temperature + "," + humidity + "," + toneFrequencyStr + "," + toneTimeStr + "," + stimulationTimeStr + "," + movementAnalysisTimeStr + "," + intervalTimeStr + "," + numberOfEvents;
    Serial.println(dataToSave);
  }
}

/* -------------------------------------------------------------------------- */
/*                                Tone Stimulus                               */
/* -------------------------------------------------------------------------- */
void ToneActivation(){
  unsigned int toneFreq = toneFrequency;
  unsigned long timeLimit = toneTime;
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
  unsigned long timeLimit = stimulationTime;
  unsigned long startMillis = millis();
  tft.fillScreen(RED);
  while(millis() - startMillis < (timeLimit)){
      for(int i=firstBAR; i<=lastBAR; i++)
        {
          digitalWrite(i, HIGH);
          delay(50);
          digitalWrite(i, LOW);
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
  unsigned long timeLimit = movementAnalysisTime;
  unsigned long startMillis = millis();
  tft.fillScreen(GREEN);
  while(millis() - startMillis < (timeLimit)){
    int val = digitalRead(RCWL);
    if (val == HIGH) {
      if (motion == false) {
        motion = true;
        digitalWrite(LED, HIGH);
        motionCounter++;
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

/* -------------------------------------------------------------------------- */
/*                         Waiting Time Between Events                        */
/* -------------------------------------------------------------------------- */
void Wait(){
  unsigned long timeLimit = intervalTime;
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
  toneTime = toneTimeStr.toInt() * 1000;
  stimulationTime = stimulationTimeStr.toInt() * 1000;
  movementAnalysisTime = movementAnalysisTimeStr.toInt() * 1000;
  intervalTime = intervalTimeStr.toInt() * 1000;
  numberOfEvents = numberOfEventsStr.toInt();
}

/* -------------------------------------------------------------------------- */
/*                                 Main Screen                                */
/* -------------------------------------------------------------------------- */
void MainScreen(){
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
}

/* -------------------------------------------------------------------------- */
/*                            Full Experiment Event                           */
/* -------------------------------------------------------------------------- */
void Experiment(){
  ReadDTH();
  experimentStart = millis();
  motionCounter = 0;
  /* ----------------------- AC Power Supply activation ----------------------- */
  RelayActivation();
  /* -------------------------- Data Type Conversion -------------------------- */
  VariableConversion();
  delay(2000);
  /* ----------------------- Experiment Start Clock Data ---------------------- */
  GetClock();
  dayStart = day;
  monthStart = month;
  yearStart = year;
  hourStart = hour;
  minuteStart = minute;
  secondStart = seconds;
  /* --------------- Repetition of events during the experiment --------------- */
  for(int i= 1; i<=numberOfEvents; i++){
    ToneActivation();
    Stimulus();
    MotionDetection();
    Wait();
    tft.setCursor(0,0);
    tft.fillScreen(BLACK);
    tft.println("Finished Event " + String(i));
    delay(2000);
  }
  experimentEnd = millis();
  experimentTotalTime = (experimentEnd - experimentStart) / 1000;
  /* ------------------------ Experiment End Clock Data ----------------------- */
  GetClock();
  dayEnd = day;
  monthEnd = month;
  yearEnd = year;
  hourEnd = hour;
  minuteEnd = minute;
  secondEnd = seconds;
  /* --------------------- Save data into the microSD card -------------------- */
  SaveData();
  delay(2000);
  /* ---------------------- AC Power Supply deactivation ---------------------- */
  RelayActivation();
  delay(2000);
  /* --------------------------- Capacitor Discharge -------------------------- */
  Discharge();
  delay(2000);
  /* ------------------------------- Main Screen ------------------------------ */
  MainScreen();
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
  tft.begin(0x9341);
  MainScreen();
  /* -------------------------------- Pin Setup ------------------------------- */
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  pinMode(RCWL, INPUT);
  pinMode(PAM8406, OUTPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(RESISTOR, OUTPUT);
  for(int i=firstBAR; i<=lastBAR; i++)
  {
    pinMode(i, OUTPUT);
  }
  /* ------------------------------ DS3231 Setup ------------------------------ */
  DS3231_init(DS3231_CONTROL_INTCN);
  t.hour=setHour; 
  t.min=setMin;
  t.sec=setSec;
  t.mday=setDay;
  t.mon=setMonth;
  t.year=setYear;
  DS3231_set(t);
  /* ------------------------------ microSD Setup ----------------------------- */
  if (!SD.begin(PIN_SD_CS)){
    Serial.println("SD initialization failed.");
    while (1);
  }else
  Serial.println("SD initialization done.");
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