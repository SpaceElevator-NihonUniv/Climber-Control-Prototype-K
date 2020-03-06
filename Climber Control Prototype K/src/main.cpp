//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   Space Elevator
//Version number:  Ver.1.0
//Date:            2020.03.06
//------------------------------------------------------------------//

//This program supports the following boards:
//* M5Stack(Grey version)
 
//Include
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include "driver/pcnt.h"


//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 10                  // Timer Interrupt Period

#define ESC_LDEC_CHANNEL 3                  // 50Hz LDEC Timer

#define PULSE_INPUT_PIN 35                  // Rotaly Encoder Phase A
#define PULSE_CTRL_PIN  36                  // Rotaly Encoder Phase B
#define PCNT_H_LIM_VAL  10000               // Counter Limit H
#define PCNT_L_LIM_VAL -10000               // Counter Limit L 

#define BufferRecords 1024                  // 1Cycle Buffer Records 


//Global
//------------------------------------------------------------------//

TaskHandle_t task_handl;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10;

// PSRAM
int memMax;
char *p;

// Encoder1
int16_t delta_count = 0;                    // Delta Counter
long    total_count = 0;                    // Total Counter

int16_t delta_count_buff;
long    total_count_buff;

// ESC
static const int escPin = 26;
Servo esc;
unsigned char power = 0;
unsigned char power_buff;

// Main
unsigned char pattern = 0;
unsigned char pattern_buff;

//SD
File file;
String fname_buff;
const char* fname;

// Log
typedef struct {
    unsigned long log_time;
    unsigned char log_pattern;
    unsigned char log_power;
    int16_t log_delta_count;
    long log_total_count;
    long log_total_count2;
    long log_total_count3;
    long log_total_count4;
    long log_total_count5;
    long log_total_count6;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};



//Prototype
//------------------------------------------------------------------//
uint8_t getBatteryGauge(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void initEncoder(void);
void initPSRAM(void);
void buttonAction(void);
void lcdDisplay(void);

//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();
  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  initEncoder();
  initPSRAM();  

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 1, &task_handl, 0);

  M5.Lcd.setTextSize(2);

  

  esc.attach(escPin, ESC_LDEC_CHANNEL, 0, 100, 1100, 1940);
  esc.write(0);

}

//Main #1
//------------------------------------------------------------------//
void loop() {

  timerInterrupt();

  switch (pattern) {
  case 0:    
    esc.write(0);
    lcdDisplay();
    buttonAction();
    break;  

  case 11:
    lcdDisplay();
    esc.write(power);
    if( total_count >= 100000 || total_count <= -100000 ) {
      power = 0;
      total_count = 0;    
      pattern = 0;
      break;
    }
    break;

  }
   
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){

  disableCore0WDT();

  SD.begin(4, SPI, 24000000);
  fname_buff  = "/log/Satellite_log_.csv";
  fname = fname_buff.c_str();
  // Create Log File  

  while(1){    
    int readBank = !writeBank;
    if (bufferIndex[readBank] >= BufferRecords) {
      static RecordType temp[BufferRecords];

      memcpy(temp, buffer[readBank], sizeof(temp));
      bufferIndex[readBank] = 0;
      file = SD.open(fname, FILE_APPEND);
      for (int i = 0; i < BufferRecords; i++) {
          file.print(temp[i].log_time);
          file.print(",");
          file.print(temp[i].log_pattern);
          file.print(",");
          file.print(temp[i].log_power);
          file.print(",");
          file.print(temp[i].log_delta_count);
          file.print(",");
          file.print(temp[i].log_total_count);
          file.print(",");
          file.print(temp[i].log_total_count2);
          file.print(",");
          file.print(temp[i].log_total_count3);
          file.print(",");
          file.print(temp[i].log_total_count4);
          file.print(",");
          file.print(temp[i].log_total_count5);
          file.print(",");
          file.println(temp[i].log_total_count6);
      }
      file.close();
    }
  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void timerInterrupt(void) {
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    
    pcnt_get_counter_value(PCNT_UNIT_0, &delta_count);
    pcnt_counter_clear(PCNT_UNIT_0);  
    total_count += delta_count;

    if (bufferIndex[writeBank] < BufferRecords) {
      RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];
      rp->log_time = millis();
      rp->log_pattern = pattern;
      rp->log_power = power;
      rp->log_delta_count = delta_count;
      rp->log_total_count = total_count;
      rp->log_total_count2 = total_count;
      rp->log_total_count3 = total_count;
      rp->log_total_count4 = total_count;
      rp->log_total_count5 = total_count;
      rp->log_total_count6 = total_count;
      if (++bufferIndex[writeBank] >= BufferRecords) {
          writeBank = !writeBank;
      }
    }
    
    iTimer10++;
    switch (iTimer10) {
    case 1:
      if(pattern == 11 && (power < 100)) power++;
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
      iTimer10 = 0;
      break;
    }
  }
}

// Initialize Encoder
//------------------------------------------------------------------//
void initEncoder(void) {
  pcnt_config_t pcnt_config_1A;
    pcnt_config_1A.pulse_gpio_num = PULSE_INPUT_PIN;
    pcnt_config_1A.ctrl_gpio_num = PULSE_CTRL_PIN;
    pcnt_config_1A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1A.channel = PCNT_CHANNEL_0;
    pcnt_config_1A.unit = PCNT_UNIT_0;
    pcnt_config_1A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_1B;
    pcnt_config_1B.pulse_gpio_num = PULSE_CTRL_PIN;
    pcnt_config_1B.ctrl_gpio_num = PULSE_INPUT_PIN;
    pcnt_config_1B.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1B.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1B.channel = PCNT_CHANNEL_1;
    pcnt_config_1B.unit = PCNT_UNIT_0;
    pcnt_config_1B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1B.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1B.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_unit_config(&pcnt_config_1A);            // Initialize Unit 1A
  pcnt_unit_config(&pcnt_config_1B);            // Initialize Unit 1B
  pcnt_counter_pause(PCNT_UNIT_0);              // Stop Counter
  pcnt_counter_clear(PCNT_UNIT_0);              // clear Counter
  pcnt_counter_resume(PCNT_UNIT_0);             // Start Count
}

// Initialize PSRAM
//------------------------------------------------------------------//
void initPSRAM(void) {

  // 4194252byte >> 800ms
  // 1024byte    >> 834us
  // 64byte      >> 642us
  
  Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Serial.println("");

  // SPRAM 
  memMax= ESP.getFreePsram();
  p = (char*) ps_calloc( memMax , sizeof(char) );

  digitalWrite(26, HIGH);

  // Memory Check
  int i = 0;
  while ( i < memMax ) {
    p[i] = (char)i;
    if ( p[i] != (char)i ) {
      Serial.printf("write error at %d\n", i);
      i--;
      break;
    }
    i++;
  }

  Serial.printf("%d bytes check Ok\n", i);
  
  free(p); // Clear

}

// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {

  // Clear Display
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("Pattern: %3d", pattern_buff);  
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.printf("Delta Counter: %6d", delta_count_buff);  
  M5.Lcd.setCursor(10, 70);
  M5.Lcd.printf("Total Counter: %6d", total_count_buff); 
  M5.Lcd.setCursor(10, 100);
  M5.Lcd.printf("Motor Power: %3d", power_buff); 

  // Refresh Display
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("Pattern: %3d", pattern);  
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.printf("Counter value: %6d", delta_count);
  M5.Lcd.setCursor(10, 70);
  M5.Lcd.printf("Total Counter: %6d", total_count); 
  M5.Lcd.setCursor(10, 100);
  M5.Lcd.printf("Motor Power: %3d", power); 

  // Load Buffer
  pattern_buff = pattern;
  delta_count_buff = delta_count;
  total_count_buff = total_count;
  power_buff = power;

}

// Button Action
//------------------------------------------------------------------//
void buttonAction(void){
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if( pattern == 0 ) {
      pattern = 11;
    }
  } else if (M5.BtnB.wasPressed()) {
  } else if (M5.BtnC.wasPressed()) {
  }
}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Battery Gauge
//------------------------------------------------------------------//
uint8_t getBatteryGauge() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  Wire.endTransmission(false);
  if(Wire.requestFrom(0x75, 1)) {
    return Wire.read();
  }
  return 0xff;
}