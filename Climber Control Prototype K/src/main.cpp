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
#define M5STACK_MPU6886 

#include <stdint.h>
#include <M5Stack.h>
#include <Servo.h>
#include <EEPROM.h>
#include <LIDARLite_v3HP.h>
#include "driver/pcnt.h"


//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 10                  // Timer Interrupt Period

#define FAST_I2C
                                            // LDEC CH 0-2,6-7 not use
#define KOSMIK1_LEDC_CHANNEL 3              // 50Hz LEDC Timer
#define KOSMIK2_LEDC_CHANNEL 4              // 50Hz LEDC Timer
#define TURNIGY1_LEDC_CHANNEL 5             // 50Hz LEDC Timer
#define TURNIGY2_LEDC_CHANNEL 8             // 50Hz LEDC Timer

#define PULSE1_INPUT_PIN 35                 // Rotaly Encoder Phase A
#define PULSE1_CTRL_PIN  36                 // Rotaly Encoder Phase B
#define PULSE2_INPUT_PIN 25                 // Rotaly Encoder Phase A
#define PULSE2_CTRL_PIN  26                 // Rotaly Encoder Phase B
#define PULSE3_INPUT_PIN 2                  // Rotaly Encoder Phase A
#define PULSE3_CTRL_PIN  5                  // Rotaly Encoder Phase B
#define PCNT_H_LIM_VAL  10000               // Counter Limit H
#define PCNT_L_LIM_VAL -10000               // Counter Limit L 

#define BufferRecords 64                    // 1Cycle Buffer Records 


//Global
//------------------------------------------------------------------//

TaskHandle_t task_handl;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10;

// PSRAM
// platformio.ini Add ↓
// build_flags =
//    -DBOARD_HAS_PSRAM
//    -mfix-esp32-psram-cache-issue
int memMax;
char *p;

// Encoder
int16_t delta_count1 = 0;                    // Delta Counter
long    total_count1 = 0;                    // Total Counter
int16_t delta_count2 = 0;                    // Delta Counter
long    total_count2 = 0;                    // Total Counter
int16_t delta_count3 = 0;                    // Delta Counter
long    total_count3 = 0;                    // Total Counter

// ESC Kosmik
static const int kosmik1Pin = 15;
Servo kosmik1;
static const int kosmik2Pin = 0;
Servo kosmik2;
unsigned char power = 0;

// ESC Turnigy
static const int turnigy1Pin = 12;
Servo turnigy1;
static const int turnigy2Pin = 13;
Servo turnigy2;

// LIDAR
LIDARLite_v3HP LidarLite1;
uint16_t distance1;
uint8_t  newDistance1 = 0;

// Main
unsigned char pattern = 0;
unsigned long seq;
unsigned long seq_buff;
unsigned long time_buff = 0;

// MPU
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

// Battery
unsigned char battery_status;
unsigned char battery_persent;

// RSSI
const int rssiPin = 34;
unsigned int duration;
int rssi_value;

//SD
File file;
String fname_buff;
const char* fname;

// Log
typedef struct {
    unsigned long log_time;
    unsigned long log_seq;
    unsigned char log_pattern;
    unsigned char log_power;
    int16_t log_delta_count1;
    long log_total_count1;
    int16_t log_delta_count2;
    long log_total_count2;
    int16_t log_delta_count3;
    long log_total_count3;
    uint16_t log_distance1;
    //float log_IMU_ax;
    //float log_IMU_ay;
    //float log_IMU_az;
    //float log_IMU_gx;
    //float log_IMU_gy;
    //float log_IMU_gz;
    //float log_IMU_pitch;
    //float log_IMU_roll;
    //float log_IMU_yaw;
    //float log_IMU_temp;
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
void initLCD(void);
void lcdDisplay(void);
uint8_t distanceContinuous(uint16_t * distance1);

//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();

  Serial2.begin(57600);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  initEncoder();
  //initPSRAM();  

  pinMode(rssiPin, INPUT);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 1, &task_handl, 0);

  M5.Lcd.setTextSize(2);
  initLCD();

  // Initialize IIC
  Wire.begin();
  Wire.setClock(400000);


  // Initialize MPU
  //M5.IMU.Init();
  
  kosmik1.attach(kosmik1Pin, KOSMIK1_LEDC_CHANNEL, 0, 100, 1100, 1940);
  kosmik1.write(0);
  kosmik2.attach(kosmik2Pin, KOSMIK2_LEDC_CHANNEL, 0, 100, 1100, 1940);
  kosmik2.write(50);
  turnigy1.attach(turnigy1Pin, TURNIGY1_LEDC_CHANNEL, 0, 100, 1100, 1940);
  turnigy1.write(75);
  turnigy2.attach(turnigy2Pin, TURNIGY2_LEDC_CHANNEL, 0, 100, 1100, 1940);
  turnigy2.write(100);

  seq_buff = millis();  

  LidarLite1.configure(0); 
  delay(100);

}

//Main #1
//------------------------------------------------------------------//
void loop() {

  timerInterrupt();

  switch (pattern) {
  case 0:    
    lcdDisplay();
    kosmik1.write(0);    
    buttonAction();
    break;  

  case 11:
    lcdDisplay();
    kosmik1.write(power);
    if( total_count1 >= 100000 || total_count1 <= -100000 ) {
      power = 0;
      total_count1 = 0; 
      time_buff = millis();
      seq_buff = millis();
      pattern = 101;
      break;
    }
    break;

  case 101:
    lcdDisplay();
    kosmik1.write(0);
    if( millis() - time_buff >= 5000 ) {
      time_buff = 0;
      seq_buff = millis();
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

  SD.begin(4, SPI, 40000000);
  // Create Log File  
  fname_buff  = "/Climber_log.csv";
  fname = fname_buff.c_str(); 
  file = SD.open(fname, FILE_APPEND); 
  file.print("Time");
  file.print(",");
  file.print("Sequence");
  file.print(",");
  file.print("Pattern");
  file.print(",");
  file.print("Power");
  file.print(",");
  file.print("Delta Count1");
  file.print(",");
  file.print("Total Count1");
  file.print(",");
  file.print("Delta Count2");
  file.print(",");
  file.print("Total Count2");
  file.print(",");
  file.print("Delta Count3");
  file.print(",");
  file.print("Total Count3");
  file.print(",");
  file.println("Distance1");
  //file.print(",");
  //file.print("AX");
  //file.print(",");
  //file.print("AY");
  //file.print(",");
  //file.print("AZ");
  //file.print(",");
  //file.print("GX");
  //file.print(",");
  //file.print("GY");
  //file.print(",");
  //file.print("GZ"); 
  //file.print(","); 
  //file.print("Pitch");
  //file.print(",");
  //file.print("Roll");
  //file.print(",");
  //file.print("Yaw");
  //file.print(",");
  //file.println("Temp");  
  file.close();

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
          file.print(temp[i].log_seq);
          file.print(",");
          file.print(temp[i].log_pattern);
          file.print(",");
          file.print(temp[i].log_power);
          file.print(",");
          file.print(temp[i].log_delta_count1);
          file.print(",");
          file.print(temp[i].log_total_count1);
          file.print(",");
          file.print(temp[i].log_delta_count2);
          file.print(",");
          file.print(temp[i].log_total_count2);
          file.print(",");
          file.print(temp[i].log_delta_count3);
          file.print(",");
          file.print(temp[i].log_total_count3);
          file.print(",");
          file.println(temp[i].log_distance1);
          //file.print(",");
          //file.print(temp[i].log_IMU_ax);
          //file.print(",");
          //file.print(temp[i].log_IMU_ay);
          //file.print(",");
          //file.print(temp[i].log_IMU_az);
          //file.print(",");
          //file.print(temp[i].log_IMU_gx);
          //file.print(",");
          //file.print(temp[i].log_IMU_gy);
          //file.print(",");
          //file.print(temp[i].log_IMU_gz);
          //file.print(",");
          //file.print(temp[i].log_IMU_pitch);
          //file.print(",");
          //file.print(temp[i].log_IMU_roll);
          //file.print(",");
          //file.print(temp[i].log_IMU_yaw);
          //file.print(",");
          //file.println(temp[i].log_IMU_temp);
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
    
    pcnt_get_counter_value(PCNT_UNIT_0, &delta_count1);
    pcnt_counter_clear(PCNT_UNIT_0);  
    total_count1 += delta_count1;
    pcnt_get_counter_value(PCNT_UNIT_1, &delta_count2);
    pcnt_counter_clear(PCNT_UNIT_1);  
    total_count2 += delta_count2;
    pcnt_get_counter_value(PCNT_UNIT_2, &delta_count3);
    pcnt_counter_clear(PCNT_UNIT_2);  
    total_count3 += delta_count3;

    Serial2.printf("Time %d\n",millis());

    seq = millis() - seq_buff;

    //M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    //M5.IMU.getAccelData(&accX,&accY,&accZ);
    //M5.IMU.getAhrsData(&pitch,&roll,&yaw);
    //M5.IMU.getTempData(&temp);

    newDistance1 = distanceContinuous(&distance1);

    if (pattern >= 11 && bufferIndex[writeBank] < BufferRecords) {
      RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];
      rp->log_time = millis();
      rp->log_seq = seq;
      rp->log_pattern = pattern;
      rp->log_power = power;
      rp->log_delta_count1 = delta_count1;
      rp->log_total_count1 = total_count1;
      rp->log_delta_count2 = delta_count2;
      rp->log_total_count2 = total_count2;
      rp->log_delta_count3 = delta_count3;
      rp->log_total_count3 = total_count3;
      rp->log_distance1 = distance1;
      //rp->log_IMU_ax = accX;
      //rp->log_IMU_ay = accY;
      //rp->log_IMU_az = accZ;
      //rp->log_IMU_gx = gyroX;
      //rp->log_IMU_gy = gyroY;
      //rp->log_IMU_gz = gyroZ;
      //rp->log_IMU_pitch = pitch;
      //rp->log_IMU_roll = roll;
      //rp->log_IMU_yaw = yaw;
      //rp->log_IMU_temp = temp;
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
      duration = pulseIn(rssiPin, HIGH, 100);
      if( duration == 0 ) {
        if( digitalRead(rssiPin) ) {
          duration = 70;
        }
      }      
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
    pcnt_config_1A.pulse_gpio_num = PULSE1_INPUT_PIN;
    pcnt_config_1A.ctrl_gpio_num = PULSE1_CTRL_PIN;
    pcnt_config_1A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1A.channel = PCNT_CHANNEL_0;
    pcnt_config_1A.unit = PCNT_UNIT_0;
    pcnt_config_1A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_1B;
    pcnt_config_1B.pulse_gpio_num = PULSE1_CTRL_PIN;
    pcnt_config_1B.ctrl_gpio_num = PULSE1_INPUT_PIN;
    pcnt_config_1B.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1B.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1B.channel = PCNT_CHANNEL_1;
    pcnt_config_1B.unit = PCNT_UNIT_0;
    pcnt_config_1B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1B.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1B.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_2A;
    pcnt_config_2A.pulse_gpio_num = PULSE2_INPUT_PIN;
    pcnt_config_2A.ctrl_gpio_num = PULSE2_CTRL_PIN;
    pcnt_config_2A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_2A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_2A.channel = PCNT_CHANNEL_0;
    pcnt_config_2A.unit = PCNT_UNIT_1;
    pcnt_config_2A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_2A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_2A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_2A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_2B;
    pcnt_config_2B.pulse_gpio_num = PULSE2_CTRL_PIN;
    pcnt_config_2B.ctrl_gpio_num = PULSE2_INPUT_PIN;
    pcnt_config_2B.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_2B.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_2B.channel = PCNT_CHANNEL_1;
    pcnt_config_2B.unit = PCNT_UNIT_1;
    pcnt_config_2B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_2B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_2B.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_2B.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_3A;
    pcnt_config_3A.pulse_gpio_num = PULSE3_INPUT_PIN;
    pcnt_config_3A.ctrl_gpio_num = PULSE3_CTRL_PIN;
    pcnt_config_3A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_3A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_3A.channel = PCNT_CHANNEL_0;
    pcnt_config_3A.unit = PCNT_UNIT_2;
    pcnt_config_3A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_3A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_3A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_3A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_3B;
    pcnt_config_3B.pulse_gpio_num = PULSE3_CTRL_PIN;
    pcnt_config_3B.ctrl_gpio_num = PULSE3_INPUT_PIN;
    pcnt_config_3B.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_3B.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_3B.channel = PCNT_CHANNEL_1;
    pcnt_config_3B.unit = PCNT_UNIT_2;
    pcnt_config_3B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_3B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_3B.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_3B.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_unit_config(&pcnt_config_1A);            // Initialize Unit 1A
  pcnt_unit_config(&pcnt_config_1B);            // Initialize Unit 1B
  pcnt_unit_config(&pcnt_config_2A);            // Initialize Unit 1A
  pcnt_unit_config(&pcnt_config_2B);            // Initialize Unit 1B
  pcnt_unit_config(&pcnt_config_3A);            // Initialize Unit 1A
  pcnt_unit_config(&pcnt_config_3B);            // Initialize Unit 1B
  pcnt_counter_pause(PCNT_UNIT_0);              // Stop Counter
  pcnt_counter_pause(PCNT_UNIT_1);              // Stop Counter
  pcnt_counter_pause(PCNT_UNIT_2);              // Stop Counter
  pcnt_counter_clear(PCNT_UNIT_0);              // clear Counter
  pcnt_counter_clear(PCNT_UNIT_1);              // clear Counter
  pcnt_counter_clear(PCNT_UNIT_2);              // clear Counter
  pcnt_counter_resume(PCNT_UNIT_0);             // Start Count
  pcnt_counter_resume(PCNT_UNIT_1);             // Start Count
  pcnt_counter_resume(PCNT_UNIT_2);             // Start Count
}

// Lidar
//------------------------------------------------------------------//
uint8_t distanceContinuous(uint16_t * distance1)
{
    newDistance1 = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (LidarLite1.getBusyFlag() == 0)
    {
        // Trigger the next range measurement
        LidarLite1.takeRange();

        // Read new distance data from device registers
        *distance1 = LidarLite1.readDistance();

        // Report to calling function that we have new data
        newDistance1 = 1;
    }

    return newDistance1;
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

// Initialize LCD
//------------------------------------------------------------------//
void initLCD(void) {

  // Refresh Display
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("Pattern:");  
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.printf("Total Counter 1:"); 
  M5.Lcd.setCursor(10, 70);
  M5.Lcd.printf("Total Counter 2:"); 
  M5.Lcd.setCursor(10, 100);
  M5.Lcd.printf("Total Counter 3:"); 
  M5.Lcd.setCursor(10, 130);
  M5.Lcd.printf("Motor Power :"); 
  M5.Lcd.setCursor(10, 160);
  M5.Lcd.printf("Distance1 :"); 
  M5.Lcd.setCursor(10, 190);
  M5.Lcd.printf("RSSI :"); 

}


// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {

  // Refresh Display
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(220, 10);
  M5.Lcd.printf("%6d", pattern);  
  M5.Lcd.setCursor(220, 40);
  M5.Lcd.printf("%6d", total_count1); 
  M5.Lcd.setCursor(220, 70);
  M5.Lcd.printf("%6d", total_count2); 
  M5.Lcd.setCursor(220, 100);
  M5.Lcd.printf("%6d", total_count3);
  M5.Lcd.setCursor(220, 130);
  M5.Lcd.printf("%3d", power);
  M5.Lcd.setCursor(220, 160);
  M5.Lcd.printf("%3d", distance1);
  M5.Lcd.setCursor(220, 190);
  M5.Lcd.printf("%3d", duration);

}

// Button Action
//------------------------------------------------------------------//
void buttonAction(void){
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if( pattern == 0 ) {
      seq_buff = millis();
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