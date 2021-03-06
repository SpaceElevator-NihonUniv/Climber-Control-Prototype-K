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
#include <Avatar.h>
#include <faces/DogFace.h>
#include <Servo.h>
#include <EEPROM.h>
#include <LIDARLite_v3HP.h>
#include "driver/pcnt.h"
#include <utility/MahonyAHRS.h>


//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 1                   // Timer Interrupt Period

#define FAST_I2C
                                            // LDEC CH 0-2,6-7 not use
#define KOSMIK1_LEDC_CHANNEL 3              // 50Hz LEDC Timer
#define KOSMIK2_LEDC_CHANNEL 4              // 50Hz LEDC Timer
#define TURNIGY1_LEDC_CHANNEL 5             // 50Hz LEDC Timer
#define TURNIGY2_LEDC_CHANNEL 8             // 50Hz LEDC Timer

#define PULSE1_INPUT_PIN 35                 // Rotaly Encoder Phase A
#define PULSE1_CTRL_PIN  36                 // Rotaly Encoder Phase B
#define PULSE2_INPUT_PIN 2                  // Rotaly Encoder Phase A
#define PULSE2_CTRL_PIN  5                  // Rotaly Encoder Phase B
#define PULSE3_INPUT_PIN 25                 // Rotaly Encoder Phase A
#define PULSE3_CTRL_PIN  26                 // Rotaly Encoder Phase B
#define PCNT_H_LIM_VAL  10000               // Counter Limit H
#define PCNT_L_LIM_VAL -10000               // Counter Limit L 

#define BufferRecords 64                    // 1Cycle Buffer Records 


//Global
//------------------------------------------------------------------//
TaskHandle_t task_handl;

// Avatar
using namespace m5avatar;
Avatar avatar;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10 = 0;
int iTimer50 = 0;

// Avatar
Face* faces[2];
const int facesSize = sizeof(faces) / sizeof(Face*);
int faceIdx = 0;

const Expression expressions[] = {
  Expression::Angry,
  Expression::Sleepy,
  Expression::Happy,
  Expression::Sad,
  Expression::Doubt,
  Expression::Neutral
};
const int expressionsSize = sizeof(expressions) / sizeof(Expression);
int idx = 0;

ColorPalette* cps[4];
const int cpsSize = sizeof(cps) / sizeof(ColorPalette*);
int cpsIdx = 0;

bool isShowingQR = false;

bool avatar_flag = false;
int avatar_cnt = 0;

unsigned char lcd_pattern = 0;
unsigned char lcd_cnt = 0;

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

// LED 
static const int ledPin = 1;
bool led_flag = false;

// LIDAR
LIDARLite_v3HP LidarLite1;
uint16_t distance1;
uint8_t  newDistance1 = 0;

LIDARLite_v3HP LidarLite2;
uint16_t distance2;
uint8_t  newDistance2 = 0;

// XBee
char     tx_pattern = 1;
char     rx_pattern = 0;
int      rx_val = 0;
char     xbee_rx_buffer[16];
int      xbee_index = 0;

// Main
String f270 = "/font/Arial270";
volatile unsigned char pattern = 0;
volatile long seq;
volatile long seq_buff;
volatile long seq_comp = 0;
volatile unsigned long time_buff = 0;
volatile bool  lcd_flag = false;
volatile unsigned int start_cnt = 0;
volatile unsigned int climbing_height = 15;
volatile unsigned char climbing_velocity = 100;
volatile unsigned char climbing_accel = 20;
volatile unsigned char decending_velocity = 5;
volatile unsigned char starting_delay = 15;
volatile unsigned char interval_time = 10;
volatile bool sleep_flag = true;
bool sd_insert = false;

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
char battery_persent;

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
    float log_IMU_ax;
    float log_IMU_ay;
    float log_IMU_az;
    float log_IMU_gx;
    float log_IMU_gy;
    float log_IMU_gz;
    float log_IMU_pitch;
    float log_IMU_roll;
    float log_IMU_yaw;
    float log_IMU_temp;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};

//Prototype
//------------------------------------------------------------------//
int8_t getBatteryLevel(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void initEncoder(void);
void initPSRAM(void);
void buttonAction(void);
void initLCD(void);
void lcdDisplay(void);
uint8_t distanceContinuous(uint16_t * distance1);
uint8_t distanceFast(uint16_t * distance1);
void xbee_rx(void);
void xbee_tx(void);
void start_sequence(void);
void eeprom_write(void);
void eeprom_read(void);

//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin(1, 1, 0, 1);

  //dacWrite(25, 0); 

  Serial2.begin(115200);
  EEPROM.begin(128);
  delay(10);
  eeprom_read();

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
  pinMode(3, OUTPUT);

  M5.Lcd.setTextSize(2);
  
  // Initialize IIC
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU
  M5.IMU.Init();
  
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

  if( sleep_flag ) {
    faces[0] = avatar.getFace();

    cps[0] = new ColorPalette();
    cps[1] = new ColorPalette();
    cps[0]->set(COLOR_PRIMARY, WHITE);
    cps[0]->set(COLOR_BACKGROUND, BLACK);
    cps[1]->set(COLOR_PRIMARY, DARKGREY);
    cps[1]->set(COLOR_BACKGROUND, WHITE);

    avatar.init();
    avatar.setExpression(expressions[2]);
    delay(2000);
    avatar.stop();  
    avatar.setExpression(expressions[5]);
    delay(100);  
    M5.Lcd.clear(); 
  } 

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 6144, NULL, 1, &task_handl, 0);

  initLCD();    
  
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

  case 1:
    buttonAction();
    break;

  // Emargency Escape
  case 2:
    if(M5.Lcd.fontsLoaded()){
      M5.Lcd.unloadFont();
    }
    pattern = 0;
    M5.Lcd.clear();  
    initLCD();    
    break;

  case 11:
    kosmik1.write(power);
    if( total_count2 >= 100000 || total_count2 <= -100000 ) {
      power = 0;
      total_count2 = 0; 
      time_buff = millis();
      seq_buff = millis();
      pattern = 101;
      break;
    }
    break;

  case 101:
    kosmik1.write(0);
    if( millis() - time_buff >= 5000 ) {
      time_buff = 0;
      seq_buff = millis();
      pattern = 0;
      initLCD();  
      break;
    }    
    break;

  // Start Sequence
  case 201:
    start_sequence();
    break;

  case 202:
    seq_buff = millis();
    M5.Lcd.clear();
    pattern = 11;
    break;

  }
   
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){

  disableCore0WDT();

  sd_insert = SD.begin(TFCARD_CS_PIN, SPI, 40000000);

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
  file.print("Distance1");
  file.print(",");
  file.print("AX");
  file.print(",");
  file.print("AY");
  file.print(",");
  file.print("AZ");
  file.print(",");
  file.print("GX");
  file.print(",");
  file.print("GY");
  file.print(",");
  file.print("GZ"); 
  file.print(","); 
  file.print("Pitch");
  file.print(",");
  file.print("Roll");
  file.print(",");
  file.print("Yaw");
  file.print(",");
  file.println("Temp");  
  file.close();

  while(1){    

    xbee_rx();
    xbee_tx();
    
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
          file.print(temp[i].log_distance1);
          file.print(",");
          file.print(temp[i].log_IMU_ax);
          file.print(",");
          file.print(temp[i].log_IMU_ay);
          file.print(",");
          file.print(temp[i].log_IMU_az);
          file.print(",");
          file.print(temp[i].log_IMU_gx);
          file.print(",");
          file.print(temp[i].log_IMU_gy);
          file.print(",");
          file.print(temp[i].log_IMU_gz);
          file.print(",");
          file.print(temp[i].log_IMU_pitch);
          file.print(",");
          file.print(temp[i].log_IMU_roll);
          file.print(",");
          file.print(temp[i].log_IMU_yaw);
          file.print(",");
          file.println(temp[i].log_IMU_temp);
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

    seq = millis() - seq_buff; 

    digitalWrite(3, led_flag);  
    led_flag = !led_flag;         
    
    iTimer10++;
    // 10ms timerinterrupt
    switch (iTimer10) {
    case 1:      
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
        rp->log_IMU_ax = accX;
        rp->log_IMU_ay = accY;
        rp->log_IMU_az = accZ;
        rp->log_IMU_gx = gyroX;
        rp->log_IMU_gy = gyroY;
        rp->log_IMU_gz = gyroZ;
        rp->log_IMU_pitch = pitch;
        rp->log_IMU_roll = roll;
        rp->log_IMU_yaw = yaw;
        rp->log_IMU_temp = temp;
        if (++bufferIndex[writeBank] >= BufferRecords) {
            writeBank = !writeBank;
        }
      }
      break;
    case 2:
      distanceContinuous(&distance1);
      break;
    case 3:      
      break;
    case 4:
      M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);      
      break;
    case 5:
      M5.IMU.getAccelData(&accX,&accY,&accZ);
      break;
    case 6:
      M5.IMU.getAhrsData(&pitch,&roll,&yaw);
      break;
    case 7:
      M5.IMU.getTempData(&temp);
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:      
      iTimer10 = 0;      
      break;
    }    
    
    iTimer50++;
    // 50ms timerinterrupt
    switch (iTimer50) {
    case 10:
      if(pattern == 11 && (power < 100)) power++;
      break;
    case 20:
      duration = pulseIn(rssiPin, HIGH, 100);
      if( duration == 0 ) {
        if( digitalRead(rssiPin) ) {
          duration = 60;
        }
      }      
      break;
    case 30:     
      if( tx_pattern == 101 ) {
        Serial2.printf("%3d, ",millis()/1000);
        Serial2.printf("%3d, ",seq/1000);
        Serial2.printf("%3d, ",pattern);
        Serial2.printf("%3d, ",power);
        Serial2.printf("%5d, ",delta_count1);
        Serial2.printf("%7d, ",total_count1);
        Serial2.printf("%5d, ",delta_count2);
        Serial2.printf("%7d, ",total_count2);
        Serial2.printf("%5d, ",delta_count3);
        Serial2.printf("%7d, ",total_count3);
        Serial2.printf("%5d, ",distance1);
        Serial2.printf("%5d, ",distance2);
        Serial2.printf("%2d",duration);
        Serial2.printf("\n");
      }
      break;
    case 40:         
      battery_persent = getBatteryLevel(); 
      break;
    case 50:      
      if( sleep_flag ) {
        if( pattern == 0 && !avatar_flag && duration == 0 ) {
          avatar_cnt++;
          if( avatar_cnt > 600 ) {
            avatar_flag = true;
            avatar.start();
            avatar.setColorPalette(*cps[1]);
            pattern = 1;
          }
        } else {
          avatar_cnt = 0;
        }
      }
      if(lcd_pattern>=10) {
        lcd_flag = true;
      } else {
        lcd_cnt++;
        if(lcd_cnt>5) {
          lcd_flag = true;
          lcd_cnt = 0;
        }
      }      
      iTimer50 = 0;
      break;
    }
  }
}

// XBee RX
//------------------------------------------------------------------//
void xbee_rx(void) {

  while (Serial2.available()) {
    xbee_rx_buffer[xbee_index] = Serial2.read();
    Serial2.write(xbee_rx_buffer[xbee_index]);

    if( xbee_rx_buffer[xbee_index] == '/' ) {
      Serial2.print("\n\n"); 
      if( tx_pattern == 0 ) {
        rx_pattern = atoi(xbee_rx_buffer);
      } else if( tx_pattern == 2 ) {
        rx_val = atof(xbee_rx_buffer);
      }
      xbee_index = 0;
      
      switch ( rx_pattern ) {          
      case 0:
        tx_pattern = 1;
        break;
        
      case 11:      
        tx_pattern = 11;
        rx_pattern = 21;
        break;
      case 21:
        if( rx_val == 1 ) {          
          seq_buff = millis();
          seq_comp = 0;
          power = 0;
          total_count1 = 0;
          total_count2 = 0;
          total_count3 = 0;
          M5.Lcd.clear();
          Serial2.printf("\n");
          tx_pattern = 101;
          M5.Lcd.setCursor(20, 20);
          M5.Lcd.setTextSize(4);
          M5.Lcd.printf("UPCOMING:");  
          M5.Lcd.setCursor(20, 100);
          M5.Lcd.setTextSize(7);
          M5.Lcd.printf("LIFTOFF"); 
          delay(1000);
          M5.Lcd.loadFont(f270, SD);
          pattern = 201;
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;

      case 31:
        tx_pattern = 31;
        rx_pattern = 41;
        break;
      case 41:
        climbing_height = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;

      case 32:
        tx_pattern = 32;
        rx_pattern = 42;
        break;
      case 42:
        climbing_velocity = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;

      case 33:
        tx_pattern = 33;
        rx_pattern = 43;
        break;
      case 43:
        climbing_accel = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;

      case 34:
        tx_pattern = 34;
        rx_pattern = 44;
        break;
      case 44:
        decending_velocity = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;

      case 35:
        tx_pattern = 35;
        rx_pattern = 45;
        break;
      case 45:
        starting_delay = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;
      
      case 36:
        tx_pattern = 36;
        rx_pattern = 46;
        break;
      case 46:
        interval_time = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;

      case 37:
        tx_pattern = 37;
        rx_pattern = 47;
        break;
      case 47:
        sleep_flag = rx_val;
        eeprom_write();
        tx_pattern = 1;
        rx_pattern = 0;
        break;


      }      
      
    } else if( xbee_rx_buffer[xbee_index] == 'T' || xbee_rx_buffer[xbee_index] == 't' ) {
      rx_pattern = 0;
      tx_pattern = 101;
      Serial2.printf("\n");
    } else if( xbee_rx_buffer[xbee_index] == 'W' || xbee_rx_buffer[xbee_index] == 'w' ) {
      if ( pattern == 1 ) {
        avatar.stop();
        delay(100);
        avatar_flag = false;
        avatar.setColorPalette(*cps[0]);
        avatar_cnt = 0;    
        pattern = 2;             
      } 
    } else if( xbee_rx_buffer[xbee_index] == ' ') {
      pattern = 2;
      Serial2.printf("\n");
      Serial2.printf(" Emargency Stop Enable \n");
      Serial2.printf(" Return Case 0 \n");
      rx_pattern = 0;
      tx_pattern = 1;      
      Serial2.printf("\n");
    } else {
        xbee_index++;
    }
    avatar_cnt = 0;
  }

}

// XBee TX
//------------------------------------------------------------------//
void xbee_tx(void) {

  switch ( tx_pattern ) {   
    // Waiting Command
    case 0:
      break;

    case 1:
      Serial2.printf("\n\n\n\n\n\n");
      Serial2.printf(" Climber Controller (M5Stack version) "
                      "Test Program Ver1.20\n");
      Serial2.printf("\n");
      Serial2.printf(" 11 : Start Seqence\n");
      Serial2.printf("\n");
      Serial2.printf(" 31 : Climbing Height    [%4d]\n", climbing_height);
      Serial2.printf(" 32 : Climbing Velocity  [%4d]\n", climbing_velocity);
      Serial2.printf(" 33 : Climbing Accel     [%4d]\n", climbing_accel);
      Serial2.printf(" 34 : Decending Velocity [%4d]\n", decending_velocity);
      Serial2.printf(" 35 : Starting delay     [%4d]\n", starting_delay);
      Serial2.printf(" 36 : Interval Time      [%4d]\n", interval_time);
      Serial2.printf(" 37 : Sleep Flag         [%4d]\n", sleep_flag);
      Serial2.printf("\n");
      Serial2.printf(" T : Telemetry\n");
      
      Serial2.printf("\n");
      Serial2.printf(" Please enter 11 to 35  ");
      
      tx_pattern = 0;
      break;

    // Waiting Value
    case 2:
      break;

    case 11:
      Serial2.printf("\n Check Current Parameters\n");
      Serial2.printf("\n");
      Serial2.printf(" Climbing Height    [%4d]\n", climbing_height);
      Serial2.printf(" Climbing Velocity  [%4d]\n", climbing_velocity);
      Serial2.printf(" Climbing Accel     [%4d]\n", climbing_accel);
      Serial2.printf(" Decending Velocity [%4d]\n", decending_velocity);
      Serial2.printf(" Starting delay     [%4d]\n", starting_delay);
      Serial2.printf(" Interval Time      [%4d]\n", interval_time);
      Serial2.printf(" Sleep Flag         [%4d]\n", sleep_flag);
      Serial2.printf("\n");
      Serial2.printf(" Confirm to Climb? -> ");
      tx_pattern = 2;
      break;

    case 31:
      Serial2.printf(" Set Climbing Height [%4dm]\n", climbing_height);
      Serial2.printf(" Please enter 0 to 1000 -> ");
      tx_pattern = 2;
      break;

    case 32:
      Serial2.printf(" Set Climbing Velocity [%4dkm/h]\n", climbing_velocity);
      Serial2.printf(" Please enter 0 to 240 -> \n");
      tx_pattern = 2;
      break;

    case 33:
      Serial2.printf(" Climbing Accel [%4dm/s2]\n", climbing_accel);
      Serial2.printf(" Please enter 0 to 50 -> ");
      tx_pattern = 2;
      break;

    case 34:
      Serial2.printf(" Decending Velocity [%4dkm/h]\n", decending_velocity);
      Serial2.printf(" Please enter 0 to 50 -> ");
      tx_pattern = 2;
      break;

    case 35:
      Serial2.printf(" Starting Delay [%4ds]\n", starting_delay);
      Serial2.printf(" Please enter 0 to 60 -> ");
      tx_pattern = 2;
      break;

    case 36:
      Serial2.printf(" Interval Time [%4ds]\n", interval_time);
      Serial2.printf(" Please enter 0 to 60 -> ");
      tx_pattern = 2;
      break;

    case 37:
      Serial2.printf(" Sleep Flag  [%4d]\n", sleep_flag);
      Serial2.printf(" Please enter 0 or 1 -> ");
      tx_pattern = 2;
      break;

    // Telemetry Mode
    case 101:
      break;

  }
}


// Initialize LCD
//------------------------------------------------------------------//
void initLCD(void) {

  M5.Lcd.clear();   
  switch (lcd_pattern) {
  case 0:         
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(20, 10);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("T+:");  
    M5.Lcd.setCursor(20, 60);
    if( sd_insert ) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-sd.jpg", 212, 0);
    }
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("Case");  
    M5.Lcd.setCursor(100, 100);
    M5.Lcd.setTextSize(15);
    M5.Lcd.printf("%3d",pattern);  
    M5.Lcd.setCursor(40, 200);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("ALL SYSTEMS ARE REDAY");  
    break;
  //case 10:
  //  M5.Lcd.setTextColor(WHITE, BLACK);
  //  M5.Lcd.setCursor(10, 10);
  //  M5.Lcd.printf("Total Counter 1:"); 
  //  M5.Lcd.setCursor(10, 40);
  //  M5.Lcd.printf("Total Counter 2:"); 
  //  M5.Lcd.setCursor(10, 70);
  //  M5.Lcd.printf("Total Counter 3:"); 
  //  M5.Lcd.setCursor(10, 100);
  //  M5.Lcd.printf("Distance1 :"); 
  //  M5.Lcd.setCursor(10, 130);
  //  M5.Lcd.printf("Distance2 :"); 
  //break;
  case 10:
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("AX:"); 
    M5.Lcd.setCursor(10, 30);
    M5.Lcd.printf("AY:"); 
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.printf("AZ:"); 
    M5.Lcd.setCursor(10, 70);
    M5.Lcd.printf("GX:"); 
    M5.Lcd.setCursor(10, 90);
    M5.Lcd.printf("GY:"); 
    M5.Lcd.setCursor(10, 110);
    M5.Lcd.printf("GZ:"); 
    M5.Lcd.setCursor(10, 130);
    M5.Lcd.printf("DX:"); 
    M5.Lcd.setCursor(10, 150);
    M5.Lcd.printf("DY:"); 
    M5.Lcd.setCursor(10, 170);
    M5.Lcd.printf("DZ:"); 
    M5.Lcd.setCursor(10, 190);
    M5.Lcd.printf("T:"); 
  break;
  case 20:   
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("Climbing Height:"); 
    M5.Lcd.setCursor(10, 40);
    M5.Lcd.printf("Climbing Velocity:"); 
    M5.Lcd.setCursor(10, 70);
    M5.Lcd.printf("Climbing Accel:"); 
    M5.Lcd.setCursor(10, 100);
    M5.Lcd.printf("Decending Velocity:"); 
    M5.Lcd.setCursor(10, 130);
    M5.Lcd.printf("Starting delay:");   
    M5.Lcd.setCursor(10, 160);
    M5.Lcd.printf("Interval Time:");   
    M5.Lcd.setCursor(10, 190);
    M5.Lcd.printf("Sleep Flag:");   
    break;
  }
}


// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {
  unsigned int time_calc, time_h, time_m, time_s;

  if( lcd_flag ) {    
    // Refresh Display
    switch (lcd_pattern) {
    case 0:
      time_calc = millis() / 1000;
      time_h = time_calc / 3600;
      time_calc %= 3600;
      time_m = time_calc / 60;
      time_calc %= 60;
      time_s = time_calc;  

      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(60, 10);
      M5.Lcd.printf("%02d:%02d:%02d", time_h, time_m, time_s);
      if( battery_persent == 100) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 290, 0);
      } else if( battery_persent == 75) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-75.jpg", 290, 0);
      } else if( battery_persent == 50) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-50.jpg", 290, 0);
      } else if( battery_persent == 25) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-25.jpg", 290, 0);
      } else {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-0.jpg", 290, 0);
      }
      if( duration > 55 ) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-100.jpg", 250, 0);
      } else if( duration > 48 ) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-75.jpg", 250, 0);
      } else if( duration > 41 ) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-50.jpg", 250, 0);
      } else if( duration > 34 ) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-25.jpg", 250, 0);
      } else {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-0.jpg", 250, 0);
      }      
      lcd_flag = false; 
      break;
    //case 10:
    //  M5.Lcd.setTextColor(WHITE, BLACK);
    //  M5.Lcd.setCursor(220, 10);
    //  M5.Lcd.printf("%7d", total_count1); 
    //  M5.Lcd.setCursor(220, 40);
    //  M5.Lcd.printf("%7d", total_count2); 
    //  M5.Lcd.setCursor(220, 70);
    //  M5.Lcd.printf("%7d", total_count3);
    //  M5.Lcd.setCursor(220, 100);
    //  M5.Lcd.printf("%7d", distance1);
    //  M5.Lcd.setCursor(220, 130);
    //  M5.Lcd.printf("%7d", distance2);
    //  lcd_flag = false;
    //  break;
    case 10:
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(220, 10);
      M5.Lcd.printf("%2.2f", accX); 
      M5.Lcd.setCursor(220, 30);
      M5.Lcd.printf("%2.2f", accY); 
      M5.Lcd.setCursor(220, 50);
      M5.Lcd.printf("%2.2f", accZ);
      M5.Lcd.setCursor(220, 70);
      M5.Lcd.printf("%3.2f", gyroX);
      M5.Lcd.setCursor(220, 90);
      M5.Lcd.printf("%3.2f", gyroY);
      M5.Lcd.setCursor(220, 110);
      M5.Lcd.printf("%3.2f", gyroZ);
      M5.Lcd.setCursor(220, 130);
      M5.Lcd.printf("%3.2f", pitch);
      M5.Lcd.setCursor(220, 150);
      M5.Lcd.printf("%3.2f", roll);
      M5.Lcd.setCursor(220, 170);
      M5.Lcd.printf("%3.2f", yaw);
      M5.Lcd.setCursor(220, 190);
      M5.Lcd.printf("%2.2f", temp);
      lcd_flag = false;
      break;
    case 20:
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(220, 10);
      M5.Lcd.printf("%7d", climbing_height); 
      M5.Lcd.setCursor(220, 40);
      M5.Lcd.printf("%7d", climbing_velocity); 
      M5.Lcd.setCursor(220, 70);
      M5.Lcd.printf("%7d", climbing_accel);
      M5.Lcd.setCursor(220, 100);
      M5.Lcd.printf("%7d", decending_velocity);
      M5.Lcd.setCursor(220, 130);
      M5.Lcd.printf("%7d", starting_delay);
      M5.Lcd.setCursor(220, 160);
      M5.Lcd.printf("%7d", interval_time);
      M5.Lcd.setCursor(220, 190);
      M5.Lcd.printf("%7d", sleep_flag);
      lcd_flag = false;
      break;
    }
  }
}

// Button Action
//------------------------------------------------------------------//
void buttonAction(void){
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if( pattern == 0 ) {
      //seq_buff = millis();
      //pattern = 11;
      lcd_pattern = 0;
      avatar_cnt = 0;      
      initLCD();
    } else if ( pattern == 1 ) {
      avatar.stop();
      delay(100);
      avatar_flag = false;
      avatar.setColorPalette(*cps[0]);
      M5.Lcd.clear();   
      pattern = 0;   
      lcd_pattern = 0;
      initLCD();
      avatar_cnt = 0;      
    } 
  } else if (M5.BtnB.wasPressed()) {
    if( pattern == 0 ) {
      lcd_pattern = 10;
      avatar_cnt = 0;
      initLCD();
    } else if ( pattern == 1 ) {
      avatar.stop();
      delay(100);
      avatar_flag = false;
      avatar.setColorPalette(*cps[0]);
      M5.Lcd.clear();  
      pattern = 0;    
      lcd_pattern = 10;
      initLCD();
      avatar_cnt = 0;      
    } 
  } else if (M5.BtnC.wasPressed()) {
    if( pattern == 0 ) {
      lcd_pattern = 20;
      avatar_cnt = 0;
      initLCD();
    } else if ( pattern == 1 ) {
      avatar.stop();
      delay(100);
      avatar_flag = false;
      avatar.setColorPalette(*cps[0]);
      M5.Lcd.clear();     
      pattern = 0; 
      lcd_pattern = 20;
      initLCD();
      avatar_cnt = 0;      
    } 
  }
  if (M5.BtnA.pressedFor(3000)) {
    seq_buff = millis();
    seq_comp = 0;
    power = 0;
    total_count1 = 0;
    total_count2 = 0;
    total_count3 = 0;
    M5.Lcd.clear();
    Serial2.printf("\n");
    tx_pattern = 101;
    M5.Lcd.setCursor(20, 20);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("UPCOMING:");  
    M5.Lcd.setCursor(20, 100);
    M5.Lcd.setTextSize(7);
    M5.Lcd.printf("LIFTOFF"); 
    delay(1000);
    M5.Lcd.loadFont(f270, SD);
    pattern = 201;
  }
}

// EEPROM Write
//------------------------------------------------------------------// 
void eeprom_write(void) {
  EEPROM.write(0, sleep_flag);
  EEPROM.write(1, (climbing_height & 0xFF));
  EEPROM.write(2, (climbing_height>>8 & 0xFF));
  EEPROM.write(3, (climbing_height>>16 & 0xFF));
  EEPROM.write(4, (climbing_height>>24 & 0xFF));
  EEPROM.write(5, climbing_velocity);
  EEPROM.write(6, climbing_accel);
  EEPROM.write(7, decending_velocity);
  EEPROM.write(8, starting_delay);
  EEPROM.write(9, interval_time);
  delay(10);
  EEPROM.commit();
  delay(10);
}

// EEPROM Read
//------------------------------------------------------------------// 
void eeprom_read(void) {
    sleep_flag = EEPROM.read(0);
    climbing_height = EEPROM.read(1) + (EEPROM.read(2)<<8) + (EEPROM.read(3)<<16) + (EEPROM.read(4)<<24);
    climbing_velocity = EEPROM.read(5);
    climbing_accel = EEPROM.read(6);
    decending_velocity = EEPROM.read(7);
    starting_delay = EEPROM.read(8);
    interval_time = EEPROM.read(9);
    delay(10);
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
int8_t getBatteryLevel() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) == 0
   && Wire.requestFrom(0x75, 1)) {
    switch (Wire.read() & 0xF0) {
    case 0xE0: return 25;
    case 0xC0: return 50;
    case 0x80: return 75;
    case 0x00: return 100;
    default: return 0;
    }
  }
  return -1;
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

uint8_t distanceFast(uint16_t * distance1)
{
    // 1. Wait for busyFlag to indicate device is idle. This must be
    //    done before triggering a range measurement.
    LidarLite1.waitForBusy();

    // 2. Trigger range measurement.
    LidarLite1.takeRange();

    // 3. Read previous distance data from device registers.
    //    After starting a measurement we can immediately read previous
    //    distance measurement while the current range acquisition is
    //    ongoing. This distance data is valid until the next
    //    measurement finishes. The I2C transaction finishes before new
    //    distance measurement data is acquired.
    *distance1 = LidarLite1.readDistance();

    return 1;
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

//Start Sequence
//------------------------------------------------------------------//
void start_sequence(void) {
  char start_pattern;  

  if( seq/1000 > seq_comp ) {
    start_pattern = starting_delay - seq/1000; 
    seq_comp = seq/1000;

    M5.Lcd.clear();
    M5.Lcd.setCursor(10, 20);
    M5.Lcd.printf("%2d", start_pattern);

    if( start_pattern <= 0 ) {
      M5.Lcd.unloadFont();
      pattern = 202;
    }
  }
}