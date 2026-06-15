#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define USE_PHYSICAL_SENSORS true

// ==========================================
// 1. 腳位宣告
// ==========================================
// 感測器 (左到右)
const int SENSOR_1 = 34; const int SENSOR_2 = 35; const int SENSOR_3 = 32; 
const int SENSOR_4 = 33; const int SENSOR_5 = 23; 

// 馬達控制 
const int MOTOR_L_PWM = 16; const int MOTOR_L_IN1 = 13; const int MOTOR_L_IN2 = 14; 
const int MOTOR_R_PWM = 27; const int MOTOR_R_IN3 = 26; const int MOTOR_R_IN4 = 25; 

// 狀態燈號與車載開關
const int ASL_GREEN = 17; const int ASL_RED = 18; const int ASL_BLUE = 19; 
const int BUTTON_VLS = 22; 

// 系統狀態
enum VehicleState { STATE_SAFE, STATE_AUTO, STATE_EBS_LOCKED };
VehicleState currentStatus = STATE_SAFE;
bool lastVlsState = true; 

// ==========================================
// 2. 驅動參數設定
// ==========================================
const int pwmFreq = 5000; const int pwmResolution = 8;
const int baseSpeed = 140; 

float Kp = 60.0;  
float Ki = 0.00;  
float Kd = 15.0;  

const float errorTable[32] = {
  0, 4, 2, 3, 0, 0, 1, 2, -2, 0, 0, 0, -1, 0, 0, 0, 
  -4, 0, 0, 0, 0, 0, 0, 0, -3, 0, 0, 0, -2, 0, 0, 0
};

// 副程式宣告
void updateASL();
void triggerEBS(String reason);
void calculateDrive(int sensorState);
int readPhysicalSensors();
void printToBoth(String msg); 

// 開關防彈跳濾波 (防雜訊暴衝)
bool readVlsDebounced() {
  static bool stableState = true;
  static int filterCount = 0;
  bool rawState = (digitalRead(BUTTON_VLS) == LOW);
  
  if (rawState != stableState) {
    filterCount++;
    if (filterCount >= 20) { stableState = rawState; filterCount = 0; }
  } else { filterCount = 0; }
  return stableState;
}

// ==========================================
// 初始設定
// ==========================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_RacingCar"); 
  
  pinMode(SENSOR_1, INPUT); pinMode(SENSOR_2, INPUT); pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT); pinMode(SENSOR_5, INPUT);
  pinMode(BUTTON_VLS, INPUT_PULLUP); 
  
  pinMode(ASL_GREEN, OUTPUT); pinMode(ASL_RED, OUTPUT); pinMode(ASL_BLUE, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT); pinMode(MOTOR_R_IN4, OUTPUT);
  
  ledcAttach(MOTOR_L_PWM, pwmFreq, pwmResolution);
  ledcAttach(MOTOR_R_PWM, pwmFreq, pwmResolution);
  
  delay(1000); // 開機電流穩定
  lastVlsState = (digitalRead(BUTTON_VLS) == LOW);
  updateASL(); 
  Serial.println("系統啟動");
}

// ==========================================
// Loop 主迴圈
// ==========================================
void loop() {
  // 1. 車體 VLS 開關偵測 
  bool currentVlsState = readVlsDebounced();
  if (currentVlsState != lastVlsState) {
    if (currentVlsState == true && currentStatus == STATE_SAFE) {
      delay(1000); currentStatus = STATE_AUTO; updateASL();
    } else if (currentVlsState == false && currentStatus == STATE_AUTO) {
      currentStatus = STATE_SAFE; updateASL();
    }
    lastVlsState = currentVlsState;
  }

  // 2. 藍牙優先遙控 
  String input = "";
  if (SerialBT.available() > 0) input = SerialBT.readStringUntil('\n'); 
  else if (Serial.available() > 0) input = Serial.readStringUntil('\n'); 

  if (input != "") {
    input.trim(); input.toUpperCase(); 
    if (input == "STOP") triggerEBS("收到 STOP 指令");
    else if (input == "RESET" && currentStatus == STATE_EBS_LOCKED) {
      currentStatus = STATE_SAFE; updateASL();
    }
    else if (input == "START" && currentStatus == STATE_SAFE) {
      delay(1000); currentStatus = STATE_AUTO; updateASL();
    }
  }

  // 3. 循跡運算執行
  if (USE_PHYSICAL_SENSORS && currentStatus == STATE_AUTO) {
    int sensorState = readPhysicalSensors();
    calculateDrive(sensorState);
    delay(1); // 採樣
  }

  //  4. 安全鎖 
  if (currentStatus != STATE_AUTO) {
    digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN3, LOW); digitalWrite(MOTOR_R_IN4, LOW);
    ledcWrite(MOTOR_L_PWM, 0);      ledcWrite(MOTOR_R_PWM, 0);
  }
}

void printToBoth(String msg) { Serial.println(msg); SerialBT.println(msg); }

int readPhysicalSensors() {
  int state = 0;
  state |= (digitalRead(SENSOR_1) << 4); state |= (digitalRead(SENSOR_2) << 3);
  state |= (digitalRead(SENSOR_3) << 2); state |= (digitalRead(SENSOR_4) << 1);
  state |= (digitalRead(SENSOR_5) << 0);
  return state;
}

void updateASL() {
  digitalWrite(ASL_GREEN, currentStatus == STATE_SAFE);
  digitalWrite(ASL_RED, currentStatus == STATE_AUTO);
  digitalWrite(ASL_BLUE, currentStatus == STATE_EBS_LOCKED);
}

void triggerEBS(String reason) { currentStatus = STATE_EBS_LOCKED; updateASL(); }

// ==========================================
// 驅動與死區核心
// ==========================================
void calculateDrive(int sensorState) {
  static float lastError = 0;
  static float integral = 0;
  float error = 0;
  int currentBaseSpeed = baseSpeed; 

  if (sensorState == 0) { 
    if (lastError > 0) error = 4.0; else if (lastError < 0) error = -4.0; else error = 0;
    currentBaseSpeed = 45; 
  } else {
    error = errorTable[sensorState];
    if (abs(error) >= 3) currentBaseSpeed = 45; 
  }
  
  integral += error; integral = constrain(integral, -50, 50); 
  float output = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));
  lastError = error; 
  
  int leftSpeed = currentBaseSpeed + output;
  int rightSpeed = currentBaseSpeed - output;
  
  if (leftSpeed >= 0) { digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW); } 
  else { digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, HIGH); leftSpeed = -leftSpeed; }

  // 右輪反轉修正 
  if (rightSpeed >= 0) { digitalWrite(MOTOR_R_IN3, LOW); digitalWrite(MOTOR_R_IN4, HIGH); } 
  else { digitalWrite(MOTOR_R_IN3, HIGH); digitalWrite(MOTOR_R_IN4, LOW); rightSpeed = -rightSpeed; }

  //  死區補償：提供突破靜摩擦力的保底電壓
  const int MIN_POWER = 80; 
  if (leftSpeed > 0 && leftSpeed < MIN_POWER) leftSpeed = MIN_POWER;
  if (rightSpeed > 0 && rightSpeed < MIN_POWER) rightSpeed = MIN_POWER;

  leftSpeed = constrain(leftSpeed, 0, 160);
  rightSpeed = constrain(rightSpeed, 0, 160);

  ledcWrite(MOTOR_L_PWM, leftSpeed);
  ledcWrite(MOTOR_R_PWM, rightSpeed);
}