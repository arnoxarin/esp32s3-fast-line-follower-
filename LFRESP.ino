/*
 * IDLineBot_Recreation.ino  (ESP32-S3)
 * Phone-free on-device menu: Start / Calibrate / PID / Speed / Line / Sensors / Save
 * Buttons: BTN1=UP  BTN2=DOWN  BTN3=SELECT/EDIT  BTN4=BACK/STOP
 * WiFi + web dashboard kept as an optional extra (not required to run).
 *
 * Board   : ESP32-S3 Dev Module
 * Libraries: ESPAsyncWebServer + AsyncTCP, ESP32Servo,
 *            Adafruit SSD1306 + Adafruit GFX
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= WIFI AP =================
const char *AP_SSID = "IDLineBot";
const char *AP_PASS = "12345678";

// ================= PINS ====================
const int s0Pin = 10, s1Pin = 11, s2Pin = 12, s3Pin = 13; // MUX S0..S3
const int analogZPin = 1;  // MUX SIG/COM (ADC1). MUX EN tied to GND.

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Buttons drive a full phone-free OLED menu:
//   BTN1 = UP | BTN2 = DOWN | BTN3 = SELECT/EDIT | BTN4 = BACK/STOP
#define BTN_START 15   // Button 1: UP  (move selection up / increase value)
#define BTN_MENU  16   // Button 2: DOWN (move selection down / decrease value)
#define BTN_SET   17   // Button 3: SELECT / toggle EDIT
#define BTN_STOP  18   // Button 4: BACK (exit screen) / STOP during a run

#define L_IN1 4    // A4950 AIN1 (left motor)
#define L_IN2 5    // A4950 AIN2 (left motor)
#define R_IN1 6    // A4950 BIN1 (right motor)
#define R_IN2 7    // A4950 BIN2 (right motor)

#define I2C_SDA   8
#define I2C_SCL   9
#define OLED_ADDR 0x3C

#define FAN_PIN   14
#define SERVO_PIN 21

// ================= A4950 DRIVER ============
const int PWM_FREQ = 20000; // 20 kHz -> silent
const int PWM_RES  = 8;     // 8-bit: 0..255

class A4950 {
public:
  A4950(int in1, int in2, int dir = 1) : _in1(in1), _in2(in2), _dir(dir) {}
  void begin() {
    ledcAttach(_in1, PWM_FREQ, PWM_RES);
    ledcAttach(_in2, PWM_FREQ, PWM_RES);
    brake();
  }
  void drive(int speed) {
    speed = constrain(speed, -255, 255) * _dir;
    if (speed >= 0) { ledcWrite(_in1, 255); ledcWrite(_in2, 255 - speed); }
    else            { ledcWrite(_in2, 255); ledcWrite(_in1, 255 + speed); }
  }
  void brake() { ledcWrite(_in1, 255); ledcWrite(_in2, 255); }
  void coast() { ledcWrite(_in1, 0);   ledcWrite(_in2, 0);   }
  void setDir(int d) { _dir = (d >= 0) ? 1 : -1; }
  int  getDir() const { return _dir; }
private:
  int _in1, _in2, _dir;
};

A4950 motorL(L_IN1, L_IN2, 1); // LEFT  (flip dir to -1 if reversed)
A4950 motorR(R_IN1, R_IN2, 1); // RIGHT

// Motor direction flags (saved to flash, editable in the MOTOR TEST tab).
int8_t dirL = 1, dirR = 1;
Servo arm;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

// ================= CONFIG ==================
struct PidProfile { float kp, ki, kd; };
PidProfile pidProf[5] = {
  { 2.0, 0.000, 10 },   // PID 1 - smooth
  { 3.0, 0.000, 18 },   // PID 2
  { 4.5, 0.02, 120 },   // PID 3 - default
  { 6.0, 0.025, 170 },   // PID 4
  { 8.0, 0.03, 250 }    // PID 5 - aggressive
};

uint8_t activePid = 5;   // 0..4
uint8_t baseSpeed = 160;  // % follow speed
uint8_t rotSpeed  = 130;  // % rotation speed
uint8_t driveSpd1 = 55;
uint8_t driveSpd2 = 85;
uint8_t accelStep = 10;
int     threshold = 45;
bool    lineIsDark = true;
bool    wifiEnabled = true;   // WiFi AP off by default; toggle in WiFi tab
bool    flipDisplay = true;    // rotate OLED 180 deg (yellow band ends up at bottom)

// ================= PLAN ====================
enum : uint8_t { A_STRAIGHT = 0, A_LEFT, A_RIGHT, A_UTURN, A_STOP,
                 A_FAN_ON, A_FAN_OFF, A_PICK, A_PLACE };
struct PlanStep { uint8_t junctions, action, speed, pid, color; };
#define MAX_STEPS 12
PlanStep plan[MAX_STEPS];
uint8_t planCount = 0;

// ================= RUNTIME =================
enum Mode : uint8_t { M_IDLE = 0, M_RUN = 1, M_RC = 2 };
volatile Mode mode = M_IDLE;

#define NUM_SENSORS 16
int sensorBits[NUM_SENSORS];
uint16_t sensorMask = 0;
float error = 0, lastError = 0, integral = 0;
const float I_MAX = 200;

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThr[NUM_SENSORS];
bool calibrated = false;

// 16-channel layout (MUX ch 0..15):
//   ch 0..3   -> 4 LEFT edge sensors (diagonal)
//   ch 4..11  -> 8 MIDDLE sensors, straight line left->right
//   ch 12..15 -> 4 RIGHT edge sensors (diagonal)
// Positions in mm-ish units, symmetric about 0 (line center).
const float sensorPos[NUM_SENSORS] = {
  -98, -86, -74, -62,                 // left diagonal edges
  -49, -35, -21,  -7,   7,  21,  35,  49,   // middle 8 straight
   62,  74,  86,  98                  // right diagonal edges
};

// Index groups for readability.
const int LEFT_EDGE[4]  = { 0, 1, 2, 3 };
const int RIGHT_EDGE[4] = { 12, 13, 14, 15 };
const int CENTER_A = 7, CENTER_B = 8;   // middle-most two sensors

uint8_t currentStep = 0;
uint8_t junctionsSeen = 0;
bool onJunction = false;
unsigned long lastJunctionTime = 0;
const unsigned long JUNCTION_COOLDOWN = 400;
unsigned long lineLostTime = 0;

int rcL = 0, rcR = 0;
bool fanOn = false;

Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
bool wifiUp = false;   // true while the AP + web server are actually running

unsigned long lastTelemetry = 0;
unsigned long lastOled = 0;
unsigned long lastDebounce = 0;

// Edge-detect state for the 4 buttons (UP/DOWN/SELECT/BACK).
int lastUpState = HIGH, lastDownState = HIGH, lastSelState = HIGH, lastBackState = HIGH;

// ---- Phone-free OLED menu ----
enum UiState : uint8_t { S_MENU = 0, S_PID, S_SPEED, S_SENSORS, S_MOTOR, S_WIFI };
uint8_t uiState = S_MENU;

const char *MENU_ITEMS[] = { "Start", "Calibrate", "PID", "Speed", "Line", "Sensors", "Motor Test", "WiFi", "Save" };
const int MENU_N = 9;
int menuSel = 0;

// ---- 16x16 monochrome icons, one per MENU_ITEMS entry (MSB-first rows) ----
// Order matches MENU_ITEMS: play, target, sliders, bolt, wave, dot-array, gear, wifi, floppy.
const unsigned char PROGMEM MENU_ICONS[9][32] = {
  { 0x00,0x00, 0x00,0x00, 0x18,0x00, 0x1C,0x00, 0x1E,0x00, 0x1F,0x00, 0x1F,0x80, 0x1F,0xC0,   // Start (play)
    0x1F,0xC0, 0x1F,0x80, 0x1F,0x00, 0x1E,0x00, 0x1C,0x00, 0x18,0x00, 0x00,0x00, 0x00,0x00 },
  { 0x01,0x80, 0x01,0x80, 0x0F,0xF0, 0x19,0x98, 0x31,0x8C, 0x61,0x86, 0x60,0x06, 0xF8,0x1F,   // Calibrate (target)
    0xF8,0x1F, 0x60,0x06, 0x61,0x86, 0x31,0x8C, 0x19,0x98, 0x0F,0xF0, 0x01,0x80, 0x01,0x80 },
  { 0x00,0x00, 0x00,0x00, 0x38,0x00, 0xFF,0xFF, 0x38,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x70,   // PID (sliders)
    0xFF,0xFF, 0x00,0x70, 0x00,0x00, 0x00,0x00, 0x07,0x00, 0xFF,0xFF, 0x07,0x00, 0x00,0x00 },
  { 0x00,0x00, 0x01,0xE0, 0x03,0xC0, 0x07,0x80, 0x0F,0x00, 0x1F,0xE0, 0x03,0xC0, 0x07,0x80,   // Speed (bolt)
    0x0F,0x00, 0x1E,0x00, 0x38,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00 },
  { 0x00,0x00, 0x00,0x00, 0x06,0x00, 0x06,0x00, 0x03,0x00, 0x01,0x80, 0x00,0xC0, 0x00,0xC0,   // Line (wave)
    0x01,0x80, 0x03,0x00, 0x06,0x00, 0x0C,0x00, 0x0C,0x00, 0x06,0x00, 0x00,0x00, 0x00,0x00 },
  { 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x66,0x66, 0x66,0x66, 0x00,0x00, 0x00,0x00,   // Sensors (dot array)
    0x00,0x00, 0x00,0x00, 0x66,0x66, 0x66,0x66, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00 },
  { 0x00,0x00, 0x03,0xC0, 0x03,0xC0, 0x33,0xCC, 0x3F,0xFC, 0x3F,0xFC, 0xFC,0x3F, 0xFC,0x3F,   // Motor Test (gear)
    0xFC,0x3F, 0xFC,0x3F, 0x3F,0xFC, 0x3F,0xFC, 0x33,0xCC, 0x03,0xC0, 0x03,0xC0, 0x00,0x00 },
  { 0x00,0x00, 0x00,0x00, 0x1F,0xF8, 0x60,0x06, 0x00,0x00, 0x07,0xE0, 0x18,0x18, 0x00,0x00,   // WiFi (arcs + dot)
    0x03,0xC0, 0x00,0x00, 0x01,0x80, 0x01,0x80, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00 },
  { 0x00,0x00, 0xFF,0xFE, 0x9C,0x02, 0x9C,0x02, 0x9C,0x02, 0x9C,0x02, 0x80,0x02, 0x80,0x02,   // Save (floppy)
    0x8F,0xE2, 0x88,0x22, 0x88,0x22, 0x88,0x22, 0x8F,0xE2, 0xFF,0xFE, 0x00,0x00, 0x00,0x00 },
};

int  pidField = 0;  bool pidEditing = false;   // 0=profile 1=Kp 2=Ki 3=Kd
int  spdField = 0;  bool spdEditing = false;   // 0=follow 1=rotate
int  motField = 0;  int  motTestDrive = 0;     // 0=Left dir 1=Right dir 2=FWD test 3=BACK test; motTestDrive: -1/0/+1

unsigned long menuFlash = 0;
const int debounceDelay = 40;

// forward decls (used before definition)
void saveConfig();
String cfgJson();
void updateOled();
void startWifi();
void stopWifi();
void applyMotorDirs();

// ================= LOW LEVEL ===============
int pct2pwm(uint8_t p) { return (int)p * 255 / 100; }
void motorStop() { motorL.drive(0); motorR.drive(0); }

void selectChannel(int ch) {
  digitalWrite(s0Pin, ch & 1);
  digitalWrite(s1Pin, ch & 2);
  digitalWrite(s2Pin, ch & 4);
  digitalWrite(s3Pin, ch & 8);
}

int readFiltered() {
  int a = analogRead(analogZPin);
  int b = analogRead(analogZPin);
  int c = analogRead(analogZPin);
  return max(min(a, b), min(max(a, b), c)); // median of 3
}

void readSensors(bool dark) {
  sensorMask = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectChannel(i);
    delayMicroseconds(10);
    int adc = readFiltered();
    if (calibrated) sensorBits[i] = dark ? (adc > sensorThr[i]) : (adc < sensorThr[i]);
    else            sensorBits[i] = dark ? (adc > threshold)    : (adc < threshold);
    if (sensorBits[i]) sensorMask |= (1 << i);
  }
}

void readSensorsRaw(int *out) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectChannel(i);
    delayMicroseconds(10);
    out[i] = readFiltered();
  }
}

// ----------- CALIBRATION: timed sweep -----------
void calibrateSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

  if (oledOk) {
    oled.clearDisplay();
    oled.setTextSize(2); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(4, 8);  oled.print("CALIBRATE");
    oled.setTextSize(1);
    oled.setCursor(4, 32); oled.print("Sweep robot over");
    oled.setCursor(4, 44); oled.print("line for 5 sec...");
    oled.display();
  }

  unsigned long startMs = millis();
  const unsigned long CALIB_TIME = 5000;
  int raw[NUM_SENSORS];

  while (millis() - startMs < CALIB_TIME) {
    readSensorsRaw(raw);
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    if (oledOk) {
      int pct = (millis() - startMs) * 100 / CALIB_TIME;
      oled.fillRect(4, 56, pct * 120 / 100, 6, SSD1306_WHITE);
      oled.display();
    }
    delay(5);
  }

  bool anyGood = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int range = sensorMax[i] - sensorMin[i];
    if (range > 100) anyGood = true;
    sensorThr[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  readSensorsRaw(raw);
  long avgThr = 0;
  for (int i = 0; i < NUM_SENSORS; i++) avgThr += sensorThr[i];
  int centerAvg = (raw[CENTER_A] + raw[CENTER_B]) / 2;
  int edgeAvg   = (raw[0] + raw[NUM_SENSORS - 1]) / 2;
  lineIsDark = (centerAvg > edgeAvg);
  threshold = avgThr / NUM_SENSORS;
  calibrated = anyGood;

  if (oledOk) {
    oled.clearDisplay();
    oled.setTextSize(1); oled.setCursor(0, 0);
    oled.print("CALIB DONE!");
    oled.setCursor(0, 12); oled.print(calibrated ? "OK - per sensor" : "WEAK - low range");
    oled.setCursor(0, 24); oled.print("Line: "); oled.print(lineIsDark ? "BLACK" : "WHITE");
    oled.setCursor(0, 36); oled.print("Thr avg: "); oled.print(threshold);
    for (int i = 0; i < NUM_SENSORS; i++) {
      int h = map(sensorThr[i], 0, 4095, 0, 12);
      oled.fillRect(i * 8, 64 - h, 7, h, SSD1306_WHITE);
    }
    oled.display();
    delay(1200);
  }
  saveConfig();            // <-- calibration values saved to flash automatically
  if (oledOk) {
    oled.clearDisplay();
    oled.setTextSize(2); oled.setCursor(6, 20);
    oled.print("SAVED!");
    oled.setTextSize(1); oled.setCursor(6, 44);
    oled.print("Calibration stored");
    oled.display();
    delay(900);
  }
}

float getLineError() {
  float sum = 0; int cnt = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if (sensorBits[i]) { sum += sensorPos[i]; cnt++; }
  if (!cnt) return lastError;
  return sum / cnt;
}

bool centerOnLine() { return sensorBits[CENTER_A] || sensorBits[CENTER_B]; }
bool lineLost()     { return sensorMask == 0; }
// A junction/node = a whole edge group lit while still centered on the line.
bool nodeLeft()  { return sensorBits[0] && sensorBits[1] && sensorBits[2] && sensorBits[3] && centerOnLine(); }
bool nodeRight() { return sensorBits[12] && sensorBits[13] && sensorBits[14] && sensorBits[15] && centerOnLine(); }

// ================= MOTION ==================
void followPid(uint8_t spdPct, uint8_t pidIdx) {
  error = getLineError();
  integral = constrain(integral + error, -I_MAX, I_MAX);
  PidProfile &p = pidProf[pidIdx];
  float out = p.kp * error + p.ki * integral + p.kd * (error - lastError);
  lastError = error;
  int base = pct2pwm(spdPct);
  motorL.drive(constrain(base + (int)out, -255, 255));
  motorR.drive(constrain(base - (int)out, -255, 255));
}

bool rotateUntilLine(int dir, bool dark, unsigned long timeoutMs) {
  int s = pct2pwm(rotSpeed);
  motorL.drive(dir > 0 ?  s : -s);
  motorR.drive(dir > 0 ? -s :  s);
  unsigned long t0 = millis();
  delay(120);
  while (millis() - t0 < timeoutMs) {
    readSensors(dark);
    if (centerOnLine()) { motorStop(); return true; }
  }
  motorStop();
  return false;
}

void nudgeForward(int ms) {
  int s = pct2pwm(baseSpeed) * 2 / 3;
  motorL.drive(s); motorR.drive(s);
  delay(ms);
  motorStop();
}

// Timed motor-test pulse used by the MOTOR TEST tab. dir = +1 fwd, -1 back.
// Shows a live banner so you can watch which way each wheel turns.
void motorTestPulse(int dir) {
  int s = pct2pwm(driveSpd1);
  if (oledOk) {
    oled.clearDisplay(); oled.setTextSize(2); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(4, 22); oled.print(dir > 0 ? "FORWARD" : "REVERSE");
    oled.display();
  }
  motorL.drive(dir > 0 ? s : -s);
  motorR.drive(dir > 0 ? s : -s);
  delay(700);
  motorStop();
}

// ================= ACTIONS =================
void executeAction(PlanStep &st, bool dark) {
  switch (st.action) {
    case A_STRAIGHT: nudgeForward(140); break;
    case A_LEFT:     nudgeForward(90); rotateUntilLine(-1, dark, 2500); break;
    case A_RIGHT:    nudgeForward(90); rotateUntilLine(+1, dark, 2500); break;
    case A_UTURN:    rotateUntilLine(+1, dark, 3500); break;
    case A_STOP:     motorStop(); mode = M_IDLE; break;
    case A_FAN_ON:   fanOn = true;  digitalWrite(FAN_PIN, HIGH); break;
    case A_FAN_OFF:  fanOn = false; digitalWrite(FAN_PIN, LOW);  break;
    case A_PICK:     motorStop(); arm.write(35);  delay(450); break;
    case A_PLACE:    motorStop(); arm.write(120); delay(450); break;
  }
  integral = 0; lastError = 0;
}

// ================= RUN LOGIC ===============
void startRun() {
  currentStep = 0; junctionsSeen = 0; onJunction = false;
  integral = 0; lastError = 0;
  lastJunctionTime = 0; lineLostTime = 0;
  mode = M_RUN;
}

void stopAll() {
  mode = M_IDLE;
  rcL = rcR = 0;
  motorStop();
}

void runTick() {
  bool usePlan = (planCount > 0) && (currentStep < planCount);
  PlanStep st;
  if (usePlan) st = plan[currentStep];
  else { st.junctions = 255; st.action = A_STRAIGHT; st.speed = baseSpeed; st.pid = activePid; st.color = lineIsDark ? 0 : 1; }

  bool dark = (st.color == 0);
  readSensors(dark);

  bool junc = nodeLeft() || nodeRight();
  if (junc && !onJunction && (millis() - lastJunctionTime > JUNCTION_COOLDOWN)) {
    onJunction = true;
    lastJunctionTime = millis();
    junctionsSeen++;
    if (usePlan && junctionsSeen >= st.junctions) {
      executeAction(st, dark);
      junctionsSeen = 0; onJunction = false; currentStep++;
      return;
    }
  }
  if (!junc) onJunction = false;

  if (lineLost()) {
    if (lineLostTime == 0) lineLostTime = millis();
    if (millis() - lineLostTime > 2000) { stopAll(); lineLostTime = 0; return; }
    int s = pct2pwm(rotSpeed) / 2;
    if (lastError >= 0) { motorL.drive(s);  motorR.drive(-s); }
    else                { motorL.drive(-s); motorR.drive(s);  }
    return;
  }
  lineLostTime = 0;

  followPid(st.speed, st.pid);
}

// ================= MENU ACTIONS ============
void doMenuSelect() {
  switch (menuSel) {
    case 0: startRun();                         break;
    case 1: calibrateSensors();                 break;
    case 2: uiState = S_PID;  pidField = 0; pidEditing = false ;saveConfig(); menuFlash = millis(); break;
    case 3: uiState = S_SPEED; spdField = 0; spdEditing = false ;saveConfig(); menuFlash = millis(); break;
    case 4: lineIsDark = !lineIsDark; saveConfig(); menuFlash = millis(); break;
    case 5: uiState = S_SENSORS;                break;
    case 6: uiState = S_MOTOR; motField = 0; motTestDrive = 0 ;saveConfig(); menuFlash = millis(); break;
    case 7: uiState = S_WIFI; saveConfig(); menuFlash = millis();                  break;
    case 8: saveConfig(); menuFlash = millis(); break;
  }
  if (wifiUp) ws.textAll(cfgJson());
}

void uiUp() {
  if (mode == M_RUN) return;
  switch (uiState) {
    case S_MENU: menuSel = (menuSel + MENU_N - 1) % MENU_N; break;
    case S_PID:
      if (!pidEditing) pidField = (pidField + 3) % 4;
      else switch (pidField) {
        case 0: activePid = (activePid + 1) % 5; break;
        case 1: pidProf[activePid].kp += 0.1f;   break;
        case 2: pidProf[activePid].ki += 0.001f; break;
        case 3: pidProf[activePid].kd += 1.0f;   break;
      }
      break;
    case S_SPEED:
      if (!spdEditing) spdField ^= 1;
      else if (spdField == 0) baseSpeed = min(100, baseSpeed + 5);
      else                    rotSpeed  = min(100, rotSpeed + 5);
      break;
    case S_MOTOR: motField = (motField + 4 - 1) % 4; break;
    case S_WIFI:  break;   // toggle handled by SELECT
    default: break;
  }
}

void uiDown() {
  if (mode == M_RUN) return;
  switch (uiState) {
    case S_MENU: menuSel = (menuSel + 1) % MENU_N; break;
    case S_PID:
      if (!pidEditing) pidField = (pidField + 1) % 4;
      else switch (pidField) {
        case 0: activePid = (activePid + 4) % 5;                                   break;
        case 1: pidProf[activePid].kp = max(0.0f, pidProf[activePid].kp - 0.1f);   break;
        case 2: pidProf[activePid].ki = max(0.0f, pidProf[activePid].ki - 0.001f); break;
        case 3: pidProf[activePid].kd = max(0.0f, pidProf[activePid].kd - 1.0f);   break;
      }
      break;
    case S_SPEED:
      if (!spdEditing) spdField ^= 1;
      else if (spdField == 0) baseSpeed = max(0, baseSpeed - 5);
      else                    rotSpeed  = max(0, rotSpeed - 5);
      break;
    case S_MOTOR: motField = (motField + 1) % 4; break;
    case S_WIFI:  break;
    default: break;
  }
}

void uiSelect() {
  if (mode == M_RUN) return;
  switch (uiState) {
    case S_MENU:  doMenuSelect();           break;
    case S_PID:   pidEditing = !pidEditing; break;
    case S_SPEED: spdEditing = !spdEditing; break;
    case S_MOTOR:
      switch (motField) {
        case 0: dirL = -dirL; applyMotorDirs(); break;   // flip LEFT direction
        case 1: dirR = -dirR; applyMotorDirs(); break;   // flip RIGHT direction
        case 2: motorTestPulse(+1); break;               // test forward
        case 3: motorTestPulse(-1); break;               // test backward
      }
      break;
    case S_WIFI:
      wifiEnabled = !wifiEnabled;
      if (wifiEnabled) startWifi(); else stopWifi();
      saveConfig();
      break;
    default: break;
  }
}

void uiBack() {
  if (mode == M_RUN) { stopAll(); uiState = S_MENU; return; }
  switch (uiState) {
    case S_PID:   if (pidEditing) pidEditing = false; else { saveConfig(); uiState = S_MENU; } break;
    case S_SPEED: if (spdEditing) spdEditing = false; else { saveConfig(); uiState = S_MENU; } break;
    case S_SENSORS: uiState = S_MENU; break;
    case S_MOTOR:  motorStop(); saveConfig(); uiState = S_MENU; break;  // save motor dirs on exit
    case S_WIFI:   saveConfig(); uiState = S_MENU; break;
    default: break;
  }
}

// ================= BUTTONS =================
void updateButtons() {
  if (millis() - lastDebounce < debounceDelay) return;
  int u = digitalRead(BTN_START);
  int d = digitalRead(BTN_MENU);
  int s = digitalRead(BTN_SET);
  int b = digitalRead(BTN_STOP);
  bool acted = false;
  if (u == LOW && lastUpState   == HIGH) { uiUp();     acted = true; }
  if (d == LOW && lastDownState == HIGH) { uiDown();   acted = true; }
  if (s == LOW && lastSelState  == HIGH) { uiSelect(); acted = true; }
  if (b == LOW && lastBackState == HIGH) { uiBack();   acted = true; }
  lastUpState = u; lastDownState = d; lastSelState = s; lastBackState = b;
  if (acted) { lastDebounce = millis(); updateOled(); }
}

// ================= OLED (128x64: yellow strip y=0-15, blue zone y=16-63) =====

// Fills the yellow strip in solid white with black text title.
// When rotated 180deg (flipDisplay=true) the yellow strip is at the BOTTOM (y=48-63).
// When not rotated, it is at the TOP (y=0-15).
// All content is placed in the 48px blue zone on the opposite side.
void drawHeader(const char* title) {
  // Bottom yellow bar (rotated 180 deg = yellow at bottom)
  oled.fillRect(0, 48, 128, 16, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setCursor(2, 52);
  oled.print(title);
  oled.setTextColor(SSD1306_WHITE);
}

// Yellow-bar header with a 16x16 icon on the left (icon drawn black on the white bar).
void drawHeaderIcon(const char* title, int iconIdx) {
  oled.fillRect(0, 48, 128, 16, SSD1306_WHITE);
  oled.drawBitmap(1, 48, MENU_ICONS[iconIdx], 16, 16, SSD1306_BLACK);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(20, 52);
  oled.print(title);
  oled.setTextColor(SSD1306_WHITE);
}

void drawRunScreen() {
  // Yellow bar at bottom: mode + PID + error
  char hdr[22];
  snprintf(hdr, sizeof(hdr), "RUN PID%d  e:%.0f", activePid + 1, error);
  drawHeader(hdr);

  // Sensor bar strip (blue zone top, y=2..19)
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = i * 8;
    if (sensorBits[i]) oled.fillRect(x, 2, 7, 16, SSD1306_WHITE);
    else               oled.drawRect(x, 2, 7, 16, SSD1306_WHITE);
  }

  oled.setCursor(0, 22);
  oled.print("Step "); oled.print(currentStep + 1);
  oled.print("  Jn "); oled.print(junctionsSeen);

  oled.setCursor(0, 33);
  oled.print(lineIsDark ? "BLACK line" : "WHITE line");

  oled.setCursor(0, 44);
  oled.print("BTN4 = STOP");
}

void drawMenuScreen() {
  // Yellow bar at BOTTOM (y=48-63): compact status line with a mini play/wifi glyph.
  oled.fillRect(0, 48, 128, 16, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setCursor(3, 50); oled.print("LINE BOT");
  // right-aligned status: PID / speed / wifi
  char st[20];
  snprintf(st, sizeof(st), "P%d %d%s", activePid + 1, baseSpeed, wifiUp ? " W" : "");
  int stw = strlen(st) * 6;
  oled.setCursor(125 - stw, 50); oled.print(st);
  // thin progress ticks showing menu position along the bottom of the yellow bar
  int fillW = (menuSel + 1) * 122 / MENU_N;
  oled.fillRect(3, 60, fillW, 2, SSD1306_BLACK);
  oled.drawRect(3, 60, 122, 2, SSD1306_BLACK);
  oled.setTextColor(SSD1306_WHITE);

  // --- Icon menu in blue zone (y=0..47): 3 rows, 16px each ---
  const int VISIBLE = 3;
  const int ROW_H   = 16;
  int scrollTop = menuSel - 1;
  if (scrollTop < 0) scrollTop = 0;
  if (scrollTop > MENU_N - VISIBLE) scrollTop = MENU_N - VISIBLE;

  for (int i = 0; i < VISIBLE; i++) {
    int idx = scrollTop + i;
    if (idx >= MENU_N) break;
    int y = i * ROW_H;
    bool sel = (idx == menuSel);
    uint16_t fg = SSD1306_WHITE;
    if (sel) {
      oled.fillRoundRect(0, y + 1, 120, ROW_H - 2, 3, SSD1306_WHITE);
      fg = SSD1306_BLACK;
    }
    // icon (16x16) on the left, vertically centered in the row
    oled.drawBitmap(2, y, MENU_ICONS[idx], 16, 16, fg);
    // label text, vertically centered (text size 1 = 8px tall)
    oled.setTextColor(fg);
    oled.setCursor(22, y + 4);
    oled.print(MENU_ITEMS[idx]);
    oled.setTextColor(SSD1306_WHITE);
  }

  // Slim scroll bar on the right edge (x=123, y=0..47)
  int trackH   = 48;
  int thumbH   = max(6, trackH * VISIBLE / MENU_N);
  int maxScroll = MENU_N - VISIBLE;
  int thumbY   = (maxScroll > 0 ? (trackH - thumbH) * scrollTop / maxScroll : 0);
  oled.drawRect(123, 0, 5, trackH, SSD1306_WHITE);
  oled.fillRoundRect(124, thumbY + 1, 3, thumbH - 2, 1, SSD1306_WHITE);

  // "SAVED!" flash (centered pill in blue zone)
  if (millis() - menuFlash < 1200) {
    oled.fillRoundRect(28, 18, 64, 13, 3, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(42, 21); oled.print("SAVED!");
    oled.setTextColor(SSD1306_WHITE);
  }
}

void drawPidScreen() {
  drawHeaderIcon("PID EDIT", 2);
  const char *lbl[4] = { "Profile", "Kp", "Ki", "Kd" };
  char val[4][12];
  snprintf(val[0], 12, "PID %d", activePid + 1);
  dtostrf(pidProf[activePid].kp, 0, 2, val[1]);
  dtostrf(pidProf[activePid].ki, 0, 4, val[2]);
  dtostrf(pidProf[activePid].kd, 0, 1, val[3]);
  for (int i = 0; i < 4; i++) {
    int y = 1 + i * 11;   // content in blue zone (y=0..47)
    if (i == pidField) {
      if (pidEditing) { oled.fillRect(0, y - 1, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
      else            { oled.drawRect(0, y - 1, 128, 10, SSD1306_WHITE); }
    }
    oled.setCursor(3, y);  oled.print(lbl[i]);
    oled.setCursor(70, y); oled.print(val[i]);
    oled.setTextColor(SSD1306_WHITE);
  }
  oled.setCursor(0, 57);
  oled.print(pidEditing ? "SEL=ok U/D=+/-" : "SEL=edit BK=save");
}

void drawSpeedScreen() {
  drawHeaderIcon("SPEED EDIT", 3);
  const char *lbl[2] = { "Follow %", "Rotate %" };
  int v[2] = { baseSpeed, rotSpeed };
  for (int i = 0; i < 2; i++) {
    int y = 2 + i * 20;   // content in blue zone (y=0..47)
    if (i == spdField) {
      if (spdEditing) { oled.fillRect(0, y - 1, 128, 14, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
      else            { oled.drawRect(0, y - 1, 128, 14, SSD1306_WHITE); }
    }
    oled.setCursor(3, y + 2);  oled.print(lbl[i]);
    oled.setCursor(84, y + 2); oled.print(v[i]);
    oled.setTextColor(SSD1306_WHITE);
  }
  oled.setCursor(0, 57);
  oled.print(spdEditing ? "SEL=ok U/D=+/-" : "SEL=edit BK=save");
}

void drawSensorScreen() {
  drawHeaderIcon("SENSORS", 5);
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = i * 8;
    if (sensorBits[i]) oled.fillRect(x, 2, 7, 22, SSD1306_WHITE);
    else               oled.drawRect(x, 2, 7, 22, SSD1306_WHITE);
  }
  oled.setCursor(0, 28);
  oled.print("Err:"); oled.print(getLineError(), 0);
  oled.print(calibrated ? "  Cal:OK" : "  NoCal");
  oled.setCursor(0, 40);
  oled.print("BTN4 = BACK");
}

void drawMotorScreen() {
  drawHeaderIcon("MOTOR TEST", 6);
  char rowL[22], rowR[22];
  snprintf(rowL, sizeof(rowL), "Left : %s", dirL > 0 ? "NORMAL" : "FLIP");
  snprintf(rowR, sizeof(rowR), "Right: %s", dirR > 0 ? "NORMAL" : "FLIP");
  const char *rows[4] = { rowL, rowR, "Test FORWARD", "Test BACKWARD" };
  for (int i = 0; i < 4; i++) {
    int y = 1 + i * 11;   // content in blue zone (y=0..47)
    if (i == motField) { oled.fillRect(0, y - 1, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
    oled.setCursor(3, y); oled.print(rows[i]);
    oled.setTextColor(SSD1306_WHITE);
  }
  oled.setCursor(0, 57);
  oled.print("SEL=do U/D=mv BK=save");
}

void drawWifiScreen() {
  drawHeaderIcon("WIFI HOTSPOT", 7);
  oled.setCursor(0, 2);  oled.print("Status : ");
  oled.print(wifiUp ? "ON" : "OFF");
  oled.setCursor(0, 13); oled.print("SSID   : "); oled.print(AP_SSID);
  oled.setCursor(0, 23); oled.print("PASS   : "); oled.print(AP_PASS);
  if (wifiUp) { oled.setCursor(0, 33); oled.print("IP     : "); oled.print(WiFi.softAPIP()); }
  oled.setCursor(0, 57);
  oled.print(wifiEnabled ? "SEL=OFF  BK=back" : "SEL=ON   BK=back");
}

void updateOled() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  if (mode == M_RUN) drawRunScreen();
  else switch (uiState) {
    case S_PID:     drawPidScreen();    break;
    case S_SPEED:   drawSpeedScreen();  break;
    case S_SENSORS: drawSensorScreen(); break;
    case S_MOTOR:   drawMotorScreen();  break;
    case S_WIFI:    drawWifiScreen();   break;
    default:        drawMenuScreen();   break;
  }
  oled.display();
}

// ================= FLASH CONFIG ============
void saveConfig() {
  prefs.begin("idlinebot", false);
  prefs.putBytes("pid", pidProf, sizeof(pidProf));
  prefs.putBytes("plan", plan, sizeof(plan));
  prefs.putUChar("pcnt", planCount);
  prefs.putUChar("apid", activePid);
  prefs.putUChar("base", baseSpeed);
  prefs.putUChar("rots", rotSpeed);
  prefs.putUChar("spd1", driveSpd1);
  prefs.putUChar("spd2", driveSpd2);
  prefs.putUChar("astep", accelStep);
  prefs.putInt("thr", threshold);
  prefs.putBool("dark", lineIsDark);
  prefs.putBytes("smin", sensorMin, sizeof(sensorMin));
  prefs.putBytes("smax", sensorMax, sizeof(sensorMax));
  prefs.putBytes("sthr", sensorThr, sizeof(sensorThr));
  prefs.putBool("cal", calibrated);
  prefs.putBool("wifi", wifiEnabled);
  prefs.putBool("flip", flipDisplay);
  prefs.putChar("dirL", (char)dirL);
  prefs.putChar("dirR", (char)dirR);
  prefs.end();
}

void loadConfig() {
  prefs.begin("idlinebot", true);
  if (prefs.isKey("pid"))  prefs.getBytes("pid", pidProf, sizeof(pidProf));
  if (prefs.isKey("plan")) prefs.getBytes("plan", plan, sizeof(plan));
  planCount = prefs.getUChar("pcnt", 0);
  activePid = prefs.getUChar("apid", 2);
  baseSpeed = prefs.getUChar("base", 70);
  rotSpeed  = prefs.getUChar("rots", 85);
  driveSpd1 = prefs.getUChar("spd1", 55);
  driveSpd2 = prefs.getUChar("spd2", 85);
  accelStep = prefs.getUChar("astep", 10);
  threshold = prefs.getInt("thr", 45);
  lineIsDark = prefs.getBool("dark", true);
  if (prefs.isKey("smin")) prefs.getBytes("smin", sensorMin, sizeof(sensorMin));
  if (prefs.isKey("smax")) prefs.getBytes("smax", sensorMax, sizeof(sensorMax));
  if (prefs.isKey("sthr")) prefs.getBytes("sthr", sensorThr, sizeof(sensorThr));
  calibrated = prefs.getBool("cal", false);
  wifiEnabled = prefs.getBool("wifi", false);
  flipDisplay = prefs.getBool("flip", true);
  dirL = (int8_t)prefs.getChar("dirL", 1);
  dirR = (int8_t)prefs.getChar("dirR", 1);
  prefs.end();
}

// ---- Motor direction + WiFi runtime control ----
void applyMotorDirs() {
  motorL.setDir(dirL);
  motorR.setDir(dirR);
}

void startWifi() {
  if (wifiUp) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.begin();
  wifiUp = true;
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());
}

void stopWifi() {
  if (!wifiUp) return;
  ws.closeAll();
  server.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiUp = false;
}

// ================= JSON ====================
String cfgJson() {
  String j = "{\"type\":\"cfg\",\"pid\":[";
  for (int i = 0; i < 5; i++) {
    j += "[" + String(pidProf[i].kp, 3) + "," + String(pidProf[i].ki, 4) + "," + String(pidProf[i].kd, 2) + "]";
    if (i < 4) j += ",";
  }
  j += "],\"apid\":" + String(activePid)
     + ",\"base\":" + String(baseSpeed)
     + ",\"rots\":" + String(rotSpeed)
     + ",\"spd1\":" + String(driveSpd1)
     + ",\"spd2\":" + String(driveSpd2)
     + ",\"step\":" + String(accelStep)
     + ",\"thr\":"  + String(threshold)
     + ",\"dark\":" + String(lineIsDark ? 1 : 0)
     + ",\"cal\":"  + String(calibrated ? 1 : 0)
     + ",\"dirL\":" + String(dirL)
     + ",\"dirR\":" + String(dirR)
     + ",\"wifi\":" + String(wifiEnabled ? 1 : 0)
     + ",\"plan\":[";
  for (int i = 0; i < planCount; i++) {
    j += "[" + String(plan[i].junctions) + "," + String(plan[i].action) + ","
       + String(plan[i].speed) + "," + String(plan[i].pid) + "," + String(plan[i].color) + "]";
    if (i < planCount - 1) j += ",";
  }
  j += "]}";
  return j;
}

String telJson() {
  return "{\"type\":\"tel\",\"s\":" + String(sensorMask)
    + ",\"m\":" + String((int)mode)
    + ",\"st\":" + String(currentStep)
    + ",\"j\":" + String(junctionsSeen)
    + ",\"e\":" + String(error, 1)
    + ",\"fan\":" + String(fanOn ? 1 : 0) + "}";
}

// ================= WEBSOCKET ===============
void handleMsg(String msg) {
  if (msg == "CMD:START") { startRun(); return; }
  if (msg == "CMD:STOP")  { stopAll();  return; }
  if (msg == "CMD:SAVE")  { saveConfig(); ws.textAll("{\"type\":\"saved\"}"); return; }
  if (msg == "CMD:CALIB") {
    calibrateSensors();
    String calMsg = "{\"type\":\"calib\",\"ok\":" + String(calibrated ? 1 : 0)
      + ",\"dark\":" + String(lineIsDark ? 1 : 0) + ",\"thr\":[";
    for (int i = 0; i < NUM_SENSORS; i++) { calMsg += String(sensorThr[i]); if (i < NUM_SENSORS - 1) calMsg += ","; }
    calMsg += "]}";
    ws.textAll(calMsg); ws.textAll(cfgJson());
    return;
  }
  if (msg == "CMD:CLEARPLAN") { planCount = 0; return; }

  int c1 = msg.indexOf(':');
  if (c1 < 0) return;
  String key = msg.substring(0, c1);
  String val = msg.substring(c1 + 1);

  if      (key == "PID")   activePid = constrain(val.toInt(), 0, 4);
  else if (key == "BASE")  baseSpeed = constrain(val.toInt(), 0, 100);
  else if (key == "ROTS")  rotSpeed  = constrain(val.toInt(), 0, 100);
  else if (key == "SPD1")  driveSpd1 = constrain(val.toInt(), 0, 100);
  else if (key == "SPD2")  driveSpd2 = constrain(val.toInt(), 0, 100);
  else if (key == "STEP")  accelStep = constrain(val.toInt(), 0, 100);
  else if (key == "THR")   threshold = val.toInt();
  else if (key == "COLOR") lineIsDark = (val.toInt() == 0);
  else if (key == "DIRL")  { dirL = (val.toInt() >= 0) ? 1 : -1; applyMotorDirs(); saveConfig(); }
  else if (key == "DIRR")  { dirR = (val.toInt() >= 0) ? 1 : -1; applyMotorDirs(); saveConfig(); }
  else if (key == "MTEST") { motorTestPulse(val == "B" ? -1 : +1); }
  else if (key == "GAIN") {
    int idx; float kp, ki, kd;
    if (sscanf(val.c_str(), "%d,%f,%f,%f", &idx, &kp, &ki, &kd) == 4 && idx >= 0 && idx < 5)
      pidProf[idx] = { kp, ki, kd };
  }
  else if (key == "PLAN") {
    int j, a, s, p, c;
    if (sscanf(val.c_str(), "%d,%d,%d,%d,%d", &j, &a, &s, &p, &c) == 5 && planCount < MAX_STEPS)
      plan[planCount++] = { (uint8_t)j, (uint8_t)a, (uint8_t)s, (uint8_t)p, (uint8_t)c };
  }
  else if (key == "RC") {
    if (val != "EXIT") mode = M_RC;
    int s1 = pct2pwm(driveSpd1);
    if      (val == "F")    { rcL =  s1; rcR =  s1; }
    else if (val == "B")    { rcL = -s1; rcR = -s1; }
    else if (val == "L")    { rcL = -s1; rcR =  s1; }
    else if (val == "R")    { rcL =  s1; rcR = -s1; }
    else if (val == "F2")   { rcL = rcR = pct2pwm(driveSpd2); }
    else if (val == "S")    { rcL = 0; rcR = 0; }
    else if (val == "FAN")  { fanOn = !fanOn; digitalWrite(FAN_PIN, fanOn); }
    else if (val == "PICK") { arm.write(35); }
    else if (val == "PLACE"){ arm.write(120); }
    else if (val == "EXIT") { stopAll(); }
  }
}

void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) client->text(cfgJson());
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String msg; msg.reserve(len);
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      handleMsg(msg);
    }
  }
}

// ================= DASHBOARD (HTML) ========
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IDLineBot</title>
<style>
:root{--bg:#0e1116;--card:#161b22;--acc:#2f81f7;--txt:#e6edf3;--mut:#8b949e;--ok:#3fb950;--bad:#f85149}
*{box-sizing:border-box;font-family:system-ui,Arial}
body{margin:auto;background:var(--bg);color:var(--txt);padding:10px;max-width:640px}
h1{font-size:18px;display:flex;justify-content:space-between;align-items:center}
.card{background:var(--card);border:1px solid #21262d;border-radius:10px;padding:12px;margin:10px 0}
.card h2{font-size:13px;color:var(--mut);margin:0 0 8px;text-transform:uppercase;letter-spacing:1px}
.sensors{display:flex;gap:3px;justify-content:center}
.dot{width:5.4%;aspect-ratio:1;border-radius:3px;background:#21262d}
.dot.on{background:var(--acc)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.btn{flex:1;padding:12px 8px;border:0;border-radius:8px;background:#21262d;color:var(--txt);font-weight:700;font-size:14px}
.b-start{background:var(--ok);color:#04120a}
.b-stop{background:var(--bad);color:#170505}
.chip{padding:8px 12px;border-radius:20px;background:#21262d;border:1px solid #30363d;font-size:13px}
.chip.sel{background:var(--acc);border-color:var(--acc);color:#04121f;font-weight:700}
.sl{width:100%}
label{font-size:12px;color:var(--mut)}
input[type=number]{width:70px;background:#0e1116;border:1px solid #30363d;color:var(--txt);border-radius:6px;padding:6px}
select{background:#0e1116;border:1px solid #30363d;color:var(--txt);border-radius:6px;padding:6px}
table{width:100%;border-collapse:collapse;font-size:12px}
td,th{padding:4px;text-align:center}
.stat{font-size:12px;color:var(--mut)}
#conn{width:10px;height:10px;border-radius:50%;background:var(--bad);display:inline-block}
#conn.ok{background:var(--ok)}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
</style></head><body>
<h1>IDLineBot <span class="stat">re-creation <span id="conn"></span></span></h1>
<div class="card"><h2>Sensors</h2><div class="sensors" id="sens"></div><div class="stat" id="stat">mode: IDLE</div></div>
<div class="card"><h2>Run</h2><div class="row">
<button class="btn b-start" onclick="send('CMD:START')">START</button>
<button class="btn b-stop" onclick="send('CMD:STOP')">STOP</button>
<button class="btn" id="calibBtn" onclick="doCalib()">CALIB</button></div>
<div class="stat" id="calStat" style="margin-top:6px"></div>
<div class="row" style="margin-top:8px">
<span class="chip" id="c0" onclick="setColor(0)">BLACK LINE</span>
<span class="chip" id="c1" onclick="setColor(1)">WHITE LINE</span></div></div>
<div class="card"><h2>PID Profile</h2><div class="row" id="pids"></div>
<div class="row" style="margin-top:8px">
<label>Kp <input type="number" id="kp" step="0.1"></label>
<label>Ki <input type="number" id="ki" step="0.001"></label>
<label>Kd <input type="number" id="kd" step="1"></label>
<button class="btn" style="flex:0" onclick="sendGain()">SET</button></div></div>
<div class="card"><h2>Speeds</h2><div id="sliders"></div></div>
<div class="card"><h2>Motor Test</h2><div class="row">
<button class="btn" onclick="send('MTEST:F')">TEST FWD</button>
<button class="btn" onclick="send('MTEST:B')">TEST BACK</button></div>
<div class="row" style="margin-top:8px">
<span class="chip" id="dl" onclick="flipDir('DIRL')">LEFT: --</span>
<span class="chip" id="dr" onclick="flipDir('DIRR')">RIGHT: --</span></div></div>
<div class="card"><div class="row">
<button class="btn" onclick="send('CMD:SAVE')">SAVE CONFIG TO FLASH</button></div></div>
<script>
const MODES=["IDLE","RUN","RC"];
let ws,cfg=null;
const $=id=>document.getElementById(id);
const NS=16;
for(let i=0;i<NS;i++){let d=document.createElement('div');d.className='dot';d.id='d'+i;$('sens').appendChild(d);}
for(let i=0;i<5;i++){let s=document.createElement('span');s.className='chip';s.id='p'+i;s.textContent='PID '+(i+1);
  s.onclick=()=>{send('PID:'+i);if(cfg){cfg.apid=i;refresh();}};$('pids').appendChild(s);}
const SL=[["BASE","Follow speed","base"],["ROTS","Rotation speed","rots"]];
SL.forEach(([cmd,lbl,key])=>{let w=document.createElement('div');
  w.innerHTML=`<label>${lbl} <span id="v_${key}"></span>%</label><input class="sl" type="range" min="0" max="100" id="s_${key}" oninput="upS('${cmd}','${key}',this.value)">`;
  $('sliders').appendChild(w);});
function upS(cmd,key,v){$('v_'+key).textContent=v;send(cmd+':'+v);}
function setColor(c){send('COLOR:'+c);if(cfg){cfg.dark=(c==0)?1:0;refresh();}}
function sendGain(){if(!cfg)return;send(`GAIN:${cfg.apid},${$('kp').value},${$('ki').value},${$('kd').value}`);}
function doCalib(){send('CMD:CALIB');$('calStat').textContent='Calibrating... sweep 3 sec';}
function refresh(){if(!cfg)return;
  for(let i=0;i<5;i++)$('p'+i).classList.toggle('sel',i==cfg.apid);
  $('c0').classList.toggle('sel',cfg.dark==1);$('c1').classList.toggle('sel',cfg.dark==0);
  let g=cfg.pid[cfg.apid];$('kp').value=g[0];$('ki').value=g[1];$('kd').value=g[2];
  $('dl').textContent='LEFT: '+(cfg.dirL>0?'NORMAL':'FLIP');
  $('dr').textContent='RIGHT: '+(cfg.dirR>0?'NORMAL':'FLIP');
  SL.forEach(([cmd,lbl,key])=>{$('s_'+key).value=cfg[key];$('v_'+key).textContent=cfg[key];});}
function initWS(){ws=new WebSocket(`ws://${location.hostname}/ws`);
  ws.onopen=()=>$('conn').classList.add('ok');
  ws.onclose=()=>{$('conn').classList.remove('ok');setTimeout(initWS,2000);};
  ws.onmessage=e=>{let d=JSON.parse(e.data);
    if(d.type=='cfg'){cfg=d;refresh();$('calStat').textContent=d.cal?'Calibrated':'Not calibrated';}
    else if(d.type=='tel'){for(let i=0;i<NS;i++)$('d'+i).classList.toggle('on',(d.s>>i)&1);
      $('stat').textContent=`mode: ${MODES[d.m]} | error: ${d.e}`;}
    else if(d.type=='saved')alert('Config saved');};}
function send(m){if(ws&&ws.readyState==1)ws.send(m);}
function flipDir(cmd){if(!cfg)return;let cur=(cmd=='DIRL')?cfg.dirL:cfg.dirR;send(cmd+':'+(cur>0?-1:1));}
initWS();
</script></body></html>
)rawliteral";

// ================= SETUP / LOOP ============
void setup() {
  Serial.begin(115200);
  pinMode(s0Pin, OUTPUT); pinMode(s1Pin, OUTPUT);
  pinMode(s2Pin, OUTPUT); pinMode(s3Pin, OUTPUT);
  pinMode(analogZPin, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_SET, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT); digitalWrite(FAN_PIN, LOW);
  arm.attach(SERVO_PIN); arm.write(90);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(13, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    oled.setRotation(flipDisplay ? 2 : 0);   // 180-degree flip (req 5)
    oled.clearDisplay();
    oled.display();
  }

  motorL.begin();
  motorR.begin();

  loadConfig();
  oled.setRotation(flipDisplay ? 2 : 0);   // re-apply after config load
  applyMotorDirs();                        // apply saved motor directions (req 3)
  motorStop();

  // Register HTTP/WebSocket handlers always; only bring the AP up if enabled.
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  if (wifiEnabled) startWifi();   // WiFi hotspot toggle (req 4)

  updateOled();  // show the menu right away
}

void loop() {
  updateButtons();

  if (mode == M_RUN) runTick();
  else if (mode == M_RC) { readSensors(lineIsDark); motorL.drive(rcL); motorR.drive(rcR); }
  else { readSensors(lineIsDark); motorStop(); }

  if (wifiUp && millis() - lastTelemetry > 100) { lastTelemetry = millis(); ws.textAll(telJson()); ws.cleanupClients(); }

  unsigned long oledIv = (mode == M_RUN) ? 500 : 200;
  if (millis() - lastOled > oledIv) { lastOled = millis(); updateOled(); }
}