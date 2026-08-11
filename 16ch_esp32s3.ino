// ==========================================================================
//  16-channel line follower - ESP32-S3, dual core
//
//  Core 1: control task - sensor scan -> PID -> speed ramp -> motors.
//          Pinned, high priority, fixed 1 kHz tick. Nothing else runs there.
//  Core 0: UI task - OLED + buttons. Suspended entirely while the robot runs.
//
//  Everything you normally want to tune is in the TUNING block below.
// ==========================================================================

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_bt.h>
#include <esp_wifi.h>

// ##########################################################################
// #                              TUNING                                    #
// #  These are the knobs. Everything here is safe to change.               #
// ##########################################################################

// ---- Progressive speed ramp -------------------------------------------
// While the line stays straight the robot climbs a speed ladder: it boosts
// for RAMP_STAGE_MS, eases back to the set speed for RAMP_SETTLE_MS, and if
// the line is STILL straight it climbs to the next stage and boosts harder.
// The instant the error starts moving the whole ladder collapses to stage 0.
//
//   set speed ->  +RAMP_STEP  ->  +2*RAMP_STEP  ->  ... up to RAMP_MAX_SPEED
//
const uint8_t RAMP_MAX_SPEED = 100;   // % hard ceiling for any ramp stage
const uint8_t RAMP_STEP = 10;         // % added per stage of the ladder
const uint16_t RAMP_STAGE_MS = 500;   // how long each boost lasts
const uint16_t RAMP_SETTLE_MS = 150;  // ease-off gap between boosts
const uint16_t RAMP_ARM_MS = 200;     // straight-line time before ramping
const uint8_t RAMP_MAX_STAGE = 5;     // stop climbing after this many stages

// What counts as "the error is not changing". Measured as the peak-to-peak
// spread of the error over the current calm spell, NOT a per-loop
// derivative - peak-to-peak means the same thing at any loop rate.
const float RAMP_ERR_ABS = 14.0f;  // |error| still considered centred
const float RAMP_ERR_BAND = 10.0f; // peak-to-peak error allowed while calm
const int RAMP_EDGE_GUARD = 2;     // outer sensors this far in kill the ramp

// ---- Switchback / hairpin ---------------------------------------------
// A fold tighter than 90 degrees cannot be taken with differential steering:
// the wheelbase simply will not arc that tightly. When the line reaches the
// edge of the bar the robot brakes, pivots in place until the centre sensors
// re-acquire, then resumes PID.
//
// HAIR_ERR is how far out the line must be to call it a hairpin. sensorPos
// runs to +/-90, so 66 means "out on the outer three sensors". Raise it if
// normal corners are pivoting, lower it if hairpins are still being missed.
const float HAIR_ERR = 66.0f;
const uint16_t HAIR_CONFIRM_MS = 12;    // must persist this long (noise reject)
const uint8_t HAIR_MAX_ON = 8;          // more sensors lit = junction, not fold
const uint16_t HAIR_BRAKE_MS = 60;      // reverse-brake burst before the pivot
const uint8_t HAIR_BRAKE_PCT = 45;      // % reverse during that burst
const uint8_t HAIR_PIVOT_PCT = 80;      // % used for the in-place pivot
const uint16_t HAIR_PIVOT_MIN_MS = 90;  // pivot this long before looking again
const uint16_t HAIR_PIVOT_MAX_MS = 1400; // give up, hand back to recovery
const uint16_t HAIR_SETTLE_MS = 40;     // straight nudge after the pivot lands

// Lit sensors this far apart still count as one line. Stops a single dead or
// marginal channel from splitting one line into two clusters.
const int CLUSTER_GAP = 1;

// ---- Control loop ------------------------------------------------------
// Fixed period, in whole FreeRTOS ticks. Kd multiplies the change in error
// between iterations, so a drifting loop rate would silently retune the
// robot. 1 ms = the default tick, which is as fast as vTaskDelayUntil can
// pace; the scan costs far less than that, so the tick is always met.
const uint32_t CONTROL_PERIOD_MS = 1;

// ---- Sensor scan -------------------------------------------------------
// Settling after switching the MUX: the RC of the MUX on-resistance plus the
// ADC sample cap and whatever cable capacitance the sensor board adds.
const uint32_t MUX_SETTLE_US = 8;

// ##########################################################################
// #                          END OF TUNING                                 #
// ##########################################################################

// ================= PINS ====================
const int s0Pin = 10, s1Pin = 11, s2Pin = 12, s3Pin = 13; // MUX S0..S3
const int analogZPin = 1; // MUX SIG/COM (ADC1). MUX EN tied to GND.

// BTN1 = UP | BTN2 = DOWN | BTN3 = SELECT/EDIT | BTN4 = BACK/STOP
#define BTN_START 16
#define BTN_MENU 15
#define BTN_SET 18
#define BTN_STOP 17

// A4950: IN1/IN2 per motor, both PWM-capable.
#define L_IN1 4
#define L_IN2 5
#define R_IN1 6
#define R_IN2 7

// OLED: SSD1306 128x64 I2C
#define I2C_SDA 8
#define I2C_SCL 9
#define OLED_ADDR 0x3C

#define FAN_PIN 14
#define SERVO_PIN 21

// ================= A4950 DRIVER ============
const int PWM_FREQ = 20000; // 20 kHz -> above audible range
const int PWM_RES = 8;      // 8-bit: 0..255, matches pct2pwm()

class A4950 {
public:
  A4950(int in1, int in2, int dir = 1) : _in1(in1), _in2(in2), _dir(dir) {}

  void begin() {
    ledcAttach(_in1, PWM_FREQ, PWM_RES); // ESP32 Arduino core 3.x
    ledcAttach(_in2, PWM_FREQ, PWM_RES);
    brake();
  }

  // speed: -255..255. Slow-decay (brake) PWM: hold one input HIGH, PWM the
  // other inverted. More linear speed response than fast decay.
  void drive(int speed) {
    speed = constrain(speed, -255, 255) * _dir;
    if (speed >= 0) {
      ledcWrite(_in1, 255);
      ledcWrite(_in2, 255 - speed);
    } else {
      ledcWrite(_in2, 255);
      ledcWrite(_in1, 255 + speed);
    }
  }

  void brake() {
    ledcWrite(_in1, 255);
    ledcWrite(_in2, 255);
  }
  void coast() {
    ledcWrite(_in1, 0);
    ledcWrite(_in2, 0);
  }

  void setDir(int d) { _dir = (d < 0) ? -1 : 1; }
  int getDir() const { return _dir; }

private:
  int _in1, _in2, _dir;
};

A4950 motorL(L_IN1, L_IN2, 1);
A4950 motorR(R_IN1, R_IN2, 1);
Servo arm;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

// ================= CONFIG ==================
struct PidProfile {
  float kp, ki, kd;
};

PidProfile pidProf[5] = {
    {2.0, 0.000, 10}, // PID 1 - smooth
    {3.0, 0.000, 18}, // PID 2
    {4.5, 0.001, 30}, // PID 3 - default
    {6.0, 0.002, 45}, // PID 4
    {8.0, 0.005, 60}  // PID 5 - aggressive
};

// Tuning values: the UI task edits them, the control task reads them. All
// are single words, so no lock is needed - worst case the control task uses
// the previous value for one 1 ms tick.
volatile uint8_t activePid = 2;  // 0..4
volatile uint8_t baseSpeed = 70; // % default follow speed
volatile uint8_t rotSpeed = 85;  // % rotation speed for turns
uint8_t driveSpd1 = 55;
uint8_t driveSpd2 = 85;
uint8_t accelStep = 10;
volatile int threshold = 2048; // raw ADC cutoff before calibration
volatile bool lineIsDark = true;
volatile int8_t motorLdir = 1;
volatile int8_t motorRdir = 1;
volatile bool rampEnabled = true; // master switch for the speed ladder
volatile bool hairEnabled = true; // master switch for the switchback pivot
uint32_t hairSinceMs = 0;         // when the hairpin pattern first appeared

// ================= PLAN ====================
enum : uint8_t {
  A_STRAIGHT = 0,
  A_LEFT,
  A_RIGHT,
  A_UTURN,
  A_STOP,
  A_FAN_ON,
  A_FAN_OFF,
  A_PICK,
  A_PLACE
};

struct PlanStep {
  uint8_t junctions; // act after this many junctions
  uint8_t action;
  uint8_t speed; // % follow speed for this step
  uint8_t pid;   // profile 0..4
  uint8_t color; // 0 = black line, 1 = white line
};

#define MAX_STEPS 12
PlanStep plan[MAX_STEPS];
uint8_t planCount = 0;

// ================= RUNTIME =================
enum Mode : uint8_t { M_IDLE = 0, M_RUN = 1 };
volatile Mode mode = M_IDLE;

#define NUM_SENSORS 16
int sensorBits[NUM_SENSORS];
uint16_t sensorMask = 0;
float error = 0, lastError = 0, integral = 0;
const float I_MAX = 200;

// ---- Per-sensor calibration ----
int sensorMin[NUM_SENSORS]; // ADC off the line (background)
int sensorMax[NUM_SENSORS]; // ADC on the line
int sensorThr[NUM_SENSORS]; // per-sensor threshold = (min+max)/2
volatile bool calibrated = false;

// Weights reflecting C-shaped geometry: outer sensors curve inward, so their
// effective lateral position is less extreme than a straight bar.
const float sensorPos[NUM_SENSORS] = {-90, -78, -66, -54, -42, -30, -18, -6,
                                      6,   18,  30,  42,  54,  66,  78,  90};

uint8_t currentStep = 0;
uint8_t junctionsSeen = 0;
bool onJunction = false;
uint32_t lastJunctionMs = 0;
const uint32_t JUNCTION_COOLDOWN = 400;
uint32_t lineLostMs = 0;

bool fanOn = false;
Preferences prefs;

// ---- Speed ramp state (control task only) ----
uint8_t rampStage = 0;      // 0 = not ramping, 1..RAMP_MAX_STAGE
bool rampBoosting = false;  // true = in a boost burst, false = settling
uint32_t rampPhaseMs = 0;   // when the current phase started
uint32_t rampCalmMs = 0;    // when the line last became calm
float rampErrMin = 0, rampErrMax = 0; // error envelope over the calm spell

// ---- Cross-core telemetry (control task writes, UI task reads) ----
volatile float tmError = 0;
volatile uint16_t tmMask = 0;
volatile uint8_t tmStep = 0;
volatile uint8_t tmJunctions = 0;
volatile bool tmRunning = false;
volatile uint8_t tmSpeed = 0;     // % actually commanded
volatile uint8_t tmRampStage = 0; // ladder stage reached
volatile uint16_t tmLoopHz = 0;   // measured control rate
volatile uint8_t tmMaxStage = 0;  // best stage of the whole run
volatile uint16_t tmMaxSpeed = 0; // best speed of the whole run
volatile uint16_t tmHairpins = 0; // switchbacks taken this run

// ---- Command mailbox: UI task -> control task ----
enum : uint8_t {
  CMD_NONE = 0,
  CMD_RUN,
  CMD_IDLE,
  CMD_CALIB,
  CMD_TEST_L,
  CMD_TEST_R,
  CMD_TEST_OFF
};
volatile uint8_t cmdBox = CMD_NONE;
volatile bool calibBusy = false; // control task is mid-calibration
volatile uint8_t calibPct = 0;   // progress for the UI to draw

TaskHandle_t hControl = NULL;
TaskHandle_t hUi = NULL;

inline void sendCmd(uint8_t c) { cmdBox = c; }

// ---- UI state ----
uint32_t lastOled = 0;
uint32_t lastDebounce = 0;
int lastUpState = HIGH, lastDownState = HIGH, lastSelState = HIGH,
    lastBackState = HIGH;

enum UiState : uint8_t {
  S_MENU = 0,
  S_PID,
  S_SPEED,
  S_SENSORS,
  S_ANALOG,
  S_MOTOR,
  S_SETTINGS,
  S_CALIB,
  S_LASTRUN
};
uint8_t uiState = S_MENU;

const char *MENU_ITEMS[] = {"Start",    "Calibrate", "PID",    "Speed",
                            "Line",     "Sensors",   "Analog", "Motors",
                            "Settings", "Save"};
const int MENU_N = 10;
int menuSel = 0;
int menuTop = 0;

int pidField = 0;
bool pidEditing = false;
int spdField = 0;
bool spdEditing = false;
#define SPD_FIELDS 4 // follow / rotate / ramp on-off / hairpin on-off

int motorField = 0;
bool testL = false, testR = false;
const uint8_t testSpeed = 50;
int analogRaw[16] = {0};

bool setEditing = false;
uint32_t menuFlash = 0;
const int debounceDelay = 40;
bool displayOff = false; // panel powered down for a run

// ================= LOW LEVEL ===============
int pct2pwm(uint8_t p) { return (int)p * 255 / 100; }

void motorStop() {
  motorL.drive(0);
  motorR.drive(0);
}

// Direct register write to the 4 MUX select lines. GPIO 10..13 all sit in the
// low bank (0..31), so one masked write flips all four simultaneously. Much
// faster than four digitalWrite() calls, and no intermediate channel is ever
// momentarily selected.
static const uint32_t MUX_MASK =
    (1UL << s0Pin) | (1UL << s1Pin) | (1UL << s2Pin) | (1UL << s3Pin);

inline void selectChannel(int ch) {
  uint32_t bits = ((ch & 1) ? (1UL << s0Pin) : 0) |
                  ((ch & 2) ? (1UL << s1Pin) : 0) |
                  ((ch & 4) ? (1UL << s2Pin) : 0) |
                  ((ch & 8) ? (1UL << s3Pin) : 0);
  REG_WRITE(GPIO_OUT_W1TC_REG, MUX_MASK & ~bits); // clear the zero bits
  REG_WRITE(GPIO_OUT_W1TS_REG, bits);             // set the one bits
}

// Median of 3. Cheap and it throws away single-sample spikes, which is the
// noise that actually causes a stray sensor bit at speed.
inline int readAdc() {
  int a = analogRead(analogZPin);
  int b = analogRead(analogZPin);
  int c = analogRead(analogZPin);
  return max(min(a, b), min(max(a, b), c));
}

// Read one channel: switch the MUX, let it settle, convert.
int readChannel(int ch) {
  selectChannel(ch);
  delayMicroseconds(MUX_SETTLE_US);
  return readAdc();
}

void readSensors(bool dark) {
  uint16_t m = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int adc = readChannel(i);
    int thr = calibrated ? sensorThr[i] : threshold;
    sensorBits[i] = dark ? (adc > thr) : (adc < thr);
    if (sensorBits[i])
      m |= (1 << i);
  }
  sensorMask = m;
}

void readSensorsRaw(int *out) {
  for (int i = 0; i < NUM_SENSORS; i++)
    out[i] = readChannel(i);
}

void readAllChannels() {
  for (int i = 0; i < 16; i++)
    analogRaw[i] = readChannel(i);
}

void saveConfig(); // forward decl - the menu calls it before it is defined

// ----------- CALIBRATION -----------
// Runs on the control task. Sweep the robot across the line by hand for 7 s;
// the UI task draws the progress bar from calibPct.
void calibrateSensors() {
  calibBusy = true;
  calibPct = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  Serial.println("CALIBRATION: sweep robot over line for 7 seconds...");
  uint32_t t0 = millis();
  const uint32_t CALIB_TIME = 7000;
  int raw[NUM_SENSORS];

  while (millis() - t0 < CALIB_TIME) {
    readSensorsRaw(raw);
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i])
        sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i])
        sensorMax[i] = raw[i];
    }
    calibPct = (millis() - t0) * 100 / CALIB_TIME;
    vTaskDelay(1); // let the UI task refresh the progress bar
  }
  calibPct = 100;

  // Per-sensor threshold = midpoint of that sensor's own min/max. A sensor is
  // trusted only if it saw a clear swing; the rest inherit the average of the
  // good ones so they don't false-trigger on noise near their flat baseline.
  const int MIN_RANGE = 150;
  bool good[NUM_SENSORS];
  int goodCount = 0;
  long sumGoodThr = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    good[i] = ((sensorMax[i] - sensorMin[i]) >= MIN_RANGE);
    if (good[i]) {
      sensorThr[i] = (sensorMin[i] + sensorMax[i]) / 2;
      sumGoodThr += sensorThr[i];
      goodCount++;
    }
  }
  int fallbackThr = goodCount ? (int)(sumGoodThr / goodCount) : 2048;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!good[i])
      sensorThr[i] = fallbackThr;
    Serial.printf("  S%02d: min=%4d max=%4d thr=%4d %s\n", i, sensorMin[i],
                  sensorMax[i], sensorThr[i], good[i] ? "OK" : "weak");
  }

  threshold = fallbackThr;
  calibrated = (goodCount >= 3);
  Serial.printf("CALIBRATION DONE: avgThreshold=%d goodSensors=%d cal=%d\n",
                threshold, goodCount, calibrated);
  saveConfig(); // a calibration you have to redo after every reboot is useless
  calibBusy = false;
}

// ================= CLUSTERED LINE POSITION ==================
// Averaging every lit sensor breaks on a switchback. Where the line folds back
// on itself both legs of the fold pass under the bar at once: sensors light on
// the left AND on the right, and their mean lands near zero. The robot reads
// "dead straight" at the exact moment it is anything but, holds the wheels
// level and drives off the line. No PID gain fixes an error signal that reads
// zero.
//
// So lit sensors are grouped into contiguous clusters (runs separated by more
// than CLUSTER_GAP dark channels) and only ONE cluster is steered on: the one
// nearest to where the line was last frame. The second leg is then ignored -
// it can never pull the error back to the middle - and the tracked leg is
// followed right out to the edge of the bar, which is what raises |error| far
// enough to trip the hairpin handler.
#define MAX_CLUSTERS 8
float clusterPos[MAX_CLUSTERS];
int clusterSize[MAX_CLUSTERS];
int clusterCount = 0;

// Where the line actually is, tracked frame to frame. This is the memory used
// to decide which of two lit clusters continues the line we were following.
float trackPos = 0;
bool trackValid = false;

static void buildClusters() {
  clusterCount = 0;
  int i = 0;
  while (i < NUM_SENSORS) {
    if (!sensorBits[i]) {
      i++;
      continue;
    }
    float sum = 0;
    int cnt = 0, gap = 0, j = i;
    while (j < NUM_SENSORS) {
      if (sensorBits[j]) {
        sum += sensorPos[j];
        cnt++;
        gap = 0;
      } else {
        gap++;
        if (gap > CLUSTER_GAP)
          break;
      }
      j++;
    }
    if (clusterCount < MAX_CLUSTERS) {
      clusterPos[clusterCount] = sum / cnt;
      clusterSize[clusterCount] = cnt;
      clusterCount++;
    }
    i = j;
  }
}

float getLineError() {
  buildClusters();

  if (clusterCount == 0) {
    trackValid = false;
    return lastError; // hold last known direction
  }
  if (clusterCount == 1) {
    trackPos = clusterPos[0];
    trackValid = true;
    return trackPos;
  }

  // Several clusters. With no history, take the widest (the real line is
  // normally fatter than a stray mark).
  int best = 0;
  if (!trackValid) {
    for (int c = 1; c < clusterCount; c++)
      if (clusterSize[c] > clusterSize[best])
        best = c;
  } else {
    float bestD = fabsf(clusterPos[0] - trackPos);
    for (int c = 1; c < clusterCount; c++) {
      float d = fabsf(clusterPos[c] - trackPos);
      // Ties go to the wider cluster.
      if (d < bestD - 0.5f ||
          (fabsf(d - bestD) <= 0.5f && clusterSize[c] > clusterSize[best])) {
        bestD = d;
        best = c;
      }
    }
  }

  trackPos = clusterPos[best];
  trackValid = true;
  return trackPos;
}

int sensorsOn() {
  int n = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if (sensorBits[i])
      n++;
  return n;
}

// C-shaped array: outer sensors curve inward/forward. Requiring a centre
// sensor as well stops a hard curve from registering as a junction.
bool centerOnLine() { return sensorBits[7] || sensorBits[8]; }
bool lineLost() { return sensorMask == 0; }
bool nodeLeft() {
  return sensorBits[0] && sensorBits[1] && sensorBits[2] && sensorBits[3] &&
         centerOnLine();
}
bool nodeRight() {
  return sensorBits[12] && sensorBits[13] && sensorBits[14] &&
         sensorBits[15] && centerOnLine();
}

// ================= SPEED RAMP ==============
void rampReset() {
  rampStage = 0;
  rampBoosting = false;
  rampPhaseMs = 0;
  rampCalmMs = 0;
}

// The progressive ladder.
//
// Straight line held for RAMP_ARM_MS  -> stage 1, boost +RAMP_STEP for
// RAMP_STAGE_MS, ease off for RAMP_SETTLE_MS. Still straight? -> stage 2,
// boost +2*RAMP_STEP. And so on up to RAMP_MAX_STAGE / RAMP_MAX_SPEED.
//
// The gate is deliberately asymmetric: climbing takes sustained calm, but
// ANY error movement collapses the ladder to stage 0 on that very tick. That
// asymmetry is what lets the robot sprint down straights and still make the
// corner at the end of one.
uint8_t rampSpeed(uint8_t setPct, float err) {
  if (!rampEnabled)
    return setPct;

  uint32_t now = millis();

  // Off-centre -> not a straight, drop everything.
  if (fabsf(err) > RAMP_ERR_ABS) {
    rampReset();
    return setPct;
  }

  // The line touching the outer sensors means something is about to happen
  // even if the centroid still looks calm. Never boost into that.
  for (int i = 0; i < RAMP_EDGE_GUARD; i++) {
    if (sensorBits[i] || sensorBits[NUM_SENSORS - 1 - i]) {
      rampReset();
      return setPct;
    }
  }

  if (rampCalmMs == 0) { // start of a possible straight
    rampCalmMs = now;
    rampErrMin = rampErrMax = err;
    return setPct;
  }

  if (err < rampErrMin)
    rampErrMin = err;
  if (err > rampErrMax)
    rampErrMax = err;

  // The error wandered too far during this spell -> the line is turning.
  if (rampErrMax - rampErrMin > RAMP_ERR_BAND) {
    rampReset();
    return setPct;
  }

  if (now - rampCalmMs < RAMP_ARM_MS)
    return setPct; // calm, but not for long enough yet

  if (rampStage == 0) { // first boost of this straight
    rampStage = 1;
    rampBoosting = true;
    rampPhaseMs = now;
  } else if (rampBoosting) {
    if (now - rampPhaseMs >= RAMP_STAGE_MS) { // boost over -> ease off
      rampBoosting = false;
      rampPhaseMs = now;
    }
  } else {
    if (now - rampPhaseMs >= RAMP_SETTLE_MS) { // settled and still straight
      if (rampStage < RAMP_MAX_STAGE)
        rampStage++; // climb one rung
      rampBoosting = true;
      rampPhaseMs = now;
    }
  }

  if (!rampBoosting)
    return setPct; // settling phase runs at the plain set speed

  int spd = (int)setPct + (int)rampStage * (int)RAMP_STEP;
  if (spd > RAMP_MAX_SPEED)
    spd = RAMP_MAX_SPEED;
  return (uint8_t)spd;
}

// ================= MOTION ==================
void followPid(uint8_t spdPct, uint8_t pidIdx) {
  error = getLineError();
  float dErr = error - lastError;

  integral = constrain(integral + error, -I_MAX, I_MAX);
  PidProfile &p = pidProf[pidIdx];
  float out = p.kp * error + p.ki * integral + p.kd * dErr;
  lastError = error;

  uint8_t usePct = rampSpeed(spdPct, error);
  int base = pct2pwm(usePct);

  int l = base + (int)out;
  int r = base - (int)out;

  // Headroom preservation. At high base speed the outer wheel saturates at
  // 255 long before the inner one bottoms out, so the differential the PID
  // asked for gets silently clipped - exactly when it matters most. Shifting
  // both wheels down preserves the full differential. This is what makes the
  // top ramp stages survivable.
  int over = max(l, r) - 255;
  if (over > 0) {
    l -= over;
    r -= over;
  }

  motorL.drive(constrain(l, -255, 255));
  motorR.drive(constrain(r, -255, 255));

  tmError = error;
  tmSpeed = usePct;
  tmRampStage = rampBoosting ? rampStage : 0;
  if (rampStage > tmMaxStage)
    tmMaxStage = rampStage;
  if (usePct > tmMaxSpeed)
    tmMaxSpeed = usePct;
}

// dir: +1 = rotate right, -1 = rotate left. Stops when the centre sensors
// re-acquire the line.
bool rotateUntilLine(int dir, bool dark, uint32_t timeoutMs) {
  int s = pct2pwm(rotSpeed);
  motorL.drive(dir > 0 ? s : -s);
  motorR.drive(dir > 0 ? -s : s);
  uint32_t t0 = millis();
  vTaskDelay(pdMS_TO_TICKS(120)); // rotate off the current line first
  while (millis() - t0 < timeoutMs) {
    readSensors(dark);
    if (centerOnLine()) {
      motorStop();
      return true;
    }
  }
  motorStop();
  return false;
}

void nudgeForward(int ms) {
  int s = pct2pwm(baseSpeed) * 2 / 3;
  motorL.drive(s);
  motorR.drive(s);
  vTaskDelay(pdMS_TO_TICKS(ms));
  motorStop();
}

// ================= ACTIONS =================
void executeAction(PlanStep &st, bool dark) {
  switch (st.action) {
  case A_STRAIGHT:
    nudgeForward(140);
    break;
  case A_LEFT:
    nudgeForward(90);
    rotateUntilLine(-1, dark, 2500);
    break;
  case A_RIGHT:
    nudgeForward(90);
    rotateUntilLine(+1, dark, 2500);
    break;
  case A_UTURN:
    rotateUntilLine(+1, dark, 3500);
    break;
  case A_STOP:
    motorStop();
    mode = M_IDLE;
    tmRunning = false;
    break;
  case A_FAN_ON:
    fanOn = true;
    digitalWrite(FAN_PIN, HIGH);
    break;
  case A_FAN_OFF:
    fanOn = false;
    digitalWrite(FAN_PIN, LOW);
    break;
  case A_PICK:
    motorStop();
    arm.write(35);
    vTaskDelay(pdMS_TO_TICKS(450));
    break;
  case A_PLACE:
    motorStop();
    arm.write(120);
    vTaskDelay(pdMS_TO_TICKS(450));
    break;
  }
  integral = 0;
  lastError = 0;
  rampReset(); // the straight is over; earn the next ramp from scratch
}

// ================= RUN LOGIC ===============
void startRun() {
  currentStep = 0;
  junctionsSeen = 0;
  onJunction = false;
  integral = 0;
  lastError = 0;
  lastJunctionMs = 0;
  lineLostMs = 0;
  rampReset();
  trackValid = false;
  hairSinceMs = 0;
  tmStep = 0;
  tmJunctions = 0;
  tmMaxStage = 0;
  tmMaxSpeed = 0;
  tmHairpins = 0;
  tmRunning = true;
  mode = M_RUN;
}

void stopAll() {
  mode = M_IDLE;
  tmRunning = false;
  rampReset();
  motorStop();
}

// ================= SWITCHBACK ==============
// A fold tighter than 90 degrees cannot be steered through: the wheelbase will
// not arc that tightly, and by the time the outer sensors see the line the
// fold is already under the chassis. The only reliable way through is to stop
// translating and rotate on the spot.
//
// isHairpin() decides. The line has to be right out at the edge of the bar and
// only a few sensors may be lit - a wide pattern is a junction or a crossing,
// and those must keep going straight, not pivot.
bool isHairpin(float err, int nOn) {
  if (!hairEnabled)
    return false;
  if (nOn == 0 || nOn > HAIR_MAX_ON)
    return false;
  if (fabsf(err) < HAIR_ERR)
    return false;
  // A real fold puts the line on one side only. Both outer groups lit means a
  // crossing, so leave it to the junction logic.
  bool leftEdge = sensorBits[0] || sensorBits[1] || sensorBits[2];
  bool rightEdge = sensorBits[NUM_SENSORS - 1] || sensorBits[NUM_SENSORS - 2] ||
                   sensorBits[NUM_SENSORS - 3];
  if (leftEdge && rightEdge)
    return false;
  return true;
}

// Brake hard, pivot in place toward the line, resume.
// dir: +1 = line is to the right (rotate right), -1 = rotate left.
void takeHairpin(int dir, bool dark) {
  tmHairpins++;

  // 1. Kill the forward momentum. A gentle ramp-down would coast us clean
  //    past the fold.
  int b = pct2pwm(HAIR_BRAKE_PCT);
  motorL.drive(-b);
  motorR.drive(-b);
  vTaskDelay(pdMS_TO_TICKS(HAIR_BRAKE_MS));
  motorStop();

  // 2. Pivot in place. Counter-rotating the wheels turns the robot about its
  //    own centre, so the sensor bar sweeps across the fold instead of driving
  //    away from it.
  int s = pct2pwm(HAIR_PIVOT_PCT);
  motorL.drive(dir > 0 ? s : -s);
  motorR.drive(dir > 0 ? -s : s);

  uint32_t t0 = millis();
  while (millis() - t0 < HAIR_PIVOT_MAX_MS) {
    readSensors(dark);
    // Ignore the first stretch: we start ON the line we came in on, so looking
    // immediately would "find" it at once and pivot nowhere.
    if (millis() - t0 >= HAIR_PIVOT_MIN_MS && centerOnLine() &&
        sensorsOn() <= HAIR_MAX_ON)
      break;
    // This task runs at priority 10 and the pivot can last over a second.
    // Without a yield the idle task never runs and the watchdog fires.
    vTaskDelay(1);
  }
  motorStop();

  // 3. Straighten up briefly so the PID restarts pointing down the new leg
  //    rather than still swinging.
  int f = pct2pwm(baseSpeed) / 2;
  motorL.drive(f);
  motorR.drive(f);
  vTaskDelay(pdMS_TO_TICKS(HAIR_SETTLE_MS));
  motorStop();

  // The old PID history describes a different line now.
  integral = 0;
  lastError = 0;
  trackValid = false;
  rampReset();
}

void runTick() {
  bool usePlan = (planCount > 0) && (currentStep < planCount);
  PlanStep st;
  if (usePlan)
    st = plan[currentStep];
  else {
    st.junctions = 255;
    st.action = A_STRAIGHT;
    st.speed = baseSpeed;
    st.pid = activePid;
    st.color = lineIsDark ? 0 : 1;
  }

  bool dark = (st.color == 0);
  readSensors(dark);
  tmMask = sensorMask;

  // ---- junction counting (cooldown prevents double-triggers) ----
  bool junc = nodeLeft() || nodeRight();
  if (junc && !onJunction && (millis() - lastJunctionMs > JUNCTION_COOLDOWN)) {
    onJunction = true;
    lastJunctionMs = millis();
    junctionsSeen++;
    tmJunctions = junctionsSeen;
    if (usePlan && junctionsSeen >= st.junctions) {
      executeAction(st, dark);
      junctionsSeen = 0;
      onJunction = false;
      currentStep++;
      tmStep = currentStep;
      tmJunctions = 0;
      return;
    }
  }
  if (!junc)
    onJunction = false;

  // ---- lost-line recovery: spin toward the last known side ----
  if (lineLost()) {
    rampReset();
    trackValid = false;
    if (lineLostMs == 0)
      lineLostMs = millis();
    if (millis() - lineLostMs > 2000) { // give up after 2 s
      stopAll();
      lineLostMs = 0;
      return;
    }
    int s = pct2pwm(rotSpeed) / 2; // half speed so we don't overshoot
    if (lastError >= 0) {
      motorL.drive(s);
      motorR.drive(-s);
    } else {
      motorL.drive(-s);
      motorR.drive(s);
    }
    return;
  }
  lineLostMs = 0;

  // ---- switchback ----
  // Confirmed over a few consecutive ticks so a single noisy scan at the edge
  // of the bar cannot trigger a pivot mid-straight.
  float err = getLineError();
  if (isHairpin(err, sensorsOn())) {
    if (hairSinceMs == 0)
      hairSinceMs = millis();
    if (millis() - hairSinceMs >= HAIR_CONFIRM_MS) {
      hairSinceMs = 0;
      takeHairpin(err > 0 ? +1 : -1, dark);
      return;
    }
  } else {
    hairSinceMs = 0;
  }

  followPid(st.speed, st.pid);
}

// ==========================================================================
//  CONTROL TASK - core 1, high priority, nothing else on this core.
//
//  Fixed 1 kHz tick via vTaskDelayUntil. The scan + PID costs far less than
//  the period; the slack is given back to the scheduler rather than burned,
//  so the tick stays exact even when the UI core is busy.
// ==========================================================================
void updateButtons(); // defined in the UI section below
void updateOled();

void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  uint32_t hzT0 = millis();
  uint16_t hzCount = 0;

  for (;;) {
    // ---- commands from the UI task ----
    uint8_t cmd = cmdBox;
    if (cmd != CMD_NONE) {
      cmdBox = CMD_NONE;
      switch (cmd) {
      case CMD_RUN:
        startRun();
        break;
      case CMD_IDLE:
        stopAll();
        testL = testR = false;
        break;
      case CMD_CALIB:
        stopAll();
        calibrateSensors();
        break;
      case CMD_TEST_L:
        testL = !testL;
        break;
      case CMD_TEST_R:
        testR = !testR;
        break;
      case CMD_TEST_OFF:
        testL = testR = false;
        motorStop();
        break;
      }
    }

    if (mode == M_RUN) {
      runTick();

      hzCount++;
      uint32_t nowMs = millis();
      if (nowMs - hzT0 >= 1000) {
        tmLoopHz = hzCount;
        hzCount = 0;
        hzT0 = nowMs;
      }
    } else if (testL || testR) {
      motorL.drive(testL ? pct2pwm(testSpeed) : 0);
      motorR.drive(testR ? pct2pwm(testSpeed) : 0);
      readSensors(lineIsDark);
      tmMask = sensorMask;
    } else {
      // Idle: keep the sensor screens live, motors off.
      motorStop();
      readSensors(lineIsDark);
      tmMask = sensorMask;
      if (uiState == S_ANALOG)
        readAllChannels();
    }

    // Pace the loop. If a tick ever overruns, xTaskDelayUntil returns false
    // without blocking - and a task at priority 10 that never blocks starves
    // core 1's idle task, which trips the task watchdog. Yield one tick in
    // that case so the robot keeps running instead of rebooting mid-course.
    if (!xTaskDelayUntil(&last, period)) {
      last = xTaskGetTickCount();
      vTaskDelay(1);
    }
  }
}

// ==========================================================================
//  UI TASK - core 0, low priority.
//
//  While the robot is running this task does essentially nothing: the panel
//  is powered down, no I2C traffic, no rendering, no menu logic. It polls
//  only the BACK button, at 20 Hz, and sleeps the rest of the time. That
//  keeps core 0 quiet so it never contends for the SPI flash cache with the
//  control task on core 1.
// ==========================================================================
void uiTask(void *) {
  for (;;) {
    if (tmRunning) {
      if (!displayOff) {
        if (oledOk) {
          oled.clearDisplay();
          oled.display();
          oled.ssd1306_command(SSD1306_DISPLAYOFF);
        }
        displayOff = true;
      }

      int b = digitalRead(BTN_STOP);
      if (b == LOW && lastBackState == HIGH)
        sendCmd(CMD_IDLE);
      lastBackState = b;

      vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz is plenty for one button
      continue;
    }

    // Just came back from a run -> wake the panel and show the summary.
    if (displayOff) {
      if (oledOk)
        oled.ssd1306_command(SSD1306_DISPLAYON);
      displayOff = false;
      uiState = S_LASTRUN;
      lastOled = 0;
      lastBackState = digitalRead(BTN_STOP); // swallow the stopping press
    }

    updateButtons();

    if (millis() - lastOled > 100) {
      lastOled = millis();
      updateOled();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==========================================================================
//  MENU
// ==========================================================================
void doMenuSelect() {
  switch (menuSel) {
  case 0:
    sendCmd(CMD_RUN);
    break;
  case 1:
    uiState = S_CALIB;
    sendCmd(CMD_CALIB);
    break;
  case 2:
    uiState = S_PID;
    pidField = 0;
    pidEditing = false;
    break;
  case 3:
    uiState = S_SPEED;
    spdField = 0;
    spdEditing = false;
    break;
  case 4:
    lineIsDark = !lineIsDark;
    saveConfig();
    menuFlash = millis();
    break;
  case 5:
    uiState = S_SENSORS;
    break;
  case 6:
    uiState = S_ANALOG;
    break;
  case 7:
    motorField = 0;
    sendCmd(CMD_TEST_OFF);
    uiState = S_MOTOR;
    break;
  case 8:
    setEditing = false;
    uiState = S_SETTINGS;
    break;
  case 9:
    saveConfig();
    menuFlash = millis();
    break;
  }
}

void uiUp() {
  switch (uiState) {
  case S_MENU:
    menuSel = (menuSel + MENU_N - 1) % MENU_N;
    break;
  case S_PID:
    if (!pidEditing)
      pidField = (pidField + 3) % 4;
    else
      switch (pidField) {
      case 0:
        activePid = (activePid + 1) % 5;
        break;
      case 1:
        pidProf[activePid].kp += 0.1f;
        break;
      case 2:
        pidProf[activePid].ki += 0.001f;
        break;
      case 3:
        pidProf[activePid].kd += 1.0f;
        break;
      }
    break;
  case S_SPEED:
    if (!spdEditing)
      spdField = (spdField + SPD_FIELDS - 1) % SPD_FIELDS;
    else if (spdField == 0)
      baseSpeed = (baseSpeed >= 95) ? 100 : baseSpeed + 5;
    else if (spdField == 1)
      rotSpeed = (rotSpeed >= 95) ? 100 : rotSpeed + 5;
    else if (spdField == 2)
      rampEnabled = true;
    else
      hairEnabled = true;
    break;
  case S_MOTOR:
    motorField = (motorField + 3) % 4;
    break;
  case S_SETTINGS:
    if (setEditing)
      threshold = (threshold >= 4070) ? 4095 : threshold + 25;
    break;
  default:
    break;
  }
}

void uiDown() {
  switch (uiState) {
  case S_MENU:
    menuSel = (menuSel + 1) % MENU_N;
    break;
  case S_PID:
    if (!pidEditing)
      pidField = (pidField + 1) % 4;
    else
      switch (pidField) {
      case 0:
        activePid = (activePid + 4) % 5;
        break;
      case 1:
        pidProf[activePid].kp = max(0.0f, pidProf[activePid].kp - 0.1f);
        break;
      case 2:
        pidProf[activePid].ki = max(0.0f, pidProf[activePid].ki - 0.001f);
        break;
      case 3:
        pidProf[activePid].kd = max(0.0f, pidProf[activePid].kd - 1.0f);
        break;
      }
    break;
  case S_SPEED:
    if (!spdEditing)
      spdField = (spdField + 1) % SPD_FIELDS;
    else if (spdField == 0)
      baseSpeed = (baseSpeed <= 5) ? 0 : baseSpeed - 5;
    else if (spdField == 1)
      rotSpeed = (rotSpeed <= 5) ? 0 : rotSpeed - 5;
    else if (spdField == 2)
      rampEnabled = false;
    else
      hairEnabled = false;
    break;
  case S_MOTOR:
    motorField = (motorField + 1) % 4;
    break;
  case S_SETTINGS:
    if (setEditing)
      threshold = (threshold <= 25) ? 0 : threshold - 25;
    break;
  default:
    break;
  }
}

void uiSelect() {
  switch (uiState) {
  case S_MENU:
    doMenuSelect();
    break;
  case S_PID:
    pidEditing = !pidEditing;
    break;
  case S_SPEED:
    spdEditing = !spdEditing;
    break;
  case S_SETTINGS:
    setEditing = !setEditing;
    break;
  case S_MOTOR:
    switch (motorField) {
    case 0:
      sendCmd(CMD_TEST_L);
      break;
    case 1:
      sendCmd(CMD_TEST_R);
      break;
    case 2:
      motorLdir = -motorLdir;
      motorL.setDir(motorLdir);
      break;
    case 3:
      motorRdir = -motorRdir;
      motorR.setDir(motorRdir);
      break;
    }
    break;
  default:
    break;
  }
}

void uiBack() {
  switch (uiState) {
  case S_PID:
    if (pidEditing)
      pidEditing = false;
    else {
      saveConfig();
      uiState = S_MENU;
    }
    break;
  case S_SPEED:
    if (spdEditing)
      spdEditing = false;
    else {
      saveConfig();
      uiState = S_MENU;
    }
    break;
  case S_SETTINGS:
    if (setEditing)
      setEditing = false;
    else {
      saveConfig();
      uiState = S_MENU;
    }
    break;
  case S_MOTOR:
    sendCmd(CMD_TEST_OFF);
    saveConfig(); // persist polarity changes
    uiState = S_MENU;
    break;
  case S_CALIB:
    if (!calibBusy)
      uiState = S_MENU;
    break;
  default:
    uiState = S_MENU;
    break;
  }
}

// ================= BUTTONS =================
void updateButtons() {
  if (millis() - lastDebounce < debounceDelay)
    return;
  int u = digitalRead(BTN_START);
  int d = digitalRead(BTN_MENU);
  int s = digitalRead(BTN_SET);
  int b = digitalRead(BTN_STOP);

  // Post-run summary: any key dismisses it.
  if (uiState == S_LASTRUN) {
    if ((u == LOW && lastUpState == HIGH) ||
        (d == LOW && lastDownState == HIGH) ||
        (s == LOW && lastSelState == HIGH) ||
        (b == LOW && lastBackState == HIGH)) {
      uiState = S_MENU;
      lastDebounce = millis();
    }
    lastUpState = u;
    lastDownState = d;
    lastSelState = s;
    lastBackState = b;
    return;
  }

  bool acted = false;
  if (u == LOW && lastUpState == HIGH) {
    uiUp();
    acted = true;
  }
  if (d == LOW && lastDownState == HIGH) {
    uiDown();
    acted = true;
  }
  if (s == LOW && lastSelState == HIGH) {
    uiSelect();
    acted = true;
  }
  if (b == LOW && lastBackState == HIGH) {
    uiBack();
    acted = true;
  }
  lastUpState = u;
  lastDownState = d;
  lastSelState = s;
  lastBackState = b;
  if (acted) {
    lastDebounce = millis();
    updateOled();
  }
}

// ================= OLED UI (128x64) ========
static int uiTextW(const char *s) { return (int)strlen(s) * 6; }

void drawHeader(const char *title, const char *info) {
  oled.fillRoundRect(0, 0, 128, 13, 3, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setCursor(5, 3);
  oled.print(title);
  if (info && info[0]) {
    oled.setCursor(123 - uiTextW(info), 3);
    oled.print(info);
  }
  oled.setTextColor(SSD1306_WHITE);
}

void drawFooter(const char *hint) {
  oled.drawFastHLine(0, 53, 128, SSD1306_WHITE);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(3, 56);
  oled.print(hint);
}

void drawScrollbar(int top, int visible, int total) {
  if (total <= visible)
    return;
  const int x = 125, y0 = 15, h = 46;
  oled.drawFastVLine(x + 1, y0, h, SSD1306_WHITE);
  int th = max(6, h * visible / total);
  int ty = y0 + (h - th) * top / (total - visible);
  oled.fillRoundRect(x, ty, 3, th, 1, SSD1306_WHITE);
}

void drawMenuScreen() {
  char info[8];
  snprintf(info, sizeof(info), "P%d", activePid + 1);
  drawHeader("MENU", info);

  const int VISIBLE = 4;
  if (menuSel < menuTop)
    menuTop = menuSel;
  if (menuSel >= menuTop + VISIBLE)
    menuTop = menuSel - VISIBLE + 1;

  char valBuf[10];
  for (int r = 0; r < VISIBLE; r++) {
    int i = menuTop + r;
    if (i >= MENU_N)
      break;
    int y = 15 + r * 12;
    bool sel = (i == menuSel);
    if (sel) {
      oled.fillRoundRect(0, y - 1, 122, 12, 2, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
    } else {
      oled.setTextColor(SSD1306_WHITE);
    }
    oled.setCursor(5, y + 1);
    oled.print(MENU_ITEMS[i]);

    valBuf[0] = 0;
    switch (i) {
    case 1:
      strcpy(valBuf, calibrated ? "OK" : "--");
      break;
    case 2:
      snprintf(valBuf, sizeof(valBuf), "P%d", activePid + 1);
      break;
    case 3:
      snprintf(valBuf, sizeof(valBuf), "%d%%", baseSpeed);
      break;
    case 4:
      strcpy(valBuf, lineIsDark ? "BLK" : "WHT");
      break;
    case 7:
      snprintf(valBuf, sizeof(valBuf), "%c/%c", motorLdir > 0 ? 'N' : 'R',
               motorRdir > 0 ? 'N' : 'R');
      break;
    case 8:
      snprintf(valBuf, sizeof(valBuf), "%d", threshold);
      break;
    }
    if (valBuf[0]) {
      oled.setCursor(118 - uiTextW(valBuf), y + 1);
      oled.print(valBuf);
    }
    oled.setTextColor(SSD1306_WHITE);
  }
  drawScrollbar(menuTop, VISIBLE, MENU_N);

  if (millis() - menuFlash < 1000) {
    oled.fillRoundRect(34, 2, 60, 11, 2, SSD1306_BLACK);
    oled.drawRoundRect(34, 2, 60, 11, 2, SSD1306_WHITE);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(48, 4);
    oled.print("SAVED");
  }
}

void drawPidScreen() {
  char info[8];
  snprintf(info, sizeof(info), "P%d", activePid + 1);
  drawHeader("PID EDIT", info);

  const char *lbl[4] = {"Profile", "Kp", "Ki", "Kd"};
  char val[4][12];
  snprintf(val[0], 12, "PID %d", activePid + 1);
  dtostrf(pidProf[activePid].kp, 0, 2, val[1]);
  dtostrf(pidProf[activePid].ki, 0, 4, val[2]);
  dtostrf(pidProf[activePid].kd, 0, 1, val[3]);

  for (int i = 0; i < 4; i++) {
    int y = 15 + i * 9;
    if (i == pidField) {
      if (pidEditing) {
        oled.fillRoundRect(0, y - 1, 128, 10, 2, SSD1306_WHITE);
        oled.setTextColor(SSD1306_BLACK);
      } else {
        oled.drawRoundRect(0, y - 1, 128, 10, 2, SSD1306_WHITE);
        oled.setTextColor(SSD1306_WHITE);
      }
    } else {
      oled.setTextColor(SSD1306_WHITE);
    }
    oled.setCursor(4, y);
    oled.print(lbl[i]);
    oled.setCursor(122 - uiTextW(val[i]), y);
    oled.print(val[i]);
    oled.setTextColor(SSD1306_WHITE);
  }
  drawFooter(pidEditing ? "SEL ok  U/D +/-" : "SEL edit  BK back");
}

void drawSpeedScreen() {
  drawHeader("SPEED", "");
  const char *lbl[SPD_FIELDS] = {"Follow", "Rotate", "Ramp", "Turn"};
  // The two speeds get a bar each (15px rows); the two toggles are compact
  // 10px rows, otherwise the fourth row falls off the bottom of the panel.
  for (int i = 0; i < SPD_FIELDS; i++) {
    int y = (i < 2) ? (14 + i * 15) : (44 + (i - 2) * 10);
    int rowH = (i < 2) ? 15 : 10;
    if (i == spdField) {
      if (spdEditing)
        oled.drawRoundRect(1, y - 3, 126, rowH + 2, 3, SSD1306_WHITE);
      else
        oled.fillRect(0, y - 2, 2, rowH, SSD1306_WHITE);
    }
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(5, y);
    oled.print(lbl[i]);

    char num[10];
    if (i == 0 || i == 1) {
      int v = (i == 0) ? baseSpeed : rotSpeed;
      snprintf(num, sizeof(num), "%d%%", v);
      oled.setCursor(120 - uiTextW(num), y);
      oled.print(num);
      int by = y + 9;
      oled.drawRoundRect(4, by, 120, 5, 2, SSD1306_WHITE);
      int w = 118 * v / 100;
      if (w > 0)
        oled.fillRect(5, by + 1, max(2, w), 3, SSD1306_WHITE);
    } else if (i == 2) {
      // Show where the ladder actually tops out at the current follow speed.
      int top = baseSpeed + RAMP_MAX_STAGE * RAMP_STEP;
      if (top > RAMP_MAX_SPEED)
        top = RAMP_MAX_SPEED;
      if (rampEnabled)
        snprintf(num, sizeof(num), "->%d%%", top);
      else
        snprintf(num, sizeof(num), "%s", "OFF");
      oled.setCursor(120 - uiTextW(num), y);
      oled.print(num);
    } else {
      snprintf(num, sizeof(num), "%s", hairEnabled ? "ON" : "OFF");
      oled.setCursor(120 - uiTextW(num), y);
      oled.print(num);
    }
  }
  drawFooter(spdEditing ? "SEL ok  U/D +/-" : "SEL edit  BK back");
}

void drawSensorScreen() {
  uint16_t m = tmMask;
  drawHeader("SENSORS", m ? "LINE" : "--");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = 1 + i * 8;
    if (m & (1 << i))
      oled.fillRoundRect(x, 16, 7, 26, 1, SSD1306_WHITE);
    else
      oled.drawRoundRect(x, 16, 7, 26, 1, SSD1306_WHITE);
  }
  oled.drawFastVLine(64, 15, 28, SSD1306_WHITE);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(4, 45);
  oled.print("err ");
  oled.print(tmError, 0);
  drawFooter("BTN4 = BACK");
}

void drawAnalogScreen() {
  drawHeader("ANALOG", "ch0-15");
  oled.setTextColor(SSD1306_WHITE);
  for (int i = 0; i < 16; i++) {
    int cx = (i % 4) * 32;
    int y = 17 + (i / 4) * 9;
    char buf[6];
    snprintf(buf, sizeof(buf), "%4d", analogRaw[i]);
    oled.setCursor(cx + 30 - uiTextW(buf), y);
    oled.print(buf);
  }
  drawFooter("row-major  BK back");
}

void drawMotorScreen() {
  drawHeader("MOTOR TEST", "");
  const char *lbl[4] = {"Left  FWD", "Right FWD", "L Polarity", "R Polarity"};
  char val[4][8];
  strcpy(val[0], testL ? "RUN" : "off");
  strcpy(val[1], testR ? "RUN" : "off");
  strcpy(val[2], motorLdir > 0 ? "NORM" : "REV");
  strcpy(val[3], motorRdir > 0 ? "NORM" : "REV");

  for (int i = 0; i < 4; i++) {
    int y = 15 + i * 9;
    if (i == motorField) {
      oled.fillRoundRect(0, y - 1, 128, 10, 2, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
    } else {
      oled.setTextColor(SSD1306_WHITE);
    }
    oled.setCursor(4, y);
    oled.print(lbl[i]);
    oled.setCursor(122 - uiTextW(val[i]), y);
    oled.print(val[i]);
    oled.setTextColor(SSD1306_WHITE);
  }
  drawFooter("SEL toggle  BK back");
}

void drawSettingsScreen() {
  drawHeader("SETTINGS", "");
  int y = 20;
  if (setEditing) {
    oled.fillRoundRect(0, y - 3, 128, 13, 2, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
  } else {
    oled.drawRoundRect(0, y - 3, 128, 13, 2, SSD1306_WHITE);
    oled.setTextColor(SSD1306_WHITE);
  }
  oled.setCursor(5, y);
  oled.print("Threshold");
  char v[8];
  snprintf(v, sizeof(v), "%d", threshold);
  oled.setCursor(120 - uiTextW(v), y);
  oled.print(v);

  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(5, 38);
  oled.print("Cal: ");
  oled.print(calibrated ? "per-sensor" : "fixed thr");

  drawFooter(setEditing ? "U/D +/-  SEL ok" : "SEL edit  BK back");
}

void drawCalibScreen() {
  drawHeader("CALIBRATE", lineIsDark ? "BLK" : "WHT");
  oled.setTextColor(SSD1306_WHITE);
  if (calibBusy) {
    oled.setCursor(6, 20);
    oled.print("Sweep robot across");
    oled.setCursor(6, 30);
    oled.print("the line...");
    oled.drawRoundRect(4, 42, 120, 9, 2, SSD1306_WHITE);
    uint8_t pct = calibPct;
    if (pct > 0)
      oled.fillRoundRect(6, 44, max(1, (int)pct * 116 / 100), 5, 1,
                         SSD1306_WHITE);
    drawFooter("keep sweeping");
  } else {
    oled.setCursor(4, 20);
    oled.print(calibrated ? "Status: OK" : "Status: WEAK");
    oled.setCursor(4, 32);
    oled.print("Thr avg: ");
    oled.print(threshold);
    drawFooter("BTN4 = BACK");
  }
}

// Shown after a run ends - the panel is off during the run itself, so this
// is where the telemetry finally gets displayed.
void drawLastRunScreen() {
  drawHeader("LAST RUN", "");
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(4, 17);
  oled.print("Loop   ");
  oled.print(tmLoopHz);
  oled.print(" Hz");

  oled.setCursor(4, 27);
  oled.print("Step ");
  oled.print(tmStep + 1);
  oled.print("  Junc ");
  oled.print(tmJunctions);

  oled.setCursor(4, 37);
  oled.print("Ramp  st");
  oled.print(tmMaxStage);
  oled.print("  max ");
  oled.print(tmMaxSpeed);
  oled.print("%");

  oled.setCursor(4, 47);
  oled.print("Hairpins  ");
  oled.print(tmHairpins);

  drawFooter("any key = menu");
}

void updateOled() {
  if (!oledOk)
    return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  switch (uiState) {
  case S_PID:
    drawPidScreen();
    break;
  case S_SPEED:
    drawSpeedScreen();
    break;
  case S_SENSORS:
    drawSensorScreen();
    break;
  case S_ANALOG:
    drawAnalogScreen();
    break;
  case S_MOTOR:
    drawMotorScreen();
    break;
  case S_SETTINGS:
    drawSettingsScreen();
    break;
  case S_CALIB:
    drawCalibScreen();
    break;
  case S_LASTRUN:
    drawLastRunScreen();
    break;
  default:
    drawMenuScreen();
    break;
  }
  oled.display();
}

// ================= NVS CONFIG ==============
void saveConfig() {
  prefs.begin("lfr", false);
  prefs.putBytes("pid", pidProf, sizeof(pidProf));
  prefs.putBytes("plan", plan, sizeof(plan));
  prefs.putUChar("planCount", planCount);
  prefs.putUChar("activePid", activePid);
  prefs.putUChar("baseSpeed", baseSpeed);
  prefs.putUChar("rotSpeed", rotSpeed);
  prefs.putUChar("driveSpd1", driveSpd1);
  prefs.putUChar("driveSpd2", driveSpd2);
  prefs.putUChar("accelStep", accelStep);
  prefs.putInt("threshold", threshold);
  prefs.putBool("lineIsDark", lineIsDark);
  prefs.putBool("ramp", rampEnabled);
  prefs.putBool("hair", hairEnabled);
  prefs.putChar("mldir", motorLdir);
  prefs.putChar("mrdir", motorRdir);
  prefs.putBytes("sMin", sensorMin, sizeof(sensorMin));
  prefs.putBytes("sMax", sensorMax, sizeof(sensorMax));
  prefs.putBytes("sThr", sensorThr, sizeof(sensorThr));
  prefs.putBool("calibrated", calibrated);
  prefs.end();
}

void loadConfig() {
  prefs.begin("lfr", true);
  if (prefs.isKey("pid"))
    prefs.getBytes("pid", pidProf, sizeof(pidProf));
  if (prefs.isKey("plan"))
    prefs.getBytes("plan", plan, sizeof(plan));
  planCount = prefs.getUChar("planCount", planCount);
  activePid = prefs.getUChar("activePid", activePid);
  baseSpeed = prefs.getUChar("baseSpeed", baseSpeed);
  rotSpeed = prefs.getUChar("rotSpeed", rotSpeed);
  driveSpd1 = prefs.getUChar("driveSpd1", driveSpd1);
  driveSpd2 = prefs.getUChar("driveSpd2", driveSpd2);
  accelStep = prefs.getUChar("accelStep", accelStep);
  threshold = prefs.getInt("threshold", threshold);
  lineIsDark = prefs.getBool("lineIsDark", lineIsDark);
  rampEnabled = prefs.getBool("ramp", rampEnabled);
  hairEnabled = prefs.getBool("hair", hairEnabled);
  motorLdir = prefs.getChar("mldir", motorLdir);
  motorRdir = prefs.getChar("mrdir", motorRdir);
  if (prefs.isKey("sMin"))
    prefs.getBytes("sMin", sensorMin, sizeof(sensorMin));
  if (prefs.isKey("sMax"))
    prefs.getBytes("sMax", sensorMax, sizeof(sensorMax));
  if (prefs.isKey("sThr"))
    prefs.getBytes("sThr", sensorThr, sizeof(sensorThr));
  calibrated = prefs.getBool("calibrated", false);
  prefs.end();
  motorL.setDir(motorLdir);
  motorR.setDir(motorRdir);
}

// ================= SETUP ===================
void setup() {
  Serial.begin(115200);

  // Radios are dead weight on a line follower: they burn power, generate
  // heat and their driver tasks steal time on core 0. Off for good.
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();

  pinMode(s0Pin, OUTPUT);
  pinMode(s1Pin, OUTPUT);
  pinMode(s2Pin, OUTPUT);
  pinMode(s3Pin, OUTPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_SET, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  analogReadResolution(12);
  // 11 dB attenuation -> full 0..3.3 V range. Without it the ADC clips around
  // 1.1 V and bright reflections all read the same, which looks exactly like
  // sensor failure under strong light.
  analogSetPinAttenuation(analogZPin, ADC_11db);

  motorL.begin();
  motorR.begin();
  motorStop();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    oled.clearDisplay();
    oled.display();
  } else {
    // Headless is survivable; trapping the CPU here is not.
    Serial.println("OLED init FAILED - running headless");
  }

  loadConfig();
  Serial.println(calibrated ? "Loaded saved calibration."
                            : "No calibration. Run Calibrate and sweep.");

  // Control task on core 1 at high priority, UI on core 0 at low priority.
  // Core 1 has no other work, so the 1 kHz tick is never late.
  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 10, &hControl, 1);
  xTaskCreatePinnedToCore(uiTask, "ui", 8192, NULL, 1, &hUi, 0);
}

// Everything lives in the two tasks. The Arduino loop task would just be a
// third thing competing for core 0, so it deletes itself.
void loop() { vTaskDelete(NULL); }
