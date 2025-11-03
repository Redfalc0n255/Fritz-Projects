#include <Arduino.h>

// ================== CONFIG ==================
#define USE_ANALOGWRITE 1   // set to 0 to use proper LEDC (20 kHz, 10-bit)

const int STBY_PIN = -1;    // tie STBY to 3.3V if not using a pin

// PWM settings (used only when USE_ANALOGWRITE==0)
const int PWM_FREQ = 20000; // 20 kHz (quiet)
const int PWM_RES  = 10;    // 10-bit -> duty 0..1023

// ================== PIN MAP (yours) ==================
// Driver A
const int A_PWMA = 27, A_AIN1 = 26, A_AIN2 = 25;   // M1 = top-left
const int A_PWMB = 33, A_BIN1 = 32, A_BIN2 = 14;   // M2 = top-right
// Driver B
const int B_PWMA = 15, B_AIN1 = 21, B_AIN2 = 5;    // M3 = back-right (invert)
const int B_PWMB = 17, B_BIN1 = 4,  B_BIN2 = 16;   // M4 = back-left

// Invert only M3
bool invertDir[5] = {false, false, false, true, false};  // index 1..4

// ===== Map motor index (1..4) to pins and PWM channel =====
void getMotorIO(int m, int &in1, int &in2, int &pwmPin, int &ch) {
  switch (m) {
    case 1: in1=A_AIN1; in2=A_AIN2; pwmPin=A_PWMA; ch=0; break;
    case 2: in1=A_BIN1; in2=A_BIN2; pwmPin=A_PWMB; ch=1; break;
    case 3: in1=B_AIN1; in2=B_AIN2; pwmPin=B_PWMA; ch=2; break;
    case 4: in1=B_BIN1; in2=B_BIN2; pwmPin=B_PWMB; ch=3; break;
    default: in1=in2=pwmPin=ch=-1; break;
  }
}

// ===== Low-level write: set duty to a given pin/channel =====
void writeDuty(int pwmPin, int ch, int duty_0_1023) {
#if USE_ANALOGWRITE
  // analogWrite on ESP32 maps to LEDC internally (default ~1kHz, 8-bit)
  int duty8 = map(constrain(duty_0_1023,0,1023), 0,1023, 0,255);
  analogWrite(pwmPin, duty8);
#else
  ledcWrite(ch, constrain(duty_0_1023,0,1023));
#endif
}

// ===== API: set one motor speed as signed percent (-100..100) =====
void setMotorPercent(int m, int pct) {
  int in1,in2,pwmPin,ch; getMotorIO(m,in1,in2,pwmPin,ch); if (pwmPin<0) return;
  pct = constrain(pct, -100, 100);
  bool inv = invertDir[m];
  int dir = (pct>=0) ? 1 : -1;
  if (inv) dir = -dir;

  if (pct == 0) {
    // brake/coast
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    writeDuty(pwmPin, ch, 0);
    return;
  }

  if (dir > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else         { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }

  int duty = map(abs(pct), 0, 100, 0, 1023);
  writeDuty(pwmPin, ch, duty);
}

void stopAll() { for (int m=1; m<=4; ++m) setMotorPercent(m, 0); }

// ===== Side helpers for differential drive =====
void setSidesPercent(int leftPct, int rightPct) {
  // LEFT side = M1 (front-left), M4 (back-left)
  setMotorPercent(1, leftPct);
  setMotorPercent(4, leftPct);
  // RIGHT side = M2 (front-right), M3 (back-right)
  setMotorPercent(2, rightPct);
  setMotorPercent(3, rightPct);
}

// ===== Setup pins and (optionally) LEDC =====
void setupPinsAndPwm() {
  int pins[] = {
    A_AIN1, A_AIN2, A_BIN1, A_BIN2,
    B_AIN1, B_AIN2, B_BIN1, B_BIN2,
    A_PWMA, A_PWMB, B_PWMA, B_PWMB
  };
  for (int i=0;i<(int)(sizeof(pins)/sizeof(pins[0]));++i) {
    pinMode(pins[i], OUTPUT); digitalWrite(pins[i], LOW);
  }
  if (STBY_PIN >= 0) { pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH); }

#if !USE_ANALOGWRITE
  // Proper LEDC setup (quiet, high-res)
  // Make sure you're compiling with ESP32 board core selected.
  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(A_PWMA, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(A_PWMB, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES); ledcAttachPin(B_PWMA, 2);
  ledcSetup(3, PWM_FREQ, PWM_RES); ledcAttachPin(B_PWMB, 3);
#endif
}

// ===== Demo sequence with differential speeds =====
void runDemo() {
  const int T = 500;       // 0.5 s run
  const int P = 400;       // 0.4 s pause
  const int FAST = 70;     // 70% speed
  const int SLOW = 30;     // 30% speed

  // Forward
  setSidesPercent(FAST, FAST); delay(T); stopAll(); delay(P);

  // Backward
  setSidesPercent(-FAST, -FAST); delay(T); stopAll(); delay(P);

  // Pivot Left (in place): left backward, right forward
  setSidesPercent(-FAST, FAST); delay(T); stopAll(); delay(P);

  // Pivot Right
  setSidesPercent(FAST, -FAST); delay(T); stopAll(); delay(P);

  // Slight Left (gentle curve): left slower, right faster
  setSidesPercent(SLOW, FAST); delay(T); stopAll(); delay(P);

  // Slight Right
  setSidesPercent(FAST, SLOW); delay(T); stopAll(); delay(P);

  // Slight Back Left (reverse gentle)
  setSidesPercent(-SLOW, -FAST); delay(T); stopAll(); delay(P);

  // Slight Back Right
  setSidesPercent(-FAST, -SLOW); delay(T); stopAll(); delay(P);
}

void setup() {
  Serial.begin(115200);
  setupPinsAndPwm();
  Serial.println(USE_ANALOGWRITE ? "PWM via analogWrite (8-bit, ~1kHz)" :
                                   "PWM via LEDC (20kHz, 10-bit)");
}

void loop() {
  runDemo();
  delay(800);
}
