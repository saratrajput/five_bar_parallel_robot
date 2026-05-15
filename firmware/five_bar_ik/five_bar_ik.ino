// Five-bar parallel robot - IK control on Arduino Nano with two SG90 servos.
//
// Serial protocol (newline-terminated, 115200 baud):
//   M x y   -> move end-effector to (x, y) in mm; reply "OK th1 th2" or "ERR OOR"
//   H       -> move to home pose; reply "OK"
//   S       -> reply "TH1 <deg> TH2 <deg>"
//   ?       -> reply with help text
// Unknown commands reply "ERR".
//
// Calibration (do this once, after assembly):
//   1) Measure L1, L2, BASE_D in mm and set them below.
//   2) With power off, set both proximal links to point along +y (straight up
//      relative to the base line connecting the two motors). This is the IK
//      reference pose theta1 = theta2 = pi/2.
//   3) Power on. The servos will hold at HOME_SERVO_DEG. Adjust THETA1_OFFSET_DEG
//      and THETA2_OFFSET_DEG until the arms physically sit at the reference pose.
//   4) Send "M <small> <Y_HOME>" with a small positive x from the host. If the
//      wrong arm moves or an arm moves the wrong direction, flip the sign of
//      THETA1_SIGN or THETA2_SIGN between +1 and -1.

#include <Servo.h>
#include <math.h>

// ---- Geometry (mm) -------------------------------------------------------
const float L1 = 33.9;        // proximal link length
const float L2 = 50.0;        // distal link length
const float BASE_D = 77.144;  // distance between the two motor axes

// ---- Servo wiring --------------------------------------------------------
const uint8_t SERVO_PIN_LEFT  = 4;   // motor at (-BASE_D/2, 0)
const uint8_t SERVO_PIN_RIGHT = 5;   // motor at (+BASE_D/2, 0)

// ---- Servo mapping -------------------------------------------------------
// Maps IK angle theta (radians, measured CCW from +x) to servo command (deg):
//   servo_deg = THETA_OFFSET_DEG + THETA_SIGN * (theta_deg - 90)
// so theta = pi/2 (arms straight up) maps to THETA_OFFSET_DEG.
const float THETA1_OFFSET_DEG = 90.0;
const float THETA2_OFFSET_DEG = 90.0;
const int   THETA1_SIGN = +1;
const int   THETA2_SIGN = +1;

const int SERVO_MIN_DEG = 5;
const int SERVO_MAX_DEG = 175;

// ---- Motion --------------------------------------------------------------
const int   STEP_DELAY_MS = 8;   // delay per 1-degree step in moveSmooth()

// --------------------------------------------------------------------------

Servo servoLeft;
Servo servoRight;

float lastServo1Deg = 90.0;
float lastServo2Deg = 90.0;
bool servosAttached = false;

void attachServos() {
  if (servosAttached) return;
  servoLeft.attach(SERVO_PIN_LEFT);
  servoRight.attach(SERVO_PIN_RIGHT);
  servoLeft.write((int)round(lastServo1Deg));
  servoRight.write((int)round(lastServo2Deg));
  servosAttached = true;
}

void releaseServos() {
  if (!servosAttached) return;
  servoLeft.detach();
  servoRight.detach();
  servosAttached = false;
}

bool solveIK(float x, float y, float &th1, float &th2) {
  const float a1x = -BASE_D / 2.0;
  const float a2x =  BASE_D / 2.0;

  float dx1 = x - a1x, dy1 = y;
  float dx2 = x - a2x, dy2 = y;
  float r1 = sqrt(dx1 * dx1 + dy1 * dy1);
  float r2 = sqrt(dx2 * dx2 + dy2 * dy2);

  float rmin = fabs(L1 - L2);
  float rmax = L1 + L2;
  if (r1 < rmin || r1 > rmax || r2 < rmin || r2 > rmax) return false;

  float c1 = (L1 * L1 + r1 * r1 - L2 * L2) / (2.0 * L1 * r1);
  float c2 = (L1 * L1 + r2 * r2 - L2 * L2) / (2.0 * L1 * r2);
  if (c1 < -1.0 || c1 > 1.0 || c2 < -1.0 || c2 > 1.0) return false;

  float phi1 = atan2(dy1, dx1);
  float phi2 = atan2(dy2, dx2);
  // Elbows-out working mode: left arm bends CCW from its heading,
  // right arm bends CW from its heading.
  th1 = phi1 + acos(c1);
  th2 = phi2 - acos(c2);
  return true;
}

float thetaToServoDeg(float theta_rad, float offset_deg, int sign) {
  float theta_deg = theta_rad * 180.0 / PI;
  return offset_deg + sign * (theta_deg - 90.0);
}

bool clampServo(float &deg) {
  if (deg < SERVO_MIN_DEG) { deg = SERVO_MIN_DEG; return false; }
  if (deg > SERVO_MAX_DEG) { deg = SERVO_MAX_DEG; return false; }
  return true;
}

void moveSmooth(float s1_target, float s2_target) {
  float d1 = s1_target - lastServo1Deg;
  float d2 = s2_target - lastServo2Deg;
  int steps = (int)ceil(max(fabs(d1), fabs(d2)));
  if (steps < 1) steps = 1;
  for (int i = 1; i <= steps; i++) {
    float t = (float)i / (float)steps;
    servoLeft.write((int)round(lastServo1Deg + d1 * t));
    servoRight.write((int)round(lastServo2Deg + d2 * t));
    delay(STEP_DELAY_MS);
  }
  lastServo1Deg = s1_target;
  lastServo2Deg = s2_target;
}

bool moveToXY(float x, float y, float &out_s1, float &out_s2) {
  float th1, th2;
  if (!solveIK(x, y, th1, th2)) return false;
  float s1 = thetaToServoDeg(th1, THETA1_OFFSET_DEG, THETA1_SIGN);
  float s2 = thetaToServoDeg(th2, THETA2_OFFSET_DEG, THETA2_SIGN);
  bool ok1 = clampServo(s1);
  bool ok2 = clampServo(s2);
  if (!ok1 || !ok2) return false;
  attachServos();
  moveSmooth(s1, s2);
  out_s1 = s1;
  out_s2 = s2;
  return true;
}

void goHome() {
  // Home = reference pose: both arms vertical (theta1 = theta2 = pi/2).
  // By the servo mapping, that is exactly the per-motor offset values.
  attachServos();
  moveSmooth(THETA1_OFFSET_DEG, THETA2_OFFSET_DEG);
}

void printHelp() {
  Serial.println(F("CMDS: M x y | H | S | R | ?"));
}

void handleLine(char *line) {
  while (*line == ' ') line++;
  if (line[0] == '\0') return;

  if (line[0] == 'M' || line[0] == 'm') {
    // AVR's default sscanf does not support %f, so parse with strtok + atof.
    char *tok1 = strtok(line + 1, " \t");
    char *tok2 = tok1 ? strtok(NULL, " \t") : NULL;
    if (!tok1 || !tok2) {
      Serial.println(F("ERR PARSE"));
      return;
    }
    float x = atof(tok1);
    float y = atof(tok2);
    float s1, s2;
    if (!moveToXY(x, y, s1, s2)) {
      Serial.println(F("ERR OOR"));
      return;
    }
    Serial.print(F("OK "));
    Serial.print(s1, 2);
    Serial.print(' ');
    Serial.println(s2, 2);
    return;
  }
  if (line[0] == 'H' || line[0] == 'h') {
    goHome();
    Serial.println(F("OK"));
    return;
  }
  if (line[0] == 'S' || line[0] == 's') {
    Serial.print(F("TH1 "));
    Serial.print(lastServo1Deg, 2);
    Serial.print(F(" TH2 "));
    Serial.println(lastServo2Deg, 2);
    return;
  }
  if (line[0] == 'R' || line[0] == 'r') {
    releaseServos();
    Serial.println(F("OK"));
    return;
  }
  if (line[0] == '?') {
    printHelp();
    return;
  }
  Serial.println(F("ERR"));
}

void setup() {
  Serial.begin(115200);
  // Do not attach servos here so a DTR reset on serial close leaves them
  // released. They lazy-attach on the first move or home command.
  lastServo1Deg = THETA1_OFFSET_DEG;
  lastServo2Deg = THETA2_OFFSET_DEG;
  Serial.println(F("READY five_bar_ik"));
}

void loop() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = '\0';
      handleLine(buf);
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
      Serial.println(F("ERR LONG"));
    }
  }
}
