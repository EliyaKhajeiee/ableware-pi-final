// Ableware — Dual Linear Actuator Driver
// Protocol: single-char commands over Serial at 115200 baud, newline-terminated
//   U  → nudge both actuators UP   (extend)
//   D  → nudge both actuators DOWN (retract)
//   S  → stop immediately
//   E  → emergency stop (same as S, sets e-stop flag)
//   R  → resume (clear e-stop flag)
//
// Adjust pin numbers and PULSE_MS to match your hardware.

// ---- Pin assignments (L298N / IBT-2 style H-bridge) -------------------
// Left actuator
const int L_INA = 4;   // forward signal
const int L_INB = 5;   // reverse signal
const int L_EN  = 6;   // PWM enable (connect to ENA; use 255 if jumpered high)

// Right actuator
const int R_INA = 7;
const int R_INB = 8;
const int R_EN  = 9;   // PWM enable (connect to ENB)

// ---- Tuning -----------------------------------------------------------
const int  PULSE_MS  = 150;   // ms to run motors per UP/DOWN command — "barely moves"
const byte SPEED     = 180;   // PWM duty 0-255; lower = slower

// ---- State ------------------------------------------------------------
bool eStopped = false;
String rxBuf  = "";

// ---- Setup ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(L_INA, OUTPUT);
  pinMode(L_INB, OUTPUT);
  pinMode(L_EN,  OUTPUT);
  pinMode(R_INA, OUTPUT);
  pinMode(R_INB, OUTPUT);
  pinMode(R_EN,  OUTPUT);

  stopAll();
  Serial.println("READY");
}

// ---- Loop -------------------------------------------------------------
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      rxBuf.trim();
      if (rxBuf.length() > 0) {
        handleCommand(rxBuf[0]);
      }
      rxBuf = "";
    } else {
      rxBuf += c;
    }
  }
}

// ---- Command handler --------------------------------------------------
void handleCommand(char cmd) {
  switch (cmd) {
    case 'U':
    case 'u':
      if (!eStopped) nudge(true);
      break;
    case 'D':
    case 'd':
      if (!eStopped) nudge(false);
      break;
    case 'S':
    case 's':
      stopAll();
      break;
    case 'E':
    case 'e':
      eStopped = true;
      stopAll();
      break;
    case 'R':
    case 'r':
      eStopped = false;
      stopAll();
      break;
    default:
      break;
  }
}

// ---- Motor helpers ----------------------------------------------------
void nudge(bool extend) {
  // Drive both actuators in the same direction for PULSE_MS, then stop.
  if (extend) {
    // Extend (UP)
    analogWrite(L_EN, SPEED);
    digitalWrite(L_INA, HIGH);
    digitalWrite(L_INB, LOW);

    analogWrite(R_EN, SPEED);
    digitalWrite(R_INA, HIGH);
    digitalWrite(R_INB, LOW);
  } else {
    // Retract (DOWN)
    analogWrite(L_EN, SPEED);
    digitalWrite(L_INA, LOW);
    digitalWrite(L_INB, HIGH);

    analogWrite(R_EN, SPEED);
    digitalWrite(R_INA, LOW);
    digitalWrite(R_INB, HIGH);
  }

  delay(PULSE_MS);
  stopAll();
}

void stopAll() {
  digitalWrite(L_INA, LOW);
  digitalWrite(L_INB, LOW);
  analogWrite(L_EN, 0);

  digitalWrite(R_INA, LOW);
  digitalWrite(R_INB, LOW);
  analogWrite(R_EN, 0);
}
