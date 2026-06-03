#include <IRremote.hpp>

// ---- Pins -------------------------------------------------------------
const int IR_PIN = 11;

const int A1_IN1 = 7;
const int A1_IN2 = 8;

const int A2_IN3 = 9;
const int A2_IN4 = 10;

// ---- IR button codes --------------------------------------------------
const uint8_t BTN_UP   = 0x09;
const uint8_t BTN_DOWN = 0x07;
const uint8_t BTN_STOP = 0x1C;

// ---- Tuning -----------------------------------------------------------
// How long each UP/DOWN command runs before auto-stopping.
// Increase to move more per press, decrease for less.
const int PULSE_MS = 200;

// ---- State ------------------------------------------------------------
bool eStopped = false;
String rxBuf  = "";

// ---- Setup ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(A1_IN1, OUTPUT);
  pinMode(A1_IN2, OUTPUT);
  pinMode(A2_IN3, OUTPUT);
  pinMode(A2_IN4, OUTPUT);

  stopBoth();

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("READY");
}

// ---- Loop -------------------------------------------------------------
void loop() {
  // ---- IR input -------------------------------------------------------
  if (IrReceiver.decode()) {
    uint8_t cmd = IrReceiver.decodedIRData.command;
    if (cmd == BTN_UP   && !eStopped) nudgeUp();
    else if (cmd == BTN_DOWN && !eStopped) nudgeDown();
    else if (cmd == BTN_STOP) stopBoth();
    IrReceiver.resume();
  }

  // ---- Serial input ---------------------------------------------------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      rxBuf.trim();
      if (rxBuf.length() > 0) handleSerial(rxBuf[0]);
      rxBuf = "";
    } else {
      rxBuf += c;
    }
  }
}

// ---- Serial command handler -------------------------------------------
void handleSerial(char cmd) {
  switch (cmd) {
    case 'U': case 'u':
      if (!eStopped) nudgeUp();
      break;
    case 'D': case 'd':
      if (!eStopped) nudgeDown();
      break;
    case 'S': case 's':
      stopBoth();
      break;
    case 'E': case 'e':
      eStopped = true;
      stopBoth();
      break;
    case 'R': case 'r':
      eStopped = false;
      stopBoth();
      break;
    default:
      break;
  }
}

// ---- Motion helpers ---------------------------------------------------
void nudgeUp() {
  digitalWrite(A1_IN1, HIGH);
  digitalWrite(A1_IN2, LOW);
  digitalWrite(A2_IN3, HIGH);
  digitalWrite(A2_IN4, LOW);
  delay(PULSE_MS);
  stopBoth();
}

void nudgeDown() {
  digitalWrite(A1_IN1, LOW);
  digitalWrite(A1_IN2, HIGH);
  digitalWrite(A2_IN3, LOW);
  digitalWrite(A2_IN4, HIGH);
  delay(PULSE_MS);
  stopBoth();
}

void stopBoth() {
  digitalWrite(A1_IN1, LOW);
  digitalWrite(A1_IN2, LOW);
  digitalWrite(A2_IN3, LOW);
  digitalWrite(A2_IN4, LOW);
}
