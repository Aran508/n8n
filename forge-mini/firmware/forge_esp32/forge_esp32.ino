// Forge Mini motion controller (ESP32 + 2x PCA9685).
//
// The Raspberry Pi plans everything; this board only moves servos smoothly,
// switches the pump/conveyor, reads sensors and enforces safety. It speaks
// newline-delimited JSON over USB serial at 115200 baud:
//
//   {"cmd":"move","arm":1,"servo":[90,120,80,60,20],"ms":400}
//   {"cmd":"aux","ch":"pump","on":true}        ch: pump | conveyor
//   {"cmd":"status"}  {"cmd":"ping"}  {"cmd":"reset"}  {"cmd":"relax"}
//
// Replies:  {"ok":true}  /  {"ok":false,"err":"..."}
// Events:   {"event":"ir","state":1}  {"event":"estop"}  {"event":"watchdog"}
//
// Libraries: Adafruit PWM Servo Driver, ArduinoJson (v7).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// ---------------- pins ----------------
const int PIN_ESTOP = 27;      // normally-closed button to GND (also cuts servo power in hardware)
const int PIN_IR = 26;         // IR break-beam at the end of the conveyor, LOW = part present
const int PIN_PUMP = 25;       // MOSFET gate for the vacuum pump
const int PIN_CONV_A = 32;     // DRV8833 IN1/IN2 for the conveyor N20 motor
const int PIN_CONV_B = 33;
const int CONVEYOR_DUTY = 180; // 0..255

// ---------------- servos ----------------
Adafruit_PWMServoDriver board0(0x40);  // arms 1-3 (channels 0-14)
Adafruit_PWMServoDriver board1(0x41);  // arm 4 (channels 0-4), spares

const int NUM_ARMS = 4;
const int JOINTS = 5;
struct ArmMap { Adafruit_PWMServoDriver *board; uint8_t firstChannel; };
ArmMap ARM_MAP[NUM_ARMS] = { {&board0, 0}, {&board0, 5}, {&board0, 10}, {&board1, 0} };

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const uint32_t TICK_MS = 10;          // 100 Hz interpolation
const uint32_t WATCHDOG_MS = 1500;    // freeze if the Pi goes quiet

struct Motion {
  float from[JOINTS];
  float to[JOINTS];
  float now[JOINTS];
  uint32_t start;
  uint32_t duration;
  bool active;
};
Motion arms[NUM_ARMS];

bool estopLatched = false;
bool relaxed = true;     // PWM off until the first move (no power-on jump)
uint32_t lastHostMsg = 0;
int lastIr = -1;

// Park pose in servo degrees; replace with your calibrated values.
const float PARK[JOINTS] = {90, 115, 131, 17, 20};

void writeServo(int arm, int joint, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f;
  uint16_t ticks = (uint16_t)(us * 4096.0f / 20000.0f);  // 50 Hz frame = 20 ms
  ARM_MAP[arm].board->setPWM(ARM_MAP[arm].firstChannel + joint, 0, ticks);
}

void relaxAll() {
  for (int a = 0; a < NUM_ARMS; a++) {
    for (int j = 0; j < JOINTS; j++) ARM_MAP[a].board->setPWM(ARM_MAP[a].firstChannel + j, 0, 4096);
    arms[a].active = false;
  }
  relaxed = true;
}

void freezeAll() {
  for (int a = 0; a < NUM_ARMS; a++) {
    for (int j = 0; j < JOINTS; j++) arms[a].to[j] = arms[a].now[j];
    arms[a].active = false;
  }
}

void setConveyor(bool on) {
  analogWrite(PIN_CONV_A, on ? CONVEYOR_DUTY : 0);
  analogWrite(PIN_CONV_B, 0);
}

void stopAux() {
  digitalWrite(PIN_PUMP, LOW);
  setConveyor(false);
}

void reply(bool ok, const char *err = nullptr) {
  JsonDocument doc;
  doc["ok"] = ok;
  if (err) doc["err"] = err;
  serializeJson(doc, Serial);
  Serial.println();
}

void emit(const char *name, int state = -1) {
  JsonDocument doc;
  doc["event"] = name;
  if (state >= 0) doc["state"] = state;
  serializeJson(doc, Serial);
  Serial.println();
}

void handleMove(JsonDocument &doc) {
  if (estopLatched) return reply(false, "estop");
  int arm = doc["arm"] | 0;
  JsonArray servo = doc["servo"];
  if (arm < 1 || arm > NUM_ARMS || servo.size() != JOINTS) return reply(false, "bad move");
  Motion &m = arms[arm - 1];
  for (int j = 0; j < JOINTS; j++) {
    m.from[j] = m.now[j];  // blend from wherever the arm is right now
    m.to[j] = servo[j].as<float>();
  }
  m.start = millis();
  m.duration = max<uint32_t>(doc["ms"] | 300, TICK_MS);
  m.active = true;
  relaxed = false;
  reply(true);
}

void handleAux(JsonDocument &doc) {
  if (estopLatched) return reply(false, "estop");
  const char *ch = doc["ch"] | "";
  bool on = doc["on"] | false;
  if (strcmp(ch, "pump") == 0) digitalWrite(PIN_PUMP, on ? HIGH : LOW);
  else if (strcmp(ch, "conveyor") == 0) setConveyor(on);
  else return reply(false, "bad channel");
  reply(true);
}

void handleStatus() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["estop"] = estopLatched;
  doc["ir"] = digitalRead(PIN_IR) == LOW ? 1 : 0;
  doc["relaxed"] = relaxed;
  JsonArray list = doc["arms"].to<JsonArray>();
  for (int a = 0; a < NUM_ARMS; a++) {
    JsonArray row = list.add<JsonArray>();
    for (int j = 0; j < JOINTS; j++) row.add(round(arms[a].now[j] * 10) / 10.0);
    doc["moving"][a] = arms[a].active;
  }
  serializeJson(doc, Serial);
  Serial.println();
}

void handleLine(const String &line) {
  JsonDocument doc;
  if (deserializeJson(doc, line)) return reply(false, "bad json");
  lastHostMsg = millis();
  const char *cmd = doc["cmd"] | "";
  if (strcmp(cmd, "move") == 0) handleMove(doc);
  else if (strcmp(cmd, "aux") == 0) handleAux(doc);
  else if (strcmp(cmd, "status") == 0) handleStatus();
  else if (strcmp(cmd, "ping") == 0) reply(true);
  else if (strcmp(cmd, "relax") == 0) { relaxAll(); reply(true); }
  else if (strcmp(cmd, "reset") == 0) {
    if (digitalRead(PIN_ESTOP) == HIGH) return reply(false, "estop still pressed");
    estopLatched = false;
    reply(true);
  } else reply(false, "unknown cmd");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_IR, INPUT_PULLUP);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_CONV_A, OUTPUT);
  pinMode(PIN_CONV_B, OUTPUT);
  stopAux();

  Wire.begin();
  board0.begin();
  board1.begin();
  board0.setPWMFreq(50);
  board1.setPWMFreq(50);

  for (int a = 0; a < NUM_ARMS; a++)
    for (int j = 0; j < JOINTS; j++) arms[a].now[j] = arms[a].to[j] = PARK[j];
  relaxAll();
  emit("ready");
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleLine(buf); buf = ""; }
    else if (c != '\r' && buf.length() < 512) buf += c;
  }

  // E-stop: button opens the circuit -> pin pulled HIGH
  if (!estopLatched && digitalRead(PIN_ESTOP) == HIGH) {
    estopLatched = true;
    freezeAll();
    stopAux();
    relaxAll();
    emit("estop");
  }

  // Watchdog: anything moving with a silent host gets frozen
  bool anyActive = false;
  for (int a = 0; a < NUM_ARMS; a++) anyActive |= arms[a].active;
  if (anyActive && millis() - lastHostMsg > WATCHDOG_MS) {
    freezeAll();
    stopAux();
    emit("watchdog");
  }

  int ir = digitalRead(PIN_IR) == LOW ? 1 : 0;
  if (ir != lastIr) { lastIr = ir; emit("ir", ir); }

  static uint32_t lastTick = 0;
  if (millis() - lastTick < TICK_MS) return;
  lastTick = millis();
  if (relaxed) return;

  for (int a = 0; a < NUM_ARMS; a++) {
    Motion &m = arms[a];
    if (m.active) {
      float f = (float)(millis() - m.start) / m.duration;
      if (f >= 1.0f) { f = 1.0f; m.active = false; }
      // Linear between waypoints: the planner already splits paths into
      // short straight segments, so the arm flows from one to the next.
      for (int j = 0; j < JOINTS; j++) m.now[j] = m.from[j] + (m.to[j] - m.from[j]) * f;
    }
    for (int j = 0; j < JOINTS; j++) writeServo(a, j, m.now[j]);
  }
}
