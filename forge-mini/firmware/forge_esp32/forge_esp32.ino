// Forge Mini motion controller (ESP32 + 2x PCA9685), tool-changer edition.
//
// The Raspberry Pi plans everything; this board moves servos smoothly, drives
// each arm's tool-signal line, identifies the mounted tools and enforces
// safety. Newline-delimited JSON over USB serial at 115200 baud:
//
//   {"cmd":"move","arm":1,"servo":[90,120,80,60,105],"ms":400}
//        servo = base, shoulder, elbow, wrist, coupler latch
//   {"cmd":"tool","arm":4,"value":90}      tool-signal channel (servo-style PWM)
//   {"cmd":"tower","state":"run"}          run | wait | idle | fault (tower light)
//   {"cmd":"fan","on":true}  {"cmd":"light","on":true}
//   {"cmd":"aux","name":"flip","deg":180,"ms":1600}   flip fixture trunnion servo
//   {"cmd":"aux","name":"dock","deg":160,"ms":800}    USB-C programming dock slide
//   {"cmd":"status"}  {"cmd":"ping"}  {"cmd":"reset"}  {"cmd":"relax"}
//
// Replies:  {"ok":true}  /  {"ok":false,"err":"..."}
// Events:   {"event":"probe","arm":3}  {"event":"estop"}  {"event":"watchdog"}
//           {"event":"interlock","source":"door"|"curtain"}  {"event":"start"}  {"event":"stop"}
//
// Enclosure (the mini version of an industrial robot cell):
//   door      reed switch on the front acrylic door; opening it while anything
//             moves or spins latches an interlock stop
//   curtain   IR break-beam pairs across the loading window; a hand in the
//             window while running latches the same stop
//   tower     red / yellow / green LED stack: red = e-stop or interlock,
//             yellow = waiting for the operator, green = running (blinks idle)
//   buttons   START and STOP on the control panel (E-STOP is separate, wired
//             in series with the servo supply)
//
// Tool coupler (per arm): 4 pogo pins = 6 V, GND, SIG, ID.
//   SIG: servo PWM from the PCA9685. A gripper uses it directly; the spindle,
//        vacuum pump and glue plunger have a tiny brushed ESC that reads it.
//   ID:  each tool has a resistor from ID to GND. A 10k pull-up to 3.3 V on
//        the ESP32 side turns it into a voltage, so we know which tool is on.
//        The probe's resistor is in series with a normally-closed switch, so
//        a touch reads as "open" for a moment -> probe event.
//
// Libraries: Adafruit PWM Servo Driver, ArduinoJson (v7).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

const int PIN_ESTOP = 27;  // normally-closed button to GND (also cuts servo power in hardware)
const int PIN_DOOR = 13;   // reed switch to GND: LOW = door closed (unplugged reads open = safe)
const int PIN_CURTAIN = 14;  // IR receivers in series to GND: LOW = all beams clear
const int PIN_START = 16;  // panel buttons to GND
const int PIN_STOP = 17;
const int PIN_RED = 18, PIN_YELLOW = 19, PIN_GREEN = 23;  // tower light via MOSFETs
const int PIN_FAN = 4;     // fume extractor fan (carbon filter) via MOSFET
const int PIN_LIGHT = 15;  // overhead LED bar via MOSFET
const int ID_PINS[4] = {32, 33, 34, 35};  // ADC1 pins, one per arm coupler
const float ID_PULLUP_OHMS = 10000.0f;
const float PROBE_OHMS = 4700.0f;

Adafruit_PWMServoDriver board0(0x40);  // arms 1-2, channels 0-11
Adafruit_PWMServoDriver board1(0x41);  // arms 3-4, channels 0-11

// Auxiliary servos on spare PCA9685 channels of board0
struct Aux { const char *name; uint8_t channel; float now, from, to; uint32_t start, ms; bool active; };
Aux AUX[] = { {"flip", 12, 0, 0, 0, 0, 1, false}, {"dock", 13, 0, 0, 0, 0, 1, false} };
const int NUM_AUX = 2;

const int NUM_ARMS = 4;
const int JOINTS = 5;  // base, shoulder, elbow, wrist, latch
struct ArmMap { Adafruit_PWMServoDriver *board; uint8_t firstChannel; };
ArmMap ARM_MAP[NUM_ARMS] = { {&board0, 0}, {&board0, 6}, {&board1, 0}, {&board1, 6} };
// channel firstChannel + 5 is the tool signal

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const uint32_t TICK_MS = 10;        // 100 Hz interpolation
const uint32_t WATCHDOG_MS = 1500;  // freeze if the Pi goes quiet

struct Motion {
  float from[JOINTS];
  float to[JOINTS];
  float now[JOINTS];
  uint32_t start;
  uint32_t duration;
  bool active;
};
Motion arms[NUM_ARMS];
float toolSignal[NUM_ARMS] = {0, 0, 0, 0};

bool estopLatched = false;
bool interlockLatched = false;
const char *towerState = "idle";
bool relaxed = true;  // PWM off until the first move (no power-on jump)
uint32_t lastHostMsg = 0;

// Park pose in servo degrees with the latch open; replace with your calibrated values.
const float PARK[JOINTS] = {90, 115, 131, 17, 20};

uint16_t degToTicks(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f;
  return (uint16_t)(us * 4096.0f / 20000.0f);  // 50 Hz frame = 20 ms
}

void writeChannel(int arm, int ch, float deg) {
  ARM_MAP[arm].board->setPWM(ARM_MAP[arm].firstChannel + ch, 0, degToTicks(deg));
}

void offChannel(int arm, int ch) {
  ARM_MAP[arm].board->setPWM(ARM_MAP[arm].firstChannel + ch, 0, 4096);
}

void toolsOff() {
  for (int a = 0; a < NUM_ARMS; a++) {
    toolSignal[a] = 0;
    offChannel(a, 5);  // spindles, pumps and plungers stop with no signal
  }
}

void relaxAll() {
  for (int a = 0; a < NUM_ARMS; a++) {
    for (int j = 0; j < JOINTS; j++) offChannel(a, j);
    arms[a].active = false;
  }
  toolsOff();
  relaxed = true;
}

void freezeAll() {
  for (int a = 0; a < NUM_ARMS; a++) {
    for (int j = 0; j < JOINTS; j++) arms[a].to[j] = arms[a].now[j];
    arms[a].active = false;
  }
}

// Resistance of the tool's ID resistor, or -1 when nothing is connected.
float toolOhms(int arm) {
  int raw = 0;
  for (int i = 0; i < 4; i++) raw += analogRead(ID_PINS[arm]);
  raw /= 4;
  if (raw > 3950) return -1;
  return ID_PULLUP_OHMS * raw / (4095.0f - raw);
}

void reply(bool ok, const char *err = nullptr) {
  JsonDocument doc;
  doc["ok"] = ok;
  if (err) doc["err"] = err;
  serializeJson(doc, Serial);
  Serial.println();
}

void emit(const char *name, int arm = 0) {
  JsonDocument doc;
  doc["event"] = name;
  if (arm) doc["arm"] = arm;
  serializeJson(doc, Serial);
  Serial.println();
}

bool doorOpen() { return digitalRead(PIN_DOOR) == HIGH; }
bool curtainBroken() { return digitalRead(PIN_CURTAIN) == HIGH; }

void setTower() {
  bool blink = (millis() / 500) % 2;
  bool fault = estopLatched || interlockLatched || strcmp(towerState, "fault") == 0;
  digitalWrite(PIN_RED, fault);
  digitalWrite(PIN_YELLOW, !fault && strcmp(towerState, "wait") == 0);
  digitalWrite(PIN_GREEN, !fault && (strcmp(towerState, "run") == 0 || (strcmp(towerState, "idle") == 0 && blink)));
}

void handleMove(JsonDocument &doc) {
  if (estopLatched) return reply(false, "estop");
  if (interlockLatched) return reply(false, "interlock");
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

void handleTool(JsonDocument &doc) {
  if (estopLatched) return reply(false, "estop");
  if (interlockLatched) return reply(false, "interlock");
  int arm = doc["arm"] | 0;
  if (arm < 1 || arm > NUM_ARMS) return reply(false, "bad arm");
  float value = doc["value"] | 0.0f;
  toolSignal[arm - 1] = value;
  if (value <= 0) offChannel(arm - 1, 5);
  else writeChannel(arm - 1, 5, value);
  reply(true);
}

void handleStatus() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["estop"] = estopLatched;
  doc["interlock"] = interlockLatched;
  doc["door_open"] = doorOpen();
  doc["curtain_broken"] = curtainBroken();
  doc["tower"] = towerState;
  doc["relaxed"] = relaxed;
  JsonArray list = doc["arms"].to<JsonArray>();
  JsonArray ids = doc["tool_ohms"].to<JsonArray>();
  JsonArray sig = doc["tool_signal"].to<JsonArray>();
  for (int a = 0; a < NUM_ARMS; a++) {
    JsonArray row = list.add<JsonArray>();
    for (int j = 0; j < JOINTS; j++) row.add(round(arms[a].now[j] * 10) / 10.0);
    float ohms = toolOhms(a);
    if (ohms < 0) ids.add(nullptr);
    else ids.add((int)ohms);
    sig.add(toolSignal[a]);
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
  else if (strcmp(cmd, "tool") == 0) handleTool(doc);
  else if (strcmp(cmd, "status") == 0) handleStatus();
  else if (strcmp(cmd, "ping") == 0) reply(true);
  else if (strcmp(cmd, "relax") == 0) { relaxAll(); reply(true); }
  else if (strcmp(cmd, "tower") == 0) {
    const char *st = doc["state"] | "idle";
    static const char *STATES[] = {"run", "wait", "idle", "fault"};
    towerState = "idle";
    for (const char *k : STATES) if (strcmp(st, k) == 0) towerState = k;
    reply(true);
  }
  else if (strcmp(cmd, "aux") == 0) {
    if (estopLatched || interlockLatched) return reply(false, "stopped");
    const char *name = doc["name"] | "";
    for (int i = 0; i < NUM_AUX; i++) if (strcmp(name, AUX[i].name) == 0) {
      AUX[i].from = AUX[i].now;
      AUX[i].to = constrain((float)(doc["deg"] | 0), 0.0f, 180.0f);
      AUX[i].start = millis();
      AUX[i].ms = max<uint32_t>(doc["ms"] | 800, TICK_MS);
      AUX[i].active = true;
      return reply(true);
    }
    reply(false, "unknown aux");
  }
  else if (strcmp(cmd, "fan") == 0) { digitalWrite(PIN_FAN, (doc["on"] | false) ? HIGH : LOW); reply(true); }
  else if (strcmp(cmd, "light") == 0) { digitalWrite(PIN_LIGHT, (doc["on"] | false) ? HIGH : LOW); reply(true); }
  else if (strcmp(cmd, "reset") == 0) {
    if (digitalRead(PIN_ESTOP) == HIGH) return reply(false, "estop still pressed");
    if (doorOpen()) return reply(false, "close the door first");
    if (curtainBroken()) return reply(false, "light curtain still blocked");
    estopLatched = false;
    interlockLatched = false;
    reply(true);
  } else reply(false, "unknown cmd");
}

// A seated probe reads ~4.7k. When its switch opens on contact the line reads
// open. (Releasing the probe at the rack looks the same; the Pi only counts
// probe events during a probing move, so that one is harmless.)
void watchProbes() {
  static float last[NUM_ARMS] = {-1, -1, -1, -1};
  for (int a = 0; a < NUM_ARMS; a++) {
    float ohms = toolOhms(a);
    bool wasProbe = last[a] > 0 && fabs(last[a] - PROBE_OHMS) < 0.25f * PROBE_OHMS;
    if (wasProbe && ohms < 0) emit("probe", a + 1);
    last[a] = ohms;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_DOOR, INPUT_PULLUP);
  pinMode(PIN_CURTAIN, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  for (int pin : {PIN_RED, PIN_YELLOW, PIN_GREEN, PIN_FAN, PIN_LIGHT}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  analogReadResolution(12);
  for (int a = 0; a < NUM_ARMS; a++) pinMode(ID_PINS[a], INPUT);  // external 10k pull-ups to 3.3 V

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
    relaxAll();
    emit("estop");
  }

  bool busy = false;
  for (int a = 0; a < NUM_ARMS; a++) busy |= arms[a].active || toolSignal[a] > 0;

  // Interlocks: door or light curtain while anything moves or spins
  if (!interlockLatched && busy && (doorOpen() || curtainBroken())) {
    interlockLatched = true;
    freezeAll();
    toolsOff();
    JsonDocument ev;
    ev["event"] = "interlock";
    ev["source"] = doorOpen() ? "door" : "curtain";
    serializeJson(ev, Serial);
    Serial.println();
  }

  // Panel buttons (simple 50 ms debounce)
  static uint32_t lastBtn = 0;
  static bool startWas = false, stopWas = false;
  if (millis() - lastBtn > 50) {
    lastBtn = millis();
    bool startNow = digitalRead(PIN_START) == LOW, stopNow = digitalRead(PIN_STOP) == LOW;
    if (startNow && !startWas) emit("start");
    if (stopNow && !stopWas) { freezeAll(); toolsOff(); towerState = "wait"; emit("stop"); }
    startWas = startNow;
    stopWas = stopNow;
  }
  setTower();

  // Watchdog: anything moving or spinning with a silent host gets stopped
  if (busy && millis() - lastHostMsg > WATCHDOG_MS) {
    freezeAll();
    toolsOff();
    emit("watchdog");
  }

  static uint32_t lastTick = 0;
  if (millis() - lastTick < TICK_MS) return;
  lastTick = millis();
  watchProbes();
  if (relaxed) return;

  for (int i = 0; i < NUM_AUX; i++) {
    Aux &x = AUX[i];
    if (x.active && !estopLatched && !interlockLatched) {
      float f = (float)(millis() - x.start) / x.ms;
      if (f >= 1.0f) { f = 1.0f; x.active = false; }
      x.now = x.from + (x.to - x.from) * f;
    }
    board0.setPWM(x.channel, 0, degToTicks(x.now));
  }

  for (int a = 0; a < NUM_ARMS; a++) {
    Motion &m = arms[a];
    if (m.active) {
      float f = (float)(millis() - m.start) / m.duration;
      if (f >= 1.0f) { f = 1.0f; m.active = false; }
      // Linear between waypoints: the planner already splits paths into
      // short straight segments, so the arm flows from one to the next.
      for (int j = 0; j < JOINTS; j++) m.now[j] = m.from[j] + (m.to[j] - m.from[j]) * f;
    }
    for (int j = 0; j < JOINTS; j++) writeChannel(a, j, m.now[j]);
  }
}
