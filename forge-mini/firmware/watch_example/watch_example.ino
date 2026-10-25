// Example firmware for the perfboard smartwatch (ESP32-C3 SuperMini).
//
// Shows a clock and checks its own hardware at boot. The cell's USB-C dock
// flashes this sketch and waits for "SELFTEST OK" on the USB serial port;
// "SELFTEST FAIL ..." stops the run. Replace the clock with your own watch
// face; keep the self-test lines.
//
// Wiring used by smartwatch.yaml:
//   OLED SSD1306 128x64 I2C: SDA GPIO 8, SCL GPIO 9, address 0x3C
//   Buttons to GND: GPIO 2, 3, 4 (internal pull-ups)
//   P-MOSFET gate (OLED power switch): GPIO 10, LOW = on, 10k pull-up to 3.3 V
// Libraries: Adafruit SSD1306, Adafruit GFX.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int PIN_SDA = 8, PIN_SCL = 9, PIN_PWR = 10;
const int BUTTONS[3] = {2, 3, 4};
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
uint32_t setMillis = 0;
int hh = 12, mm = 0;

bool selfTest(String &why) {
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() != 0) { why = "oled not on i2c (check SDA/SCL tracks)"; return false; }
  for (int i = 0; i < 3; i++)
    if (digitalRead(BUTTONS[i]) == LOW) { why = "button " + String(i + 1) + " stuck low (solder bridge?)"; return false; }
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_PWR, LOW);  // switch the OLED on
  for (int b : BUTTONS) pinMode(b, INPUT_PULLUP);
  delay(50);
  Wire.begin(PIN_SDA, PIN_SCL);
  String why;
  bool ok = selfTest(why) && oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!ok && why.length() == 0) why = "oled init failed";
  Serial.println(ok ? "SELFTEST OK" : "SELFTEST FAIL " + why);
  oled.clearDisplay();
  oled.display();
}

void loop() {
  if (digitalRead(BUTTONS[0]) == LOW) { hh = (hh + 1) % 24; delay(200); }
  if (digitalRead(BUTTONS[1]) == LOW) { mm = (mm + 1) % 60; delay(200); }
  uint32_t minutes = (millis() - setMillis) / 60000;
  int h = (hh + (mm + minutes) / 60) % 24, m = (mm + minutes) % 60;
  char buf[6];
  snprintf(buf, sizeof buf, "%02d:%02d", h, m);
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(3);
  oled.setCursor(20, 18);
  oled.print(buf);
  oled.display();
  delay(250);
}
