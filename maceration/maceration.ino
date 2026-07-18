// Maceration Controller
// Combines 4-channel relay control, passive buzzer alerts, speed sensor monitoring, and temperature monitoring
// 4 tactile push buttons provide toggle control for each relay

// ============================================================
// REQUIRED LIBRARIES
// ============================================================

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// Relay pins (active-LOW: LOW=ON, HIGH=OFF)
// 3 relays — IN1, IN2, IN3. GPIO12 is intentionally avoided:
// it's a strapping pin and the relay module's pull-up would prevent ESP32 boot.
// Also disconnect GPIO12 from the relay module on the hardware side.
const int relayPins[3] = {27, 26, 13};

// Button pins (INPUT_PULLUP mode: active-LOW, connect to GND when pressed)
const int buttonPins[4] = {15, 4, 16, 17};  

// Buzzer pin
const int buzzerPin = 23;

// Speed sensor pin
#define SENSOR_PIN 14

// Temperature sensor pin (DS18B20)
#define ONE_WIRE_BUS 22

// TFT Display pins (ST7789 240x240)
// Remapped to avoid conflicts with buttons (2,4), buzzer (23)
#define TFT_CS    -1  // pin BLK
#define TFT_DC    5   // pin DC
#define TFT_RST   19  // pin RES
#define TFT_MOSI  21  // pin SDA
#define TFT_SCLK  18  // pin SCL

// ============================================================
// DISPLAY & SENSOR INSTANCES
// ============================================================

// Setup OneWire instance to communicate with DS18B20
OneWire oneWire(ONE_WIRE_BUS);

// Pass OneWire reference to DallasTemperature library
DallasTemperature tempSensor(&oneWire);

// ST7789 TFT display instance
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ============================================================
// BUZZER NOTE FREQUENCIES
// ============================================================

#define NOTE_G5  784
#define NOTE_FS5 740
#define NOTE_DS5 622
#define NOTE_A4  440
#define NOTE_GS4 415
#define NOTE_E5  659
#define NOTE_GS5 831
#define NOTE_C6  1047

// ============================================================
// GLOBAL VARIABLES
// ============================================================

// Relay state tracking (true = ON/LOW, false = OFF/HIGH)
bool relayState[3] = {false, false, false};

// Button state tracking
const char* buttonLabels[4] = {"S1", "S2", "S3", "S4"};
int currentState[4] = {HIGH, HIGH, HIGH, HIGH};
int lastState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounce[4] = {0, 0, 0, 0};
const unsigned long debounceDelay = 50;

// Buzzer timing
bool startupMelodyPlayed = false;

// Non-blocking beep state
bool beepActive = false;
unsigned long beepStartTime = 0;
const unsigned long beepDuration = 80;

// Speed sensor
volatile unsigned int pulseCount = 0;
unsigned int rpm = 0;
unsigned long previousMillis = 0;
const int interval = 1000;
const int slotsOnDisk = 20;

// Temperature sensor
float temperature = -127.0;
unsigned long previousTempMillis = 0;
const int tempInterval = 2000;
bool tempFirstRead = true;

// Display
float gearAngle = 0;       // Current gear rotation angle (degrees)
unsigned long previousDisplayMillis = 0;
const int displayInterval = 150;  // Update display every 150ms
bool displayFirstDraw = true;     // First frame: draw static layout

// Change-detection: track last drawn values to avoid redundant redraws
unsigned int lastDrawnRpm = 0xFFFFFFFF;
float lastDrawnTemp = -999.0;
bool lastDrawnRelay[3] = {false, false, false};

// Display region Y coordinates (prevents overlap)
#define DISPLAY_RPM_Y    15
#define DISPLAY_RPM_H    37
#define DISPLAY_TEMP_Y   58
#define DISPLAY_TEMP_H   37
#define DISPLAY_RELAY_Y  115
#define DISPLAY_RELAY_H  120

// ============================================================
// INTERRUPT SERVICE ROUTINE
// ============================================================

void IRAM_ATTR countPulse() {
  pulseCount++;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Configure relay pins as outputs, initialize to OFF (HIGH)
  for (int i = 0; i < 3; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);
  }

  // Configure button pins with internal pull-up resistors
  // Buttons should connect to GND when pressed (active-LOW)
  for (int i = 0; i < 4; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Configure buzzer pin
  pinMode(buzzerPin, OUTPUT);

  // Configure speed sensor pin
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // Attach interrupt for speed sensor
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);

  // Initialize temperature sensor (non-blocking mode — don't wait for conversion)
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();  // Kick off first conversion immediately

  // Initialize TFT display
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  Serial.println("FC-03 Speed Sensor Initialized.");
  Serial.println("DS18B20 Temperature Sensor Initialized.");
  Serial.println("All systems ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  handleButtons();
  handleSpeedSensor();
  handleTemperature();
  handleBuzzer();
  handleBeep();
  updateDisplay();
}

// ============================================================
// BUTTON HANDLER
// ============================================================

void handleButtons() {
  for (int i = 0; i < 4; i++) {
    int reading = digitalRead(buttonPins[i]);

    if (reading != lastState[i]) {
      lastDebounce[i] = millis();
    }

    if ((millis() - lastDebounce[i]) > debounceDelay) {
      if (reading != currentState[i]) {
        currentState[i] = reading;
        if (currentState[i] == LOW) {
          Serial.print("[");
          Serial.print(buttonLabels[i]);
          Serial.println("] PRESSED");
          if (i == 3) toggleAllRelays();
          else        toggleRelay(i);
        } else {
          Serial.print("[");
          Serial.print(buttonLabels[i]);
          Serial.println("] RELEASED");
        }
      }
    }

    lastState[i] = reading;
  }
}

// ============================================================
// RELAY TOGGLE FUNCTION
// ============================================================

void toggleRelay(int index) {
  // Flip the relay state
  relayState[index] = !relayState[index];

  // Apply to physical relay (active-LOW logic)
  digitalWrite(relayPins[index], relayState[index] ? LOW : HIGH);

  // Start non-blocking beep (handled by handleBeep in main loop)
  beepActive = true;
  beepStartTime = millis();
  tone(buzzerPin, 1000);

  // Print state change to Serial
  Serial.print("Relay ");
  Serial.print(index + 1);
  Serial.print(": ");
  Serial.println(relayState[index] ? "ON" : "OFF");
}

// Group toggle: if any relay is on, turn all off; otherwise turn all on.
void toggleAllRelays() {
  bool anyOn = false;
  for (int i = 0; i < 3; i++) if (relayState[i]) { anyOn = true; break; }
  bool target = !anyOn;
  for (int i = 0; i < 3; i++) {
    relayState[i] = target;
    digitalWrite(relayPins[i], target ? LOW : HIGH);
  }
  beepActive = true;
  beepStartTime = millis();
  tone(buzzerPin, 1000);
  Serial.print("All relays: ");
  Serial.println(target ? "ON" : "OFF");
}

// ============================================================
// SPEED SENSOR HANDLER
// ============================================================

void handleSpeedSensor() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Temporarily detach interrupt
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));

    // Calculate RPM
    rpm = (pulseCount * 60) / slotsOnDisk;

    // Print results
    Serial.print("Pulses per sec: ");
    Serial.print(pulseCount);
    Serial.print(" | Speed: ");
    Serial.print(rpm);
    Serial.println(" RPM");

    // Reset counter and update timer
    pulseCount = 0;
    previousMillis = currentMillis;

    // Reattach interrupt
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);
  }
}

// ============================================================
// TEMPERATURE SENSOR HANDLER
// ============================================================

void handleTemperature() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousTempMillis >= tempInterval) {
    // Read the result from the PREVIOUS request (non-blocking)
    float t = tempSensor.getTempCByIndex(0);

    if (t != DEVICE_DISCONNECTED_C) {
      temperature = t;
    }

    // Print results (only on change)
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Kick off the NEXT conversion (returns immediately, conversion happens in background)
    tempSensor.requestTemperatures();

    // Update timer
    previousTempMillis = currentMillis;
  }
}

// ============================================================
// BUZZER HANDLER
// ============================================================

void handleBuzzer() {
  // Play Zelda melody only once at startup
  if (!startupMelodyPlayed) {
    int melody[] = {
      NOTE_G5, NOTE_FS5, NOTE_DS5, NOTE_A4,
      NOTE_GS4, NOTE_E5, NOTE_GS5, NOTE_C6
    };

    int durations[] = {
      100, 100, 100, 100,
      100, 100, 100, 600
    };

    int numNotes = sizeof(melody) / sizeof(melody[0]);

    for (int i = 0; i < numNotes; i++) {
      tone(buzzerPin, melody[i]);
      delay(durations[i]);
      noTone(buzzerPin);
      delay(20);
    }

    startupMelodyPlayed = true;
  }
}

// ============================================================
// NON-BLOCKING BEEP HANDLER
// ============================================================

void handleBeep() {
  if (beepActive && (millis() - beepStartTime >= beepDuration)) {
    noTone(buzzerPin);
    beepActive = false;
  }
}

// ============================================================
// DISPLAY FUNCTIONS
// ============================================================

// Draw a rotating gear icon at (cx, cy) with given radius and number of teeth
// Gear rotates based on gearAngle (in degrees)
void drawGear(int cx, int cy, int outerR, int innerR, int teeth, float angle, uint16_t color) {
  float step = 360.0 / teeth;
  float halfTooth = step * 0.3;

  for (int i = 0; i < teeth; i++) {
    float baseAngle = angle + i * step;
    float a1 = radians(baseAngle - halfTooth);
    float a2 = radians(baseAngle + halfTooth);

    // Outer tooth rectangle (two triangles approximating a trapezoid)
    int x1 = cx + cos(a1) * innerR;
    int y1 = cy + sin(a1) * innerR;
    int x2 = cx + cos(a1) * outerR;
    int y2 = cy + sin(a1) * outerR;
    int x3 = cx + cos(a2) * outerR;
    int y3 = cy + sin(a2) * outerR;
    int x4 = cx + cos(a2) * innerR;
    int y4 = cy + sin(a2) * innerR;

    tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
    tft.fillTriangle(x1, y1, x3, y3, x4, y4, color);
  }

  // Center hub circle
  tft.fillCircle(cx, cy, innerR - 1, color);

  // Center hole (dark)
  tft.fillCircle(cx, cy, 4, ST77XX_BLACK);
}

// Draw a thermometer icon at (cx, cy) with mercury level based on temperature
void drawThermometer(int cx, int cy, float temp, uint16_t color) {
  // Bulb at bottom
  tft.fillCircle(cx, cy + 12, 6, color);
  tft.fillCircle(cx, cy + 12, 4, ST77XX_BLACK);  // hollow center

  // Tube
  tft.fillRect(cx - 3, cy - 18, 6, 30, color);

  // Clear inside of tube (mercury area will be filled separately)
  tft.fillRect(cx - 1, cy - 16, 2, 24, ST77XX_BLACK);

  // Mercury fill — map temperature range 0-100°C to fill height
  int fillH = map(constrain((int)temp, 0, 100), 0, 100, 0, 22);
  int mercuryColor = ST77XX_RED;
  if (temp < 30) mercuryColor = 0x07FF;       // Cyan for cool
  else if (temp < 60) mercuryColor = ST77XX_YELLOW; // Yellow for warm
  else mercuryColor = ST77XX_RED;              // Red for hot

  tft.fillRect(cx - 1, cy + 6 - fillH, 2, fillH, mercuryColor);
  tft.fillCircle(cx, cy + 12, 4, mercuryColor);

  // Tick marks on right side
  for (int i = 0; i < 4; i++) {
    int yTick = cy + 6 - i * 6;
    tft.drawPixel(cx + 4, yTick, ST77XX_WHITE);
    tft.drawPixel(cx + 5, yTick, ST77XX_WHITE);
  }
}

// Separator line across the display
void drawSeparator(int y) {
  tft.drawFastHLine(5, y, 230, 0x7BEF);  // Dim gray line
}

// Clear only a specific rectangular region (instead of full screen)
void clearRegion(int x, int y, int w, int h) {
  tft.fillRect(x, y, w, h, ST77XX_BLACK);
}

// Draw static layout elements (title + separators) — called once
void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);

  // Title
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 3);
  tft.print("MACERATION CONTROLLER");

  // Separators
  drawSeparator(55);
  drawSeparator(100);

  // Relay section header
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 105);
  tft.print("RELAYS");
}

// Main display update — called every displayInterval ms
// Change-detection: only redraws zones whose data actually changed.
// When nothing changes, zero SPI traffic = zero flicker.
void updateDisplay() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDisplayMillis < displayInterval) return;
  previousDisplayMillis = currentMillis;

  // First frame: draw the complete static layout
  if (displayFirstDraw) {
    drawStaticLayout();
    displayFirstDraw = false;
    lastDrawnRpm = rpm;
    lastDrawnTemp = temperature;
    for (int i = 0; i < 3; i++) lastDrawnRelay[i] = relayState[i];
    // Draw first frame of all sections (force all flags true)
    drawSections(true, true, true);
    return;
  }

  // ---- Check what changed ----
  bool rpmChanged   = (rpm != lastDrawnRpm) || (rpm > 0);  // gear always animates if rpm>0
  bool tempChanged   = (temperature != lastDrawnTemp);
  bool relayChanged  = false;
  for (int i = 0; i < 3; i++) {
    if (relayState[i] != lastDrawnRelay[i]) {
      relayChanged = true;
      break;
    }
  }

  // Nothing changed and rpm is 0 — skip entirely
  if (!rpmChanged && !tempChanged && !relayChanged) return;

  drawSections(rpmChanged, tempChanged, relayChanged);
}

// Draw the three display sections, only those whose flag is true
void drawSections(bool doRpm, bool doTemp, bool doRelay) {
  // ---- RPM Section ----
  if (doRpm) {
    clearRegion(0, DISPLAY_RPM_Y, 240, DISPLAY_RPM_H);

    float rpmFactor = (rpm > 0) ? (float)rpm / 300.0 : 0;
    gearAngle += rpmFactor * 30;
    if (gearAngle >= 360) gearAngle -= 360;

    uint16_t rpmColor = (rpm > 0) ? ST77XX_CYAN : 0x7BEF;
    drawGear(22, 28, 14, 9, 6, gearAngle, rpmColor);

    tft.setTextSize(1);
    tft.setTextColor(rpmColor);
    tft.setCursor(40, 18);
    tft.print("RPM");

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(40, 28);
    char rpmBuf[8];
    sprintf(rpmBuf, "%d", rpm);
    tft.print(rpmBuf);

    lastDrawnRpm = rpm;
  }

  // ---- Temperature Section ----
  if (doTemp) {
    clearRegion(0, DISPLAY_TEMP_Y, 240, DISPLAY_TEMP_H);

    drawThermometer(22, 72, temperature, ST77XX_WHITE);

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(40, 62);
    tft.print("TEMP");

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(40, 72);
    char tempBuf[10];
    dtostrf(temperature, 1, 1, tempBuf);
    tft.print(tempBuf);
    tft.setTextSize(1);
    tft.print(" ");
    tft.print((char)247);  // ° symbol
    tft.print("C");

    lastDrawnTemp = temperature;
  }

  // ---- Relay Status Section ----
  if (doRelay) {
    clearRegion(0, DISPLAY_RELAY_Y, 240, DISPLAY_RELAY_H);

    uint16_t onColor  = 0x07E0;   // Green
    uint16_t offColor = 0x7BEF;   // Dim gray
    const char* relayLabels[] = {"R1", "R2", "R3"};

    for (int i = 0; i < 3; i++) {
      int col = i % 2;
      int row = i / 2;
      int x = 10 + col * 115;
      int y = 120 + row * 50;

      uint16_t bgColor = relayState[i] ? 0x0400 : 0x2104;
      tft.fillRoundRect(x, y, 105, 40, 5, bgColor);
      tft.drawRoundRect(x, y, 105, 40, 5, relayState[i] ? onColor : offColor);

      tft.setTextColor(relayState[i] ? onColor : offColor);
      tft.setTextSize(1);
      tft.setCursor(x + 5, y + 5);
      tft.print(relayLabels[i]);

      tft.fillCircle(x + 28, y + 12, 4, relayState[i] ? onColor : offColor);

      tft.setTextSize(2);
      tft.setCursor(x + 38, y + 5);
      tft.print(relayState[i] ? "ON" : "OFF");

      tft.setTextSize(1);
      tft.setTextColor(offColor);
      tft.setCursor(x + 5, y + 24);
      tft.print("Pin ");
      tft.print(relayPins[i]);

      lastDrawnRelay[i] = relayState[i];
    }
  }
}
