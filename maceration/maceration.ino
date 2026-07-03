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
const int relayPins[4] = {13, 12, 27, 26};

// Button pins (INPUT_PULLUP mode: active-LOW, connect to GND when pressed)
const int buttonPins[4] = {2, 4, 16, 17};  

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
bool relayState[4] = {false, false, false, false};

// Button state tracking
int currentButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
int lastButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[4] = {0, 0, 0, 0};
const unsigned long debounceDelay = 50;

// Buzzer timing
bool startupMelodyPlayed = false;

// Speed sensor
volatile unsigned int pulseCount = 0;
unsigned int rpm = 0;
unsigned long previousMillis = 0;
const int interval = 1000;
const int slotsOnDisk = 20;

// Temperature sensor
float temperature = 0.0;
unsigned long previousTempMillis = 0;
const int tempInterval = 2000;

// Display
float gearAngle = 0;       // Current gear rotation angle (degrees)
unsigned long previousDisplayMillis = 0;
const int displayInterval = 250;  // Update display every 250ms

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
  for (int i = 0; i < 4; i++) {
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

  // Initialize temperature sensor
  tempSensor.begin();

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
  updateDisplay();
}

// ============================================================
// BUTTON HANDLER
// ============================================================

void handleButtons() {
  for (int i = 0; i < 4; i++) {
    int reading = digitalRead(buttonPins[i]);

    // If button state changed, reset debounce timer
    if (reading != lastButtonState[i]) {
      lastDebounceTime[i] = millis();
    }

    // Check if enough time has passed for debouncing
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      // If button state has actually changed
      if (reading != currentButtonState[i]) {
        currentButtonState[i] = reading;

        // Print status based on button state
        Serial.print("Button ");
        Serial.print(i + 1);
        Serial.print(" (Pin ");
        Serial.print(buttonPins[i]);
        Serial.print("): ");

        if (currentButtonState[i] == LOW) {
          // Button is pressed (connected to GND)
          Serial.println("PRESSED");
          toggleRelay(i);
        } else {
          // Button is released (pulled HIGH by internal resistor)
          Serial.println("RELEASED");
        }
      }
    }

    lastButtonState[i] = reading;
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

  // Play button press beep (single note)
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  // Print state change to Serial
  Serial.print("Relay ");
  Serial.print(index + 1);
  Serial.print(": ");
  Serial.println(relayState[index] ? "ON" : "OFF");
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
    // Request temperature reading from DS18B20
    tempSensor.requestTemperatures();

    // Read temperature in Celsius
    temperature = tempSensor.getTempCByIndex(0);

    // Print results
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

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

// Main display update — called every displayInterval ms
void updateDisplay() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDisplayMillis < displayInterval) return;
  previousDisplayMillis = currentMillis;

  // Clear screen
  tft.fillScreen(ST77XX_BLACK);

  // ---- Title ----
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 3);
  tft.print("MACERATION CONTROLLER");

  // ---- RPM Section ----
  // Rotating gear icon (rotation speed proportional to RPM)
  // Gear rotates faster at higher RPM
  float rpmFactor = (rpm > 0) ? (float)rpm / 300.0 : 0;
  gearAngle += rpmFactor * 30;  // 30 degrees per update at 300 RPM
  if (gearAngle >= 360) gearAngle -= 360;

  uint16_t rpmColor = (rpm > 0) ? ST77XX_CYAN : 0x7BEF;  // Bright if spinning, dim if stopped
  drawGear(22, 28, 14, 9, 6, gearAngle, rpmColor);

  // RPM label and value
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

  // ---- Separator ----
  drawSeparator(55);

  // ---- Temperature Section ----
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

  // ---- Separator ----
  drawSeparator(100);

  // ---- Relay Status Section (2x2 grid) ----
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 105);
  tft.print("RELAYS");

  uint16_t onColor  = 0x07E0;   // Green
  uint16_t offColor = 0x7BEF;   // Dim gray
  const char* relayLabels[] = {"R1", "R2", "R3", "R4"};

  for (int i = 0; i < 4; i++) {
    int col = i % 2;
    int row = i / 2;
    int x = 10 + col * 115;
    int y = 120 + row * 50;

    uint16_t bgColor = relayState[i] ? 0x0400 : 0x2104;  // Dark green bg if ON, dark bg if OFF
    tft.fillRoundRect(x, y, 105, 40, 5, bgColor);
    tft.drawRoundRect(x, y, 105, 40, 5, relayState[i] ? onColor : offColor);

    // Relay label
    tft.setTextColor(relayState[i] ? onColor : offColor);
    tft.setTextSize(1);
    tft.setCursor(x + 5, y + 5);
    tft.print(relayLabels[i]);

    // Status dot
    tft.fillCircle(x + 28, y + 12, 4, relayState[i] ? onColor : offColor);

    // ON/OFF text
    tft.setTextSize(2);
    tft.setCursor(x + 38, y + 5);
    tft.print(relayState[i] ? "ON" : "OFF");

    // Pin info
    tft.setTextSize(1);
    tft.setTextColor(offColor);
    tft.setCursor(x + 5, y + 24);
    tft.print("Pin ");
    tft.print(relayPins[i]);
  }
}
