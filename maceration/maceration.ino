// Maceration Controller
// Combines 4-channel relay control, passive buzzer alerts, speed sensor monitoring, and temperature monitoring
// 4 tactile push buttons provide toggle control for each relay

// ============================================================
// REQUIRED LIBRARIES
// ============================================================

#include <OneWire.h>
#include <DallasTemperature.h>

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

// ============================================================
// SENSOR INSTANCES
// ============================================================

// Setup OneWire instance to communicate with DS18B20
OneWire oneWire(ONE_WIRE_BUS);

// Pass OneWire reference to DallasTemperature library
DallasTemperature tempSensor(&oneWire);

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
