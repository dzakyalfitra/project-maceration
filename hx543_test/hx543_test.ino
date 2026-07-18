// HX-543 1x4 Membrane Keypad Test
// 4-button membrane keypad — each button shorts to GND when pressed
// Uses INPUT_PULLUP, same debounce pattern as button_test/

const int buttonPins[4] = {4, 16, 17, 5};

int currentState[4] = {HIGH, HIGH, HIGH, HIGH};
int lastState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounce[4] = {0, 0, 0, 0};
const unsigned long debounceDelay = 50;

// Label mapping for the HX-543 1x4 membrane (left to right)
const char* buttonLabels[4] = {"S1", "S2", "S3", "S4"};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  Serial.println("=== HX-543 1x4 Membrane Keypad Test ===");
  Serial.println("Pins: 2(S1), 4(S2), 16(S3), 17(S4)");
  Serial.println("Press any key...");
}

void loop() {
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
