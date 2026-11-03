// =========================================================================
// PINS CONFIGURATION
// =========================================================================
const int BUTTON_PIN = 4;   // Push button connected to pin 4
const int LED_1_PIN = 10;   // First LED connected to pin 10
const int LED_2_PIN = 11;   // Second LED connected to pin 11
const int LED_3_PIN = 12;   // Third LED connected to pin 12

// Variables for button state management
int buttonState = 0;         // Current state of the button
int lastButtonState = HIGH;  // Previous state of the button (using INPUT_PULLUP)
int stepSequence = 0;        // Keeps track of the LED sequence step

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Push Button & 3 LEDs Program Start ---");

  // Initialize button pin with internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize LED pins as outputs
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);

  // Ensure all LEDs are off at startup
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
  digitalWrite(LED_3_PIN, LOW);
}

void loop() {
  // Read the current state of the button
  // Note: With INPUT_PULLUP, LOW means pressed and HIGH means released
  buttonState = digitalRead(BUTTON_PIN);

  // Check if the button was pressed (transition from HIGH to LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50); // Small debounce delay

    // Increment sequence step (0, 1, 2, then reset to 0)
    stepSequence = (stepSequence + 1) % 4;

    Serial.print("Button pressed! Current Step: ");
    Serial.println(stepSequence);

    // Control LEDs successively based on the step
    switch (stepSequence) {
      case 1:
        digitalWrite(LED_1_PIN, HIGH);
        digitalWrite(LED_2_PIN, LOW);
        digitalWrite(LED_3_PIN, LOW);
        Serial.println("LED 1 ON");
        break;

      case 2:
        digitalWrite(LED_1_PIN, LOW);
        digitalWrite(LED_2_PIN, HIGH);
        digitalWrite(LED_3_PIN, LOW);
        Serial.println("LED 2 ON");
        break;

      case 3:
        digitalWrite(LED_1_PIN, LOW);
        digitalWrite(LED_2_PIN, LOW);
        digitalWrite(LED_3_PIN, HIGH);
        Serial.println("LED 3 ON");
        break;

      default:
        digitalWrite(LED_1_PIN, LOW);
        digitalWrite(LED_2_PIN, LOW);
        digitalWrite(LED_3_PIN, LOW);
        Serial.println("All LEDs OFF");
        break;
    }
  }

  // Save the current button state for the next loop check
  lastButtonState = buttonState;
  
  delay(10); // Short delay for system stability
}
