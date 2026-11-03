#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD screen initialization (I2C address often 0x27 or 0x3F, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- LCD Program Start ---");

  // I2C bus initialization on the ESP32 (SDA = 21, SCL = 22)
  Wire.begin(21, 22);
  Serial.println("I2C bus initialized (SDA=21, SCL=22)");

  // LCD screen initialization
  lcd.init();                      
  lcd.backlight(); // Turn on backlight
  Serial.println("LCD initialized and backlight activated");

  // Display on the 1st row
  lcd.setCursor(0, 0); // Column 0, Row 0
  lcd.print("RED CUBE");

  // Display on the 2nd row
  lcd.setCursor(0, 1); // Column 0, Row 1
  lcd.print("SORTED"); 
  Serial.println("Text sent to LCD successfully!");
}

void loop() {
  // Nothing to do here for a static display
}
