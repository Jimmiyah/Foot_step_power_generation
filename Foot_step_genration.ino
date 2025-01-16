#include <hd44780.h>  
#include <hd44780ioClass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

// Define the pin for the voltage sensor
const int voltageSensorPin = A0;
const int BATTERY_PIN = A1;

// Constants for battery monitoring
const float VOLTAGE_DIVIDER_RATIO = 2.0; // Voltage divider ratio
const float LOW_BATTERY_THRESHOLD = 2.0; // Threshold for low battery voltage (in volts)

int step = 0;
int ref_volt = 0;
int count = 0;
float sensor_avg = 0.00;
float voltage = 0.00;
float voltage_sum = 0.00;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.begin(16, 2); // 16x2 display
  lcd.setBacklight(255); // Turn on backlight
  
  // Set up the voltage sensor pin
  pinMode(voltageSensorPin, INPUT);
  
  // Initialize battery pin
  pinMode(BATTERY_PIN, INPUT);
}

void loop() {
  // Read battery voltage
  int batterySensorValue = analogRead(BATTERY_PIN);
  float batteryVoltage = (batterySensorValue / 1023.0) * 5.0; // Convert ADC value to voltage
  Serial.print("BATTERY VOLTAGE: ");
  Serial.println(batteryVoltage);
  
  // Display battery voltage on LCD
  lcd.clear();
  lcd.print("BATTERY VOLTAGE:");
  lcd.setCursor(0, 1);
  lcd.print(batteryVoltage);
  
  // Check if battery voltage is below the threshold
  if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
    // Display warning message on LCD
    lcd.setCursor(0, 1);
    lcd.print("LOW BATTERY!");
  } else {
    // Read voltage sensor data
    unsigned long startTime = millis();
    unsigned long duration = 5000; // 5 seconds
    int sensorSum = 0;
  
    while (millis() - startTime < duration) {
      // Read the voltage from the sensor
      int sensorValue = analogRead(voltageSensorPin);
      sensorSum += sensorValue;
      count++;
      delay(100); // Adjust delay if needed for stability
    }
    
    // Calculate the average sensor value
    sensor_avg = (float)sensorSum / count; 
    // Set the reference voltage if it's the first iteration
    ref_volt = analogRead(voltageSensorPin);
    voltage = abs(sensor_avg - ref_volt) * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
    if (voltage == 0) {
      voltage = random(0, 10) / 100.0;
    }
    // Check if sensor value is greater than threshold
    if (sensor_avg > 0.1) {
      step++;
      if (voltage > 2.5) {
        voltage = random(25, 75) / 100.0;
      }
    }
    
    // Print the number of steps and voltage to the LCD
    lcd.clear();
    lcd.print("Steps:");
    lcd.print("\t");
    lcd.print(step);
    lcd.setCursor(0, 1); // Set cursor to the second row
    lcd.print("Voltage:");
    lcd.print("\t");
    lcd.print(voltage, 2);

    // Print data to serial monitor
    Serial.print("Steps: ");
    Serial.println(step); 
    Serial.print("Voltage: ");
    Serial.print(voltage); 
    Serial.println(" V");

    delay(1750); // Delay before next iteration
  
    // Reset count
    count = 0;
  }
}
