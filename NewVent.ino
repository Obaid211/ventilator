#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "MAX30100_PulseOximeter.h"
#include <DHT.h>

// Regular LCD (16x4) Pin Connections
#define LCD_RS 7
#define LCD_EN 8
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// MAX30100 Sensor
#define REPORTING_PERIOD_MS 1000
PulseOximeter pox;
uint32_t lastReport = 0;
float spo2 = 0, heartRate = 0;

// DHT11 Sensor
#define DHTPIN A3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature = 0, humidity = 0;

// Pin Definitions
#define P_RED_LED 2
#define P_GREEN_LED 3
#define P_SETTING_BUTTON 4
#define P_START_BUTTON 5
#define P_SERVO 6
#define P_VOLUME_POT A0
#define P_PRESSURE_POT A1
#define P_RATE_POT A2

// Servo Control Parameters
#define SERVO_UPDATE_INTERVAL 15
Servo servo;
uint16_t desired_servo_pos = 0;
uint16_t current_servo_pos = 0;
long last_servo_update = 0;
bool ventilating = false;

// Button Debounce
unsigned long lastDebounceTime = 0;
const int debounceDelay = 300;

void onBeatDetected() {
    Serial.println("Heartbeat detected!");
    digitalWrite(P_GREEN_LED, HIGH);
    delay(50);
    digitalWrite(P_GREEN_LED, LOW);
}

void setup() {
    Serial.begin(9600);

    pinMode(P_RED_LED, OUTPUT);
    pinMode(P_GREEN_LED, OUTPUT);
    pinMode(P_SETTING_BUTTON, INPUT_PULLUP);
    pinMode(P_START_BUTTON, INPUT_PULLUP);
    
    servo.attach(P_SERVO);
    servo.write(0);
    
    lcd.begin(16, 4);  // Set up LCD as 16x4
    lcd.setCursor(0, 0);
    lcd.print("Ventilator Ready");
    
    dht.begin();
    if (!pox.begin()) {
        Serial.println("MAX30100 INIT FAILED");
        digitalWrite(P_RED_LED, HIGH);  // Turn on error LED
        lcd.setCursor(0, 1);
        lcd.print("MAX30100 FAIL!");
    } else {
        pox.setOnBeatDetectedCallback(onBeatDetected);
    }
}

void updateSensorData() {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    
    pox.update();
    if (millis() - lastReport > REPORTING_PERIOD_MS) {
        spo2 = pox.getSpO2();
        heartRate = pox.getHeartRate();
        lastReport = millis();
    }
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HR:"); lcd.print(heartRate, 0); lcd.print(" BPM ");

    lcd.setCursor(0, 1);
    lcd.print("SpO2:"); lcd.print(spo2, 0); lcd.print("%");

    lcd.setCursor(0, 2);
    lcd.print("Temp:"); lcd.print(temperature, 1); lcd.print("C ");
    lcd.print("Hum:"); lcd.print(humidity, 0); lcd.print("%");

    lcd.setCursor(0, 3);
    lcd.print("Ventilator: ");
    lcd.print(ventilating ? "ON " : "OFF");
}

void loop() {
    updateSensorData();
    
    // Toggle ventilator on button press (with debounce)
    if (!digitalRead(P_START_BUTTON)) {
        if (millis() - lastDebounceTime > debounceDelay) {
            ventilating = !ventilating;
            lastDebounceTime = millis();
        }
    }

    // Read potentiometers
    int volume = analogRead(P_VOLUME_POT);
    int pressure = analogRead(P_PRESSURE_POT);
    int rate = analogRead(P_RATE_POT);

    // Scale potentiometer values
    int inhaleTime = map(rate, 0, 1023, 500, 3000);  // 0.5s - 3s inhale time
    int exhaleTime = inhaleTime; // Simple exhale timing

    if (ventilating) {
        servo.write(90);  // Simulate pressing AMBU bag
        delay(inhaleTime);
        servo.write(0);   // Release AMBU bag
        delay(exhaleTime);
    }
}
