#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "MAX30100_PulseOximeter.h"
#include <DHT.h>

#define LCD_RS 7
#define LCD_EN 8
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define REPORTING_PERIOD_MS 1000
PulseOximeter pox;
uint32_t lastReport = 0;
float spo2 = 0, heartRate = 0;

#define DHTPIN A3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature = 0, humidity = 0;

#define P_RED_LED 2
#define P_GREEN_LED 3
#define P_SETTING_BUTTON 4
#define P_START_BUTTON 5
#define P_SERVO 6
#define P_ADJUST_POT A0

Servo servo;
bool ventilating = false;

bool inSettingsMode = false;
unsigned long settingButtonPressedAt = 0;
int menuIndex = 0;
int inhaleTime = 1500;
int exhaleTime = 1500;

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

    lcd.begin(16, 4);
    lcd.setCursor(0, 0);
    lcd.print("Ventilator Ready");

    dht.begin();
    if (!pox.begin()) {
        Serial.println("MAX30100 INIT FAILED");
        digitalWrite(P_RED_LED, HIGH);
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
}

void displayNormalData() {
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

void displaySettingsMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Settings Mode");

    switch (menuIndex) {
        case 0:
            lcd.setCursor(0, 1);
            lcd.print(">Inhale Time");
            lcd.setCursor(0, 2);
            lcd.print(" "); lcd.print(map(analogRead(P_ADJUST_POT), 0, 1023, 500, 3000)); lcd.print("ms");
            break;
        case 1:
            lcd.setCursor(0, 1);
            lcd.print(">Exhale Time");
            lcd.setCursor(0, 2);
            lcd.print(" "); lcd.print(map(analogRead(P_ADJUST_POT), 0, 1023, 500, 3000)); lcd.print("ms");
            break;
        case 2:
            lcd.setCursor(0, 1);
            lcd.print(">Exit Settings");
            break;
    }
}

void loop() {
    updateSensorData();

    if (!digitalRead(P_SETTING_BUTTON)) {
        if (settingButtonPressedAt == 0) settingButtonPressedAt = millis();
        if ((millis() - settingButtonPressedAt > 1000) && !inSettingsMode) {
            inSettingsMode = true;
            menuIndex = 0;
            lcd.clear();
        }
    } else {
        settingButtonPressedAt = 0;
    }

    if (inSettingsMode) {
        displaySettingsMenu();

        if (!digitalRead(P_START_BUTTON) && millis() - lastDebounceTime > debounceDelay) {
            lastDebounceTime = millis();
            menuIndex = (menuIndex + 1) % 3;
        }

        int adjustedValue = map(analogRead(P_ADJUST_POT), 0, 1023, 500, 3000);
        if (menuIndex == 0) inhaleTime = adjustedValue;
        if (menuIndex == 1) exhaleTime = adjustedValue;
        if (menuIndex == 2 && !digitalRead(P_START_BUTTON) && millis() - lastDebounceTime > debounceDelay) {
            inSettingsMode = false;
            lastDebounceTime = millis();
            lcd.clear();
        }
        return;
    }

    displayNormalData();

    if (!digitalRead(P_START_BUTTON)) {
        if (millis() - lastDebounceTime > debounceDelay) {
            ventilating = !ventilating;
            lastDebounceTime = millis();
        }
    }

    if (ventilating) {
        servo.write(90);
        delay(inhaleTime);
        servo.write(0);
        delay(exhaleTime);
    }
}