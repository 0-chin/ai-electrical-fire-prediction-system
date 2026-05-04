#include <OneWire.h>
#include <DallasTemperature.h>

#define MQ2_PIN 3
#define TEMP_PIN 4
#define BUZZER_PIN 15
#define RED_LED 12
#define GREEN_LED 13
#define BLUE_LED 14
#define RELAY_PIN 12

OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  pinMode(MQ2_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  sensors.begin();
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  int gasValue = analogRead(MQ2_PIN);

  Serial.print("Temp: "); Serial.print(tempC);
  Serial.print(" Gas: "); Serial.println(gasValue);

  if (tempC < 40 && gasValue < 300) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
  } else if ((tempC >= 40 && tempC < 60) || (gasValue >= 300 && gasValue < 600)) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
  } else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(RELAY_PIN, HIGH);
  }

  delay(1000);
}
