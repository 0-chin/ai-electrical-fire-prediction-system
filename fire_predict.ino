/*
 * ================================================================
 *   AI-Based Embedded Electrical Fire Prediction System
 *   Microcontroller : ESP32-S3-WROOM-1-N4
 *   IDE             : Arduino IDE 2.x
 *   Board package   : esp32 by Espressif (v3.x)
 * ================================================================
 *  SENSORS
 *   IO1  → ACS712-20A   (current,  analog)
 *   IO2  → ZMPT101B     (voltage,  analog)
 *   IO3  → MQ-2 AOUT    (gas,      analog)
 *   IO4  → DS18B20      (temp,     1-Wire)
 *   IO5  → MQ-2 DOUT    (gas alert,digital)
 *
 *  OUTPUTS
 *   IO10 → Relay        (load cut)
 *   IO11 → Buzzer
 *   IO12 → RGB LED Red
 *   IO13 → RGB LED Green
 *   IO14 → RGB LED Blue
 * ================================================================
 *  RISK LEVELS
 *   SAFE    → Green LED,  silent,     relay ON
 *   WARNING → Amber LED,  slow beep,  relay ON
 *   DANGER  → Red LED,    fast beep,  relay OFF  ← load cut
 * ================================================================
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// ── Pin definitions ───────────────────────────────────────────
#define PIN_CURRENT     1
#define PIN_VOLTAGE     2
#define PIN_GAS_ANALOG  3
#define PIN_ONEWIRE     4
#define PIN_GAS_DIGITAL 5

#define PIN_RELAY       10
#define PIN_BUZZER      11
#define PIN_LED_R       12
#define PIN_LED_G       13
#define PIN_LED_B       14

// ── Thresholds ────────────────────────────────────────────────
#define TEMP_WARN       55.0    // °C
#define TEMP_DANGER     75.0    // °C
#define CURR_WARN       12.0    // Amps
#define CURR_DANGER     18.0    // Amps
#define VOLT_LOW        180.0   // VAC — under voltage
#define VOLT_HIGH       260.0   // VAC — over voltage
#define GAS_WARN        400     // ADC raw (0–4095)
#define GAS_DANGER      700     // ADC raw

// ── DS18B20 setup ─────────────────────────────────────────────
OneWire           oneWire(PIN_ONEWIRE);
DallasTemperature tempSensor(&oneWire);

// ── Risk level ────────────────────────────────────────────────
enum Risk { SAFE, WARNING, DANGER };
Risk systemRisk = SAFE;

// ── Buzzer non-blocking timing ────────────────────────────────
unsigned long lastBuzzerMs  = 0;
bool          buzzerOn      = false;

// ── Sensor sampling interval ──────────────────────────────────
unsigned long lastSampleMs  = 0;
const unsigned long SAMPLE_INTERVAL = 1000;  // 1 second

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_GAS_DIGITAL, INPUT);
  pinMode(PIN_RELAY,  OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R,  OUTPUT);
  pinMode(PIN_LED_G,  OUTPUT);
  pinMode(PIN_LED_B,  OUTPUT);

  // Safe startup state
  digitalWrite(PIN_RELAY,  HIGH);   // relay ON = load connected
  digitalWrite(PIN_BUZZER, LOW);
  setRGB(0, 255, 0);                // green = SAFE

  tempSensor.begin();

  Serial.println("================================================");
  Serial.println("  Fire Prediction System  —  STARTED");
  Serial.println("================================================");
  Serial.println("Temp(C) | Current(A) | Voltage(V) | Gas  | Risk");
  Serial.println("--------|------------|------------|------|------");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // Read and evaluate sensors every 1 second
  if (now - lastSampleMs >= SAMPLE_INTERVAL) {
    lastSampleMs = now;

    float temp    = readTemperature();
    float current = readCurrent();
    float voltage = readVoltage();
    int   gas     = readGas();
    bool  gasAlert = digitalRead(PIN_GAS_DIGITAL);

    systemRisk = evaluateRisk(temp, current, voltage, gas, gasAlert);
    applyOutputs(systemRisk);
    printReport(temp, current, voltage, gas, gasAlert);
  }

  // Handle buzzer beep pattern non-blocking
  handleBuzzer(millis());
}

// ─────────────────────────────────────────────────────────────
//  SENSOR FUNCTIONS
// ─────────────────────────────────────────────────────────────

float readTemperature() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) return 0.0;
  return t;
}

// ACS712-20A: Vout = 2.5V at 0A, sensitivity = 100mV/A
// 4kΩ + 3kΩ divider scales 5V → 3.3V max at ADC
float readCurrent() {
  int   raw     = analogRead(PIN_CURRENT);
  float vADC    = (raw / 4095.0) * 3.3;
  float vSensor = vADC * (7.0 / 3.0);        // undo voltage divider
  float amps    = (vSensor - 2.5) / 0.1;     // ACS712 formula
  return (amps < 0) ? 0 : amps;
}

// ZMPT101B: 0–5V output = 0–300VAC
// 4kΩ + 3kΩ divider scales sensor output to safe ADC range
float readVoltage() {
  int   raw     = analogRead(PIN_VOLTAGE);
  float vADC    = (raw / 4095.0) * 3.3;
  float vSensor = vADC * (7.0 / 3.0);        // undo voltage divider
  float vAC     = vSensor * (300.0 / 5.0);   // scale to AC volts
  return vAC;
}

// MQ-2: higher raw value = more gas/smoke
int readGas() {
  return analogRead(PIN_GAS_ANALOG);
}

// ─────────────────────────────────────────────────────────────
//  RISK EVALUATION
// ─────────────────────────────────────────────────────────────
Risk evaluateRisk(float t, float c, float v, int g, bool gasAlert) {
  Risk r = SAFE;

  // Temperature
  if      (t >= TEMP_DANGER) r = riskMax(r, DANGER);
  else if (t >= TEMP_WARN)   r = riskMax(r, WARNING);

  // Current / overload
  if      (c >= CURR_DANGER) r = riskMax(r, DANGER);
  else if (c >= CURR_WARN)   r = riskMax(r, WARNING);

  // Voltage — only evaluate if sensor is connected (v > 0)
  if (v > 10) {
    if (v > VOLT_HIGH || v < VOLT_LOW) r = riskMax(r, WARNING);
  }

  // Gas / smoke
  if      (gasAlert || g >= GAS_DANGER) r = riskMax(r, DANGER);
  else if (g >= GAS_WARN)               r = riskMax(r, WARNING);

  return r;
}

Risk riskMax(Risk a, Risk b) {
  return (b > a) ? b : a;
}

// ─────────────────────────────────────────────────────────────
//  OUTPUT CONTROL
// ─────────────────────────────────────────────────────────────
void applyOutputs(Risk r) {
  switch (r) {
    case SAFE:
      digitalWrite(PIN_RELAY, HIGH);  // load ON
      setRGB(0, 255, 0);              // green
      break;

    case WARNING:
      digitalWrite(PIN_RELAY, HIGH);  // load still ON
      setRGB(255, 140, 0);            // amber
      break;

    case DANGER:
      digitalWrite(PIN_RELAY, LOW);   // CUT the load
      setRGB(255, 0, 0);              // red
      break;
  }
}

// Non-blocking buzzer — fast beep on DANGER, slow on WARNING
void handleBuzzer(unsigned long now) {
  unsigned long interval = 0;

  if      (systemRisk == DANGER)  interval = 200;
  else if (systemRisk == WARNING) interval = 800;
  else {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn = false;
    return;
  }

  if (now - lastBuzzerMs >= interval) {
    lastBuzzerMs = now;
    buzzerOn = !buzzerOn;
    digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
  }
}

// Set RGB LED color (0–255 per channel)
void setRGB(int r, int g, int b) {
  analogWrite(PIN_LED_R, r);
  analogWrite(PIN_LED_G, g);
  analogWrite(PIN_LED_B, b);
}

// ─────────────────────────────────────────────────────────────
//  SERIAL MONITOR REPORT
// ─────────────────────────────────────────────────────────────
void printReport(float t, float c, float v, int g, bool gasAlert) {
  const char* rLabel[] = { "SAFE   ", "WARNING", "DANGER " };
  const char* rIcon[]  = { "[OK]   ", "[WARN] ", "[FIRE!]" };

  Serial.printf("%-7.1f | %-10.2f | %-10.1f | %-4d | %s %s\n",
    t, c, v, g, rLabel[systemRisk], rIcon[systemRisk]);

  if (systemRisk == DANGER) {
    Serial.println("  *** HAZARD DETECTED — RELAY TRIPPED — LOAD CUT ***");
    if (t >= TEMP_DANGER)
      Serial.printf("  > Temp    : %.1f C  (limit %.0f C)\n", t, TEMP_DANGER);
    if (c >= CURR_DANGER)
      Serial.printf("  > Current : %.2f A  (limit %.0f A)\n", c, CURR_DANGER);
    if (g >= GAS_DANGER)
      Serial.printf("  > Gas ADC : %d      (limit %d)\n",     g, GAS_DANGER);
    if (gasAlert)
      Serial.println("  > Gas digital threshold triggered");
    Serial.println("  ------------------------------------------------");
  }

  if (systemRisk == WARNING) {
    Serial.println("  [!] Approaching threshold — monitor closely");
  }
}
