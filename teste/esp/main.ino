#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <ESP32Encoder.h>

// === Config iguais ao seu header ===
#define PWM_BASE_FREQ   19000
#define PWM_RESOLUTION  8      // 0..255
#define PWM_CH          1

#define PWM         19
#define BRAKE       18
#define DIR         23
#define ENCODER_CHA 13
#define ENCODER_CHB 5
#define ENCODER_PPR 400

#define SERIAL_BAUD     115200
#define READ_TIMEOUT_MS 20

ESP32Encoder motor_enc;

uint8_t duty = 200;   // 255=parado, 0=rápido (inverso)
uint8_t pwm_val = 0;

void applyDuty(uint8_t d) {
  duty = d;
  pwm_val = 255 - duty;              // inverso Nidec
  ledcWriteChannel(PWM_CH, pwm_val);
}

void setup() {
#if !defined(ARDUINO_ARCH_ESP32)
#error "Selecione uma placa ESP32 (ESP32 Arduino)."
#endif
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(READ_TIMEOUT_MS);

  pinMode(DIR, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(BRAKE, HIGH); // libera motor

  #ifdef LEDC_AUTO_CLK
  ledcSetClockSource(LEDC_AUTO_CLK);
  #endif
  ledcAttachChannel(PWM, PWM_BASE_FREQ, PWM_RESOLUTION, PWM_CH);
  applyDuty(duty);

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  ESP32Encoder::isrServiceCpuCore = 0;
  motor_enc.attachFullQuad(ENCODER_CHA, ENCODER_CHB);
  motor_enc.clearCount();

  Serial.println("READY");
}

void loop() {
  // ---- comandos: U<0..255>\n ----
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() && (s.charAt(0)=='U' || s.charAt(0)=='u')) {
      int v = s.substring(1).toInt();
      if (v < 0) v = 0; if (v > 255) v = 255;
      applyDuty((uint8_t)v);
      Serial.printf("OK U=%d\n", v);
    }
  }

  // ---- Telemetria: RPM:<valor>\n a cada 50 ms ----
  static uint32_t last_ms = 0;
  static long long last_count = 0;
  uint32_t now = millis();
  if (now - last_ms >= 50) { // 20 Hz
    uint32_t dt = now - last_ms;
    last_ms = now;

    long long count = motor_enc.getCount();
    long long dcount = count - last_count;
    last_count = count;

    // RPM: dcount pulsos em dt ms -> rev = dcount/PPR; RPM = rev * 60000/dt
    float rpm = (dt > 0) ? ((float)dcount / (float)ENCODER_PPR) * (60000.0f / (float)dt) : 0.0f;
    Serial.printf("RPM:%.2f\n", rpm);
  }
}
