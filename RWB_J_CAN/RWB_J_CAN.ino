#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "waveshare_twai_port.h"
#include <bb_captouch.h>

// =====================================================
// DISPLAY (Waveshare ESP32-S3-Touch-LCD-4.3B)
// =====================================================
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    5, 3, 46, 7,
    1, 2, 42, 41, 40,
    39, 0, 45, 48, 47, 21,
    14, 38, 18, 17, 10,
    0, 40, 48, 88,
    0, 13, 3, 32,
    1, 16000000
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

// =====================================================
// TOUCH
// =====================================================
#define TP_SDA_PIN 8
#define TP_SCL_PIN 9

BBCapTouch touch;

// =====================================================
// CAN — Trijekt Protocol Type 2
// =====================================================
bool can_ok = false;

const uint32_t ID_RPM     = 0x770;
const uint32_t ID_COOLANT = 0x772;

// Scaling factors
const float RPM_FACTOR     = 1.23f;   // raw * 1.23 = real RPM
const float COOLANT_FACTOR = 0.1f;

uint16_t rpm_filtered = 0;
float coolant_filtered = 0.0f;

const uint16_t MAX_RPM = 8000;   // full bar at 8000 rpm

// =====================================================
// HELPERS
// =====================================================
uint16_t U16BE(const twai_message_t &m, int o) {
  return (uint16_t(m.data[o]) << 8) | m.data[o+1];
}

// =====================================================
// ECU DATA
// =====================================================
struct ECUData {
  uint16_t rpm;
  float coolant;
} ecu;

// =====================================================
// UI – Background
// =====================================================
void drawUI() {
  gfx->fillScreen(BLACK);

  gfx->setTextColor(WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(50,20);
  gfx->print("Trijekt Dashboard - Protocol 2");

  // RPM BAR FRAME
  int gx = 100, gy = 120, gw = 600, gh = 40;
  gfx->drawRect(gx, gy, gw, gh, WHITE);

  // Title text
  gfx->setCursor(100, 90);
  gfx->print("RPM Bar:");

  gfx->setCursor(100, 200);
  gfx->print("RPM:");

  gfx->setCursor(450, 200);
  gfx->print("Coolant:");
}

// =====================================================
// Draw RPM Bar
// =====================================================
void drawRPMBar(uint16_t rpm) {
  int gx = 100, gy = 120, gw = 600, gh = 40;

  // Clear bar inside
  gfx->fillRect(gx+2, gy+2, gw-4, gh-4, BLACK);

  float frac = (float)rpm / MAX_RPM;
  if (frac > 1) frac = 1;

  int fw = frac * (gw - 4);

  for (int x=0; x<fw; x++) {
    float t = float(x) / (gw - 4);

    uint16_t c =
      (t < 0.65f) ? gfx->color565(0,220,0) :      // green
      (t < 0.85f) ? gfx->color565(255,215,0) :    // yellow
                    gfx->color565(255,0,0);       // red

    gfx->drawFastVLine(gx + 2 + x, gy + 2, gh - 4, c);
  }
}

// =====================================================
// Update text values
// =====================================================
void updateText() {
  // RPM
  gfx->fillRect(170,190,150,40,BLACK);
  gfx->setCursor(170,200);
  gfx->setTextColor(WHITE,BLACK);
  gfx->setTextSize(3);
  gfx->printf("%5u", ecu.rpm);

  // Coolant
  gfx->fillRect(620,190,150,40,BLACK);
  gfx->setCursor(620,200);
  gfx->setTextColor(WHITE,BLACK);
  gfx->printf("%5.1f C", ecu.coolant);
}

// =====================================================
// CAN decode
// =====================================================
void decodeTrijekt(const twai_message_t &m) {

  // ----- RPM -----
  if (m.identifier == ID_RPM && m.data_length_code >= 2) {
    uint16_t raw = U16BE(m, 0);
    float rpmf = raw * RPM_FACTOR;
    uint16_t rpm = (uint16_t)rpmf;

    if (rpm_filtered == 0)
      rpm_filtered = rpm;
    else
      rpm_filtered = (rpm_filtered * 3 + rpm) / 4;

    ecu.rpm = rpm_filtered;

    Serial.printf("RPM raw=%u -> %.1f filtered=%u\n",
                   raw, rpmf, ecu.rpm);
  }

  // ----- Coolant -----
  if (m.identifier == ID_COOLANT && m.data_length_code >= 2) {

    int16_t raw = (int16_t)U16BE(m,0);
    float temp = raw * COOLANT_FACTOR;

    coolant_filtered = (coolant_filtered * 3 + temp) / 4.0f;
    ecu.coolant = coolant_filtered;

    Serial.printf("Coolant raw=%d -> %.1fC\n", raw, ecu.coolant);
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  gfx->begin();
  drawUI();

  Wire.begin(TP_SDA_PIN, TP_SCL_PIN);
  touch.init(TP_SDA_PIN, TP_SCL_PIN, -1, -1);

  can_ok = waveshare_twai_init();
  Serial.printf("CAN init: %s\n", can_ok ? "OK" : "FAIL");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static uint32_t lastUI = 0;

  if (can_ok) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {

      Serial.printf("CAN RX ID=0x%03X Data:", m.identifier);
      for (int i=0;i<m.data_length_code;i++)
        Serial.printf(" %02X", m.data[i]);
      Serial.println();

      decodeTrijekt(m);
    }
  }

  if (millis() - lastUI > 100) {
    drawRPMBar(ecu.rpm);
    updateText();
    lastUI = millis();
  }
}
