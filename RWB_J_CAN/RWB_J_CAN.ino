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
// TOUCH (GT911)
// =====================================================
#define TP_SDA_PIN 8
#define TP_SCL_PIN 9

BBCapTouch touch;
TOUCHINFO touchInfo;

uint32_t lastTouchMs = 0;
const uint32_t TOUCH_DEBOUNCE = 150;

// =====================================================
// CAN
// =====================================================
bool can_ok = false;

const uint32_t ID_770     = 0x770;
const uint32_t ID_RECEIVE = 0x600;

// =====================================================
// Endian helpers
// =====================================================
uint16_t U16BE(const twai_message_t &m, int o) {
  return (uint16_t(m.data[o]) << 8) | m.data[o + 1];
}
int16_t S16BE(const twai_message_t &m, int o) {
  return (int16_t)U16BE(m, o);
}

// =====================================================
// ECU Data (simplified)
// =====================================================
struct ECUData {
  uint8_t raw[8];

  uint16_t w0, w1, w2, w3;
  int16_t  s0, s1, s2, s3;

  // displayed values
  uint16_t rpm;
  float coolant;
} ecu;

// simple smoothing state
uint16_t rpm_filtered = 0;
float coolant_filtered = 0.0f;

// =====================================================
// BUTTONS
// =====================================================
struct Button {
  int x, y, w, h;
  const char *label;
  bool state;
};

Button buttons[4] = {
  {  0, 400, 200, 80, "IN1", false },
  {200, 400, 200, 80, "IN2", false },
  {400, 400, 200, 80, "IN3", false },
  {600, 400, 200, 80, "IN4", false }
};

void drawButton(int i) {
  Button &b = buttons[i];
  uint16_t bg = b.state ? gfx->color565(0,200,0) : gfx->color565(50,50,50);

  gfx->fillRect(b.x, b.y, b.w, b.h, bg);
  gfx->drawRect(b.x, b.y, b.w, b.h, WHITE);

  gfx->setTextColor(WHITE, bg);
  gfx->setTextSize(2);
  gfx->setCursor(b.x + 20, b.y + 30);
  gfx->print(b.label);
}

void drawAllButtons() {
  for (int i = 0; i < 4; i++) drawButton(i);
}

// =====================================================
// CAN Debug Print
// =====================================================
void debugPrintCAN(const twai_message_t &m) {
  Serial.print("CAN RX ID=0x");
  Serial.print(m.identifier, HEX);
  Serial.print(" DLC=");
  Serial.print(m.data_length_code);
  Serial.print(" Data:");

  for (int i = 0; i < m.data_length_code; i++) {
    Serial.print(" ");
    if (m.data[i] < 0x10) Serial.print("0");
    Serial.print(m.data[i], HEX);
  }
  Serial.println();
}

// =====================================================
// CAN Send Bitfield (buttons)
// =====================================================
void sendButtonCAN() {
  if (!can_ok) return;

  uint8_t bits = 0;
  for (int i = 0; i < 4; i++)
    if (buttons[i].state) bits |= (1 << i);

  twai_message_t msg = {};
  msg.identifier = ID_RECEIVE;
  msg.data_length_code = 1;
  msg.data[0] = bits;

  twai_transmit(&msg, pdMS_TO_TICKS(20));

  Serial.print("TX Buttons = 0b");
  Serial.println(bits, BIN);
}

// =====================================================
// Touch
// =====================================================
void handleTouch() {
  int count = touch.getSamples(&touchInfo);
  if (count <= 0) return;

  uint16_t x = touchInfo.x[0];
  uint16_t y = touchInfo.y[0];

  if (x > 800 || y > 480) return;
  if (x == 0xFFFF || y == 0xFFFF) return;

  uint32_t now = millis();
  if (now - lastTouchMs < TOUCH_DEBOUNCE) return;
  lastTouchMs = now;

  if (y < 400) return;

  for (int i = 0; i < 4; i++) {
    Button &b = buttons[i];
    if (x >= b.x && x < b.x + b.w &&
        y >= b.y && y < b.y + b.h) {

      b.state = !b.state;
      drawButton(i);
      sendButtonCAN();
      break;
    }
  }
}

// =====================================================
// Decode 0x770 (stable version, ignore 0/0 frames)
// =====================================================
void decodeTrijekt(const twai_message_t &m) {
  if (m.identifier != ID_770) return;

  // Save raw bytes
  for (int i = 0; i < 8; i++)
    ecu.raw[i] = m.data[i];

  // Unsigned words
  ecu.w0 = U16BE(m, 0);
  ecu.w1 = U16BE(m, 2);
  ecu.w2 = U16BE(m, 4);
  ecu.w3 = U16BE(m, 6);

  // Signed words
  ecu.s0 = S16BE(m, 0);
  ecu.s1 = S16BE(m, 2);
  ecu.s2 = S16BE(m, 4);
  ecu.s3 = S16BE(m, 6);

  // ----------------------------------------
  // Filter out sentinel / nonsense frames
  // ----------------------------------------

  // rpm (w0) must be non-negative and reasonable
  if (ecu.s0 < 0 || ecu.w0 > 10000) {
    Serial.println("  -> rpm invalid, frame ignored");
    return;
  }

  // coolant plausibly between -40.0°C and 150.0°C
  float tempC = ecu.s1 * 0.1f;
  if (tempC < -40.0f || tempC > 150.0f) {
    Serial.println("  -> coolant invalid, frame ignored");
    return;
  }

  // Ignore "page" where rpm & coolant are both 0 (the 00 00 00 00 ... 41 1E FC frames)
  if (ecu.w0 == 0 && ecu.w1 == 0) {
    Serial.println("  -> 0/0 page (no rpm/coolant), ignored");
    return;
  }

  // At this point we consider the frame "good" for rpm & coolant
  uint16_t new_rpm = ecu.w0;
  float new_coolant = tempC;

  // ----------------------------------------
  // Simple smoothing (low-pass)
  // ----------------------------------------
  static bool first = true;
  if (first) {
    rpm_filtered = new_rpm;
    coolant_filtered = new_coolant;
    first = false;
  } else {
    rpm_filtered     = (uint16_t)((rpm_filtered * 3 + new_rpm) / 4);
    coolant_filtered = (coolant_filtered * 3.0f + new_coolant) / 4.0f;
  }

  ecu.rpm     = rpm_filtered;
  ecu.coolant = coolant_filtered;

  // Debug
  Serial.print("GOOD frame -> raw rpm=");
  Serial.print(new_rpm);
  Serial.print(" rawCool=");
  Serial.print(new_coolant, 1);
  Serial.print("   | filt rpm=");
  Serial.print(ecu.rpm);
  Serial.print(" filtCool=");
  Serial.println(ecu.coolant, 1);
}

// =====================================================
// UI
// =====================================================
void drawStaticUI() {
  gfx->fillScreen(BLACK);

  gfx->setTextColor(WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(20,20);
  gfx->print("RWB CAN 0x770 STABLE");

  int y = 100, dy = 35;
  gfx->setTextSize(2);

  gfx->setCursor(20,y); gfx->print("RPM:");
  gfx->setCursor(420,y); gfx->print("Coolant:"); y+=dy;

  gfx->setCursor(20,y); gfx->print("Raw[0..3]:");
  gfx->setCursor(420,y); gfx->print("Raw[4..7]:"); y+=dy;

  drawAllButtons();
}

void updateDashboard() {
  int y = 100;
  int dy = 35;
  gfx->setTextColor(WHITE, BLACK);
  gfx->setTextSize(2);

  // RPM + Coolant
  gfx->setCursor(150,y);
  gfx->printf("%5u rpm", ecu.rpm);
  gfx->setCursor(520,y);
  gfx->printf("%5.1f C", ecu.coolant);
  y+=dy;

  // Raw bytes [0..3]
  gfx->setCursor(150,y);
  gfx->printf("%02X %02X %02X %02X",
              ecu.raw[0], ecu.raw[1], ecu.raw[2], ecu.raw[3]);
  // Raw bytes [4..7]
  gfx->setCursor(520,y);
  gfx->printf("%02X %02X %02X %02X",
              ecu.raw[4], ecu.raw[5], ecu.raw[6], ecu.raw[7]);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  gfx->begin();
  drawStaticUI();

  Wire.begin(TP_SDA_PIN, TP_SCL_PIN);
  int r = touch.init(TP_SDA_PIN, TP_SCL_PIN, -1, -1);
  if (r == 0) {
    touch.setOrientation(0, 800, 480);
  }

  can_ok = waveshare_twai_init();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static uint32_t lastDash = 0;

  if (can_ok) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
      debugPrintCAN(m);
      decodeTrijekt(m);
    }
  }

  if (millis() - lastDash > 100) {
    updateDashboard();
    lastDash = millis();
  }

  handleTouch();
}
