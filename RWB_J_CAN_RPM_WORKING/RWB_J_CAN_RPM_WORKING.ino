#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "waveshare_twai_port.h"
#include <bb_captouch.h>

// =====================================================
// DISPLAY (Waveshare ESP32-S3 4.3" 800×480 RGB LCD)
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
TOUCHINFO touchInfo;

// =====================================================
// CONSTS FOR SCREEN
// =====================================================
const int SCREEN_W = 800;
const int SCREEN_H = 480;

// =====================================================
// CAN — Trijekt Premium Protocol
// =====================================================
bool can_ok = false;

// ----- BASE IDs (ADJUST THESE TO YOUR ECU SETUP!) -----
const uint32_t ID_A        = 0x770;          // example base A
const uint32_t ID_B        = 0x780;          // example base B

// ----- Derived IDs from documentation -----
const uint32_t ID_RPM          = ID_A;           // RPM frame
const uint32_t ID_STATUS_B     = ID_A + 0x01;    // Drosselklappe / Gaspedalstellung
const uint32_t ID_TEMP_A       = ID_A + 0x02;    // Temperaturen A (Motor temp)
const uint32_t ID_BATT_VOLT    = ID_B + 0x01;    // Analoge Eingänge A (Batt, bytes 3:4)

// ***** NEW: Funktionseingang status over CAN *****
const uint32_t ID_FUNC_STATUS  = ID_A + 0x03;    // <--- PLACEHOLDER, CHANGE TO REAL ID
const uint8_t  FUNC_BYTE_INDEX = 0;             // which data byte (0–7) has the bit
const uint8_t  FUNC_PIN38_MASK = 0x01;          // which bit in that byte is pin 38 (0x01..0x80)

// Scaling factors from Trijekt docs
const float RPM_FACTOR      = 1.23f;   // example: raw * 1.23 = RPM
const float MOTOR_FACTOR    = 0.1f;    // 0.1 °C
const float BATT_FACTOR     = 0.001f;  // 1 mV = 0.001 V
const float THROTTLE_FACTOR = 0.1f;    // 0.1 %

// Filters
uint16_t rpm_filtered             = 0;
float batt_filtered               = 0.0f;
float motor_filtered              = 0.0f;
float throttle_dk_filtered        = 0.0f;
float throttle_pedal_filtered     = 0.0f;

const uint16_t MAX_RPM = 8000;
uint16_t rpm_max = 0;

// RPM timeout: if no frame for this long -> rpm = 0
const uint32_t RPM_TIMEOUT_MS = 500;
uint32_t lastRpmUpdateMs = 0;

// Funktionseingang status (from CAN)
bool funk_active = false;

// =====================================================
// THEME
// =====================================================
uint16_t bgColor = BLACK;
uint16_t fgColor = WHITE;

// =====================================================
// HELPERS
// =====================================================
uint16_t U16BE(const twai_message_t &m, int o) {
  return (uint16_t(m.data[o]) << 8) | m.data[o+1];
}

bool pointInRect(int16_t x, int16_t y, int16_t rx, int16_t ry, int16_t rw, int16_t rh) {
  return (x >= rx && x < (rx + rw) && y >= ry && y < (ry + rh));
}

// =====================================================
// ECU DATA
// =====================================================
struct ECUData {
  uint16_t rpm;
  float batt;            // battery voltage
  float motor;           // motor temperature
  float throttle_dk;     // Drosselklappe (%)
  float throttle_pedal;  // Gaspedalstellung (%)
} ecu;

// =====================================================
// BUTTON POSITIONS
// =====================================================
int BTN_THEME_W = 220;
int BTN_THEME_H = 60;
int BTN_SPACE   = 60;

// *** MOVED TO THE RIGHT ***
int BTN_THEME_X = SCREEN_W - (2 * BTN_THEME_W + BTN_SPACE) - 40;
int BTN_THEME_Y = SCREEN_H - 80;

int BTN_RESET_X = BTN_THEME_X + BTN_THEME_W + BTN_SPACE;
int BTN_RESET_Y = BTN_THEME_Y;

// =====================================================
// BUTTON DRAW
// =====================================================
void drawButton(int x, int y, int w, int h, const char *label) {
  uint16_t fill = gfx->color565(40, 40, 40);
  gfx->fillRect(x, y, w, h, fill);
  gfx->drawRect(x, y, w, h, fgColor);

  gfx->setTextSize(2);
  gfx->setTextColor(fgColor, fill);
  gfx->setCursor(x + 15, y + h/2 - 8);
  gfx->print(label);
}

void toggleTheme() {
  if (bgColor == BLACK) {
    bgColor = gfx->color565(10, 15, 40);
    fgColor = gfx->color565(230, 230, 255);
  } else {
    bgColor = BLACK;
    fgColor = WHITE;
  }
}

// =====================================================
// FUNKTIONSEINGANG STATUS INDICATOR
// =====================================================
void drawFunkButton(bool active) {
  // Center of the circle, near bottom-left, aligned near buttons
  int r  = 20;
  int cx = 80;                 // X center of circle
  int cy = SCREEN_H - 50;      // Y center of circle (a bit above bottom)

  // Clear a rectangle that fully covers circle + text
  int clearX = cx - r - 10;
  int clearY = cy - r - 10;
  int clearW = 200;            // enough for circle + "FuncPin 38"
  int clearH = r*2 + 20;
  gfx->fillRect(clearX, clearY, clearW, clearH, bgColor);

  // Draw circle (red or green)
  uint16_t color = active ? gfx->color565(0, 200, 0)   // green
                          : gfx->color565(220, 0, 0);  // red

  gfx->fillCircle(cx, cy, r, color);
  gfx->drawCircle(cx, cy, r, fgColor);

  // Label
  gfx->setTextSize(2);
  gfx->setTextColor(fgColor, bgColor);
  gfx->setCursor(cx + r + 10, cy - 7);
  gfx->print("FuncPin 38");
}

// =====================================================
// UI – FULL SCREEN LAYOUT
// =====================================================
void drawUI() {
  gfx->fillScreen(bgColor);
  gfx->setTextColor(fgColor, bgColor);

  // Header centered
  gfx->setTextSize(3);
  const char *title = "RWB JANINE DASH";
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds((char*)title, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor((SCREEN_W - w) / 2, 20);
  gfx->print(title);

  // Max RPM label
  gfx->setTextSize(2);
  gfx->setCursor(SCREEN_W - 200, 20);
  gfx->print("Max RPM:");

  // RPM bar frame
  int gx = 60, gy = 110, gw = SCREEN_W - 120, gh = 60;
  gfx->setTextSize(3);
  gfx->setCursor(gx, gy - 40);
  gfx->print("RPM");

  gfx->drawRect(gx, gy, gw, gh, fgColor);

  // scale ticks
  gfx->setTextSize(2);
  for (int rpm = 0; rpm <= MAX_RPM; rpm += 1000) {
    float frac = (float)rpm / MAX_RPM;
    int x = gx + 2 + frac * (gw - 4);
    gfx->drawFastVLine(x, gy + gh + 2, 10, fgColor);
    gfx->setCursor(x - 8, gy + gh + 18);
    gfx->print(rpm / 1000);
  }
  gfx->setCursor(gx + gw - 400, gy - 20);
  gfx->print("x1000 rpm");

  // Big RPM value area
  gfx->fillRect(gx + 160, gy + gh + 50, 260, 60, bgColor);

  // Boxes (Batt / Motor / Throttle)
  int cardTop = 260 + 20;  // moved 20px down
  int cardHeight = 90;
  int cardMargin = 20;
  int cardWidth = (SCREEN_W - 4 * cardMargin) / 3;

  // Box 1: Battery Voltage
  int c1x = cardMargin;
  gfx->drawRoundRect(c1x, cardTop, cardWidth, cardHeight, 12, fgColor);
  gfx->setCursor(c1x + 10, cardTop + 10);
  gfx->print("Batt V");

  // Box 2: Motor Temp
  int c2x = c1x + cardWidth + cardMargin;
  gfx->drawRoundRect(c2x, cardTop, cardWidth, cardHeight, 12, fgColor);
  gfx->setCursor(c2x + 10, cardTop + 10);
  gfx->print("Motor");

  // Box 3: Throttle (Drossel + Pedal)
  int c3x = c2x + cardWidth + cardMargin;
  gfx->drawRoundRect(c3x, cardTop, cardWidth, cardHeight, 12, fgColor);
  gfx->setCursor(c3x + 10, cardTop + 10);
  gfx->print("Throttle");

  // Buttons (now right-aligned)
  drawButton(BTN_THEME_X, BTN_THEME_Y, BTN_THEME_W, BTN_THEME_H, "Touch 1");
  drawButton(BTN_RESET_X, BTN_RESET_Y, BTN_THEME_W, BTN_THEME_H, "Touch 2");

  // Funktionseingang indicator (initial draw)
  drawFunkButton(funk_active);
}

// =====================================================
// RPM BAR with minimal redraw (less flicker)
// =====================================================
void drawRPMBar(uint16_t rpm) {
  static int lastFw = 0;  // last bar width in pixels

  int gx = 60, gy = 110, gw = SCREEN_W - 120, gh = 60;

  float frac = (float)rpm / MAX_RPM;
  if (frac > 1) frac = 1;
  int fw = frac * (gw - 4);  // new bar width

  if (fw > lastFw) {
    // RPM increased: draw ONLY the new part
    for (int x = lastFw; x < fw; x++) {
      float t = (float)x / (gw - 4);
      uint16_t c =
        (t < 0.65f) ? gfx->color565(0, 220, 0) :
        (t < 0.85f) ? gfx->color565(255, 215, 0) :
                      gfx->color565(255, 0, 0);
      gfx->drawFastVLine(gx + 2 + x, gy + 2, gh - 4, c);
    }
  } else if (fw < lastFw) {
    // RPM decreased: clear ONLY the now-unused part
    gfx->fillRect(gx + 2 + fw, gy + 2, (lastFw - fw), gh - 4, bgColor);
  }

  lastFw = fw;
}

// =====================================================
// UPDATE TEXT VALUES
// =====================================================
void updateText() {
  gfx->setTextColor(fgColor, bgColor);

  // Max RPM
  gfx->fillRect(SCREEN_W - 100, 15, 90, 25, bgColor);
  gfx->setCursor(SCREEN_W - 100, 18);
  gfx->printf("%5u", rpm_max);

  // Big RPM
  int gx = 60, gy = 110, gh = 60;
  gfx->fillRect(gx + 160, gy + gh + 50, 260, 60, bgColor);
  gfx->setTextSize(5);
  gfx->setCursor(gx + 160, gy + gh + 55);
  gfx->printf("%5u", ecu.rpm);

  // Boxes: Batt / Motor / Throttle
  int cardTop = 260 + 20;
  int cardMargin = 20;
  int cardWidth = (SCREEN_W - 4 * cardMargin) / 3;
  int valY = cardTop + 40;

  int c1x = cardMargin;
  int c2x = c1x + cardWidth + cardMargin;
  int c3x = c2x + cardWidth + cardMargin;

  // Battery Voltage
  gfx->setTextSize(3);
  gfx->fillRect(c1x + 10, valY, cardWidth - 20, 40, bgColor);
  gfx->setCursor(c1x + 10, valY + 5);
  gfx->printf("%4.1f V", ecu.batt);

  // Motor Temp
  gfx->fillRect(c2x + 10, valY, cardWidth - 20, 40, bgColor);
  gfx->setCursor(c2x + 10, valY + 5);
  gfx->printf("%5.1f C", ecu.motor);

  // Throttle box: show BOTH DK and GP with markers
  gfx->fillRect(c3x + 10, valY, cardWidth - 20, 40, bgColor);
  gfx->setTextSize(2);
  gfx->setCursor(c3x + 10, valY);        // first line
  gfx->printf("DK %4.1f%%", ecu.throttle_dk);
  gfx->setCursor(c3x + 10, valY + 22);   // second line
  gfx->printf("GP %4.1f%%", ecu.throttle_pedal);
}

// =====================================================
// CAN DECODE
// =====================================================
void decodeTrijekt(const twai_message_t &m) {

  // RPM (ID_A)
  if (m.identifier == ID_RPM && m.data_length_code >= 2) {
    uint16_t raw = U16BE(m, 0);
    uint16_t rpm = raw * RPM_FACTOR;

    rpm_filtered = (rpm_filtered * 3 + rpm) / 4;
    ecu.rpm = rpm_filtered;

    if (ecu.rpm > rpm_max) rpm_max = ecu.rpm;

    // update RPM timestamp for timeout
    lastRpmUpdateMs = millis();
  }

  // Motor Temperature (Temperaturen A, bytes 1:2)
  if (m.identifier == ID_TEMP_A && m.data_length_code >= 2) {
    int16_t raw = (int16_t)U16BE(m, 0); // bytes 1:2
    float temp = raw * MOTOR_FACTOR;
    motor_filtered = (motor_filtered * 3 + temp) / 4.0f;
    ecu.motor = motor_filtered;
  }

  // Battery Voltage (Analoge Eingänge A, bytes 3:4)
  if (m.identifier == ID_BATT_VOLT && m.data_length_code >= 4) {
    int16_t raw = (int16_t)U16BE(m, 2);  // bytes 3:4
    float v = raw * BATT_FACTOR;
    batt_filtered = (batt_filtered * 3 + v) / 4.0f;
    ecu.batt = batt_filtered;
  }

  // Statusdaten B: Drosselklappe (1:2) + Gaspedalstellung (3:4)
  if (m.identifier == ID_STATUS_B && m.data_length_code >= 4) {
    // Drosselklappe
    int16_t raw_dk = (int16_t)U16BE(m, 0);  // bytes 1:2
    float dk = raw_dk * THROTTLE_FACTOR;

    // Gaspedalstellung
    int16_t raw_gp = (int16_t)U16BE(m, 2);  // bytes 3:4
    float gp = raw_gp * THROTTLE_FACTOR;

    throttle_dk_filtered     = (throttle_dk_filtered * 3 + dk) / 4.0f;
    throttle_pedal_filtered  = (throttle_pedal_filtered * 3 + gp) / 4.0f;

    ecu.throttle_dk     = throttle_dk_filtered;
    ecu.throttle_pedal  = throttle_pedal_filtered;
  }

  // ***** Funktionseingang status frame *****
  if (m.identifier == ID_FUNC_STATUS && m.data_length_code > FUNC_BYTE_INDEX) {
    uint8_t byteVal = m.data[FUNC_BYTE_INDEX];
    bool newState = (byteVal & FUNC_PIN38_MASK) != 0;

    if (newState != funk_active) {
      funk_active = newState;
      drawFunkButton(funk_active);
    }
  }
}

// =====================================================
// TOUCH
// =====================================================
void handleTouchPress(int16_t x, int16_t y) {

  if (pointInRect(x, y, BTN_THEME_X, BTN_THEME_Y, BTN_THEME_W, BTN_THEME_H)) {
    toggleTheme();
    drawUI();
  }

  if (pointInRect(x, y, BTN_RESET_X, BTN_RESET_Y, BTN_THEME_W, BTN_THEME_H)) {
    rpm_max = 0;
    drawUI();
  }
}

void handleTouch() {
  int n = touch.getSamples(&touchInfo);
  if (n > 0 && touchInfo.count > 0) {
    handleTouchPress(touchInfo.x[0], touchInfo.y[0]);
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  gfx->begin();
  gfx->setTextColor(fgColor, bgColor);
  drawUI();

  Wire.begin(TP_SDA_PIN, TP_SCL_PIN);
  touch.init(TP_SDA_PIN, TP_SCL_PIN, -1, -1);

  can_ok = waveshare_twai_init();

  // Start with RPM timeout base
  lastRpmUpdateMs = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static uint32_t lastUI = 0;

  if (can_ok) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
      decodeTrijekt(m);
    }
  }

  // If no RPM frame recently, force RPM to 0
  if (millis() - lastRpmUpdateMs > RPM_TIMEOUT_MS && ecu.rpm != 0) {
    ecu.rpm = 0;
    rpm_filtered = 0;
  }

  handleTouch();

  if (millis() - lastUI > 100) {
    drawRPMBar(ecu.rpm);
    updateText();
    lastUI = millis();
  }
}
