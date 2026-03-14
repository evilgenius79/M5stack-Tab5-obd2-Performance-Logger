// ================================================================
//  OBD2 Performance Logger  —  M5Stack Tab5
//  ESP32-P4 | TWAI CAN | SDIO SD | GPS 10Hz
//  v3: Light tree, reaction time, peak boost, trap speed,
//      settings screen, screen dimming, GPS stale-speed fix
// ================================================================

#include <M5Unified.h>
#include <M5GFX.h>
#include <SD_MMC.h>
#include "driver/twai.h"
#include <TinyGPS++.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ----------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------
void     configureGPS10Hz();
void     setupCAN();
void     setupOBD2();
float    getOBD2Data(uint8_t pid);
void     drawBaseUI();
void     drawGaugeFace();
void     drawLeftPanelLabels();
void     drawRightPanelLabels();
void     drawStatusBar();
void     drawReadyBanner();
void     drawStagedBanner();
void     drawLaunchBanner();
void     drawFinishBanner();
void     drawSplashBG();
void     drawSplashStep(const char* label, bool ok, int step);
void     drawSplashPending(const char* label, int step);
void     updateLeftPanel(uint32_t now);
void     updateRightPanel(float rpm);
void     updateGauge(float psi, float rpm);
void     drawNeedle(float psi);
void     drawRPMNeedle(float rpm);
void     drawRPMPeakMarker(float peakRpm);
void     drawLightTree();
void     eraseLightTree();
void     drawSettingsScreen();
void     drawSettingsIMU();
void     handleSettingsTouch();
void     checkCANHealth();
void     performCANRecovery(uint32_t current_time);
float    getBatteryVoltage();
void     accumulateDistance();
void     resetRun();
void     triggerLaunch(uint32_t now);
void     initSprites();
void     pushGaugeSprite();
void     pushLeftSprite();
void     pushRightSprite();
// DTC
bool     requestDTCs();
bool     clearDTCs();
void     formatDTC(uint8_t b0, uint8_t b1, char* out);
const char* lookupDTC(uint16_t code);
void     drawDTCScreen();
void     handleDTCTouch();

// ----------------------------------------------------------------
// Hardware Pins — Tab5 official pinmap
// ----------------------------------------------------------------
#define CAN_TX_PIN   49   // Expansion header
#define CAN_RX_PIN   50
#define GPS_RX_PIN   53   // HY2.0-4P PORT.A Yellow
#define GPS_TX_PIN   54   // HY2.0-4P PORT.A White
#define SD_CLK_PIN   43   // SDIO CLK
#define SD_CMD_PIN   44   // SDIO CMD
#define SD_D0_PIN    39   // SDIO DAT0
#define SD_D1_PIN    40   // SDIO DAT1
#define SD_D2_PIN    41   // SDIO DAT2
#define SD_D3_PIN    42   // SDIO DAT3

// ----------------------------------------------------------------
// OBD2 Protocol
// ----------------------------------------------------------------
#define OBD2_BROADCAST_ID    0x7DF
#define OBD2_REPLY_ID        0x7E8
#define PID_MAP              0x0B   // Manifold Absolute Pressure (kPa)
#define PID_BARO             0x33   // Barometric Pressure (kPa)
#define PID_RPM              0x0C   // Engine RPM
#define PID_IAT              0x0F   // Intake Air Temp (raw = C + 40)
#define OBD2_SINGLE_BYTE_LEN 0x03
#define OBD2_TWO_BYTE_LEN    0x04

// ----------------------------------------------------------------
// Timing & Fixed Thresholds
// ----------------------------------------------------------------
#define MIN_VALID_BARO         70.0f
#define MAX_VALID_BARO        110.0f
#define DEFAULT_BARO_KPA      101.325f
#define LOG_INTERVAL           10           // Every 10 loops ~100 ms = 1 s log rate
#define CAN_TIMEOUT_MS         200
#define CAN_RECEIVE_WAIT_MS    50
#define CAN_HEALTH_CHECK_MS    5000
#define CAN_MAX_RECOVERY_TRIES 5
#define LAUNCH_SPEED_THRESHOLD 3.0f         // mph — must be near-stationary to arm
#define BATTERY_CHECK_MS       120000UL     // 2 minutes
#define QUARTER_MILE_M         402.336f
#define SD_BUFFER_SIZE         10
// GPS @ 10 Hz: max legitimate step at 150 mph = ~6.7 m; 15 m gives clean margin
#define GPS_MAX_STEP_M         15.0f
// Screen dim after 5 minutes of no activity
#define SCREEN_DIM_MS          300000UL
#define BRIGHTNESS_FULL        255
#define BRIGHTNESS_DIM         60
// Light tree inter-stage delay (ms)
#define TREE_STAGE_MS          500
// Long-press duration to open settings (ms)
#define LONGPRESS_MS           1500

// ----------------------------------------------------------------
// Colour Palette  (RGB565)
// ----------------------------------------------------------------
#define COL_BG         0x18E3   // #181C27  deep navy-charcoal
#define COL_PANEL      0x2124   // #212631  slightly lighter panel
#define COL_BORDER     0x4228   // #424450  subtle border
#define COL_AMBER      0xFD20   // #FF6800  amber — primary value colour
#define COL_AMBER_DIM  0x8940   // #882200  dimmed amber
#define COL_CYAN       0x07FF   // #00FFFF  boost / connected
#define COL_RED        0xF800   // #FF0000  over-boost / error
#define COL_GREEN      0x07E0   // #00FF00  good / launched
#define COL_YELLOW     0xFFE0   // #FFFF00  warning
#define COL_WHITE      0xFFFF
#define COL_LTGREY     0xC618   // #C0C0C0
#define COL_DKGREY     0x632C   // #606060
#define COL_NEEDLE     0xFD20   // amber needle
// Light tree bulb colours
#define COL_BULB_OFF   0x2124   // unlit bulb (panel colour)
#define COL_BULB_AMBER 0xFD20   // lit amber stage
#define COL_BULB_GREEN 0x07E0   // GO

// Additional colours for sprite rendering
#define COL_GAUGE_BG   0x0861   // #080C18 — very dark blue-black gauge face
#define COL_ARC_VAC    0x2965   // dim blue-grey for vacuum zone arc
#define COL_ARC_BOOST  0x8440   // dim amber for boost zone arc bg
#define COL_ARC_RED    0x6000   // dim red for overboost zone arc bg
#define COL_PANEL_MID  0x18E3   // slightly darker panel for section bg
#define COL_AMBER_BRT  0xFEA0   // brighter amber for live values
#define COL_CYAN_DIM   0x0410   // dimmed cyan

// ----------------------------------------------------------------
// Sprite declarations  (M5Canvas = off-screen buffer → push atomically)
// ----------------------------------------------------------------
// Gauge sprite: 600×600 centred on the gauge area
// Left/Right panel sprites: panel width × panel height
// All allocated in PSRAM (ESP32-P4 has 8 MB)
M5Canvas gaugeSprite(&M5.Display);
M5Canvas leftSprite(&M5.Display);
M5Canvas rightSprite(&M5.Display);

// Sprite geometry — computed at runtime from display size
#define GAUGE_SPR_W   600
#define GAUGE_SPR_H   600
// Sprite-local centre (centre of the 600×600 canvas)
#define GCX  (GAUGE_SPR_W / 2)
#define GCY  (GAUGE_SPR_H / 2)
// Where the gauge sprite is pushed to on the display
// Centred horizontally in the panel between LEFT_W and W-RIGHT_W
// cx = display_W/2 = 640, so push_x = 640 - 300 = 340
#define GAUGE_PUSH_X  340
#define GAUGE_PUSH_Y   60   // below status bar, centred vertically

// Arc geometry (all radii relative to GCX/GCY)
#define ARC_BOOST_OUT  230   // outer edge of boost colour arc
#define ARC_BOOST_IN   200   // inner edge of boost colour arc
#define ARC_TICK_OUT   238   // outer edge of boost ticks
#define ARC_TICK_MAJ   218   // major tick inner
#define ARC_TICK_MIN   228   // minor tick inner
#define ARC_LABEL_R    192   // boost label radius
#define ARC_SEP_R      245   // separator ring
#define ARC_RPM_OUT    272   // outer edge of RPM arc band
#define ARC_RPM_IN     252   // inner edge of RPM arc band
#define ARC_RPM_TICK_OUT 272
#define ARC_RPM_TICK_MAJ 255
#define ARC_RPM_TICK_MIN 262
#define ARC_RPM_LABEL_R  284
#define ARC_PEAK_OUT   280   // peak RPM marker outer
#define ARC_PEAK_IN    272
#define NEEDLE_R_OUTER 237   // boost needle tip
#define NEEDLE_R_INNER  35   // boost needle tail (inside pivot)
#define RPM_NEEDLE_OUT 271   // RPM needle tip
#define RPM_NEEDLE_IN  252   // RPM needle tail
// Sweep: 140° start, 260° total (same as before)
#define ARC_START_DEG  140.0f
#define ARC_SWEEP_DEG  260.0f

// ----------------------------------------------------------------
// Layout Constants  (1280 × 720 landscape)
// — must appear before initSprites() which uses them
// ----------------------------------------------------------------
#define STATUS_H   28
#define BOTTOM_H   36
#define LEFT_W    200
#define RIGHT_W   200

// ================================================================
//  SPRITE INIT & PUSH HELPERS
// ================================================================
void initSprites() {
  int H = M5.Display.height();
  int panelH = H - STATUS_H - BOTTOM_H;

  // Gauge sprite — allocate in PSRAM
  gaugeSprite.setColorDepth(16);
  gaugeSprite.createSprite(GAUGE_SPR_W, GAUGE_SPR_H);
  gaugeSprite.fillSprite(COL_GAUGE_BG);

  // Panel sprites
  leftSprite.setColorDepth(16);
  leftSprite.createSprite(LEFT_W, panelH);
  leftSprite.fillSprite(COL_PANEL);

  rightSprite.setColorDepth(16);
  rightSprite.createSprite(RIGHT_W, panelH);
  rightSprite.fillSprite(COL_PANEL);
}

void pushGaugeSprite() {
  gaugeSprite.pushSprite(GAUGE_PUSH_X, GAUGE_PUSH_Y);
}

void pushLeftSprite() {
  leftSprite.pushSprite(0, STATUS_H);
}

void pushRightSprite() {
  int W = M5.Display.width();
  rightSprite.pushSprite(W - RIGHT_W, STATUS_H);
}

// ----------------------------------------------------------------
// Settings (user-adjustable, persisted in globals)
// ----------------------------------------------------------------
// IMU axis for launch detection: 0=X, 1=Y, 2=Z
uint8_t  setting_imu_axis         = 0;
// IAT colour thresholds (°F)
float    setting_iat_warn          = 120.0f;
float    setting_iat_crit          = 150.0f;
// RPM redline for right-panel colour change
int      setting_rpm_redline       = 6000;
// Launch acceleration threshold (g) — axis-independent
float    setting_launch_accel      = 0.10f;

// ----------------------------------------------------------------
// Light Tree State Machine
// ----------------------------------------------------------------
enum TreeState {
  TREE_IDLE,      // waiting for staging conditions
  TREE_STAGED,    // conditions met, showing "STAGED" — tap to start sequence
  TREE_AMBER1,    // first amber lit
  TREE_AMBER2,
  TREE_AMBER3,
  TREE_GREEN,     // GO — waiting for motion
  TREE_DONE       // launched, tree dismissed
};
TreeState tree_state         = TREE_IDLE;
uint32_t  tree_stage_time    = 0;    // millis() when current stage started
uint32_t  tree_green_time    = 0;    // millis() when green lit
float     reaction_time_s    = -1.0f; // seconds from green to motion; -1 = not yet

// ----------------------------------------------------------------
// Global State
// ----------------------------------------------------------------
float    baro_kpa              = DEFAULT_BARO_KPA;

// CAN / OBD2
uint32_t can_rx_count          = 0;
uint32_t can_err_count         = 0;
uint32_t last_can_success_count= 0;
uint32_t last_can_success_time = 0;
uint32_t last_can_health_check = 0;
uint8_t  can_recovery_attempts = 0;
bool     obd2_connected        = false;

// GPS
HardwareSerial GPS_Serial(1);
TinyGPSPlus    gps;
double   startLat              = 0.0;
double   startLng              = 0.0;
double   lastLat               = 0.0;
double   lastLng               = 0.0;
float    current_gps_mph       = 0.0f;
bool     gps_configured        = false;
bool     gps_has_fix           = false;
uint8_t  gps_satellites        = 0;

// Performance tracking
bool     is_launched           = false;
uint32_t launch_start_time     = 0;
bool     reached_60            = false;
float    time_0_60             = 0.0f;
bool     reached_14_mile       = false;
float    time_14_mile          = 0.0f;
float    trap_speed_mph        = 0.0f;   // GPS speed at quarter-mile crossing
float    distance_traveled_m   = 0.0f;
float    peak_boost_psi        = 0.0f;   // max boost seen during run
float    peak_rpm              = 0.0f;   // max RPM seen since last reset

// Sensors
float    intake_temp_f         = -999.0f;
bool     iat_valid             = false;
int      poll_counter          = 0;
int      log_counter           = 0;

// Gauge needle tracking — angles in degrees for sprite redraw
float last_boost_psi   = -999.0f;   // last rendered boost value
float last_rpm_val     = -1.0f;     // last rendered RPM
float old_peak_rpm     = -1.0f;     // last drawn peak RPM marker value
// (old x/y needle coords no longer needed — sprite redraws whole gauge)
int cx, cy;   // display centre — kept for light tree positioning

// SD
File     logFile;
bool     sd_ready              = false;
int      run_number            = 1;
bool     sd_log_failed         = false;
String   sd_buffer[SD_BUFFER_SIZE];
uint8_t  sd_buffer_index       = 0;

// UI / interaction
uint32_t last_touch_time       = 0;
uint32_t touch_press_start     = 0;   // for long-press detection
bool     touch_held            = false;
bool     in_settings           = false;
uint32_t last_status_update    = 0;
uint32_t last_battery_check    = 0;
uint32_t last_activity_time    = 0;   // for screen dimming
bool     screen_dimmed         = false;
float    battery_voltage        = 0.0f;

// DTC screen state
bool     in_dtc_screen          = false;
#define  MAX_DTCS               20
char     dtc_list[MAX_DTCS][6]; // e.g. "P0301\0"
int      dtc_count              = 0;
bool     dtc_scanned            = false;   // true once a scan has been attempted
bool     dtc_clear_confirm      = false;   // two-tap clear confirmation
int      dtc_scroll             = 0;       // top visible card index


// ================================================================
//  BOOT SPLASH
// ================================================================
void drawSplashBG() {
  M5.Display.fillScreen(COL_BG);
  int W = M5.Display.width();
  int H = M5.Display.height();

  M5.Display.fillRect(0, 0, W, 4, COL_AMBER);

  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(COL_WHITE, COL_BG);
  M5.Display.setTextSize(4);
  M5.Display.drawString("OBD2 LOGGER", W / 2, H / 2 - 100);

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_DKGREY, COL_BG);
  M5.Display.drawString("M5STACK TAB5  //  ESP32-P4", W / 2, H / 2 - 60);

  M5.Display.drawFastHLine(W / 2 - 180, H / 2 - 40, 360, COL_BORDER);

  // 5 rows × 28px + 4px padding top/bottom = 148px
  M5.Display.fillRect(W / 2 - 240, H / 2 - 10, 480, 152, COL_PANEL);
  M5.Display.drawRect(W / 2 - 240, H / 2 - 10, 480, 152, COL_BORDER);
}

void drawSplashPending(const char* label, int step) {
  int W = M5.Display.width(), H = M5.Display.height();
  int y = H / 2 + 2 + step * 28 + 4;
  int x = W / 2 - 228;
  M5.Display.fillCircle(x + 8, y + 9, 6, COL_DKGREY);
  M5.Display.setTextDatum(middle_left);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_DKGREY, COL_PANEL);
  M5.Display.drawString(label, x + 22, y + 9);
  M5.Display.setTextDatum(middle_right);
  M5.Display.setTextColor(COL_DKGREY, COL_PANEL);
  M5.Display.drawString("...", W / 2 + 228, y + 9);
}

void drawSplashStep(const char* label, bool ok, int step) {
  int W = M5.Display.width(), H = M5.Display.height();
  int y = H / 2 + 2 + step * 28 + 4;
  int x = W / 2 - 228;
  M5.Display.fillCircle(x + 8, y + 9, 6, ok ? COL_GREEN : COL_RED);
  M5.Display.setTextDatum(middle_left);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_LTGREY, COL_PANEL);
  M5.Display.drawString(label, x + 22, y + 9);
  M5.Display.setTextDatum(middle_right);
  M5.Display.setTextColor(ok ? COL_GREEN : COL_RED, COL_PANEL);
  M5.Display.drawString(ok ? "OK" : "FAIL", W / 2 + 228, y + 9);
}


// ================================================================
//  SETUP
// ================================================================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.setBrightness(BRIGHTNESS_FULL);
  cx = M5.Display.width()  / 2;
  cy = M5.Display.height() / 2;
  initSprites();

  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  configureGPS10Hz();

  drawSplashBG();

  // IMU
  drawSplashPending("IMU (BMI270)", 0);
  bool imu_ok = M5.Imu.begin();
  drawSplashStep("IMU (BMI270)", imu_ok, 0);

  // SD Card
  drawSplashPending("microSD  (SDIO)", 1);
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN, SD_D1_PIN, SD_D2_PIN, SD_D3_PIN);
  for (int a = 0; a < 3 && !sd_ready; a++) {
    if (SD_MMC.begin("/sdcard", false) && SD_MMC.cardType() != CARD_NONE)
      sd_ready = true;
    if (!sd_ready) { SD_MMC.end(); delay(300); }
  }
  if (sd_ready) {
    uint64_t mb = SD_MMC.cardSize() / (1024 * 1024);
    char lbl[32]; sprintf(lbl, "microSD  (%llu MB)", mb);
    drawSplashStep(lbl, true, 1);
  } else {
    drawSplashStep("microSD  (SDIO)", false, 1);
  }

  // CAN
  drawSplashPending("CAN Bus  (500 kbps)", 2);
  setupCAN();
  drawSplashStep("CAN Bus  (500 kbps)", true, 2);

  // OBD2
  drawSplashPending("OBD2 ECU", 3);
  setupOBD2();
  drawSplashStep("OBD2 ECU", obd2_connected, 3);

  // Baro
  drawSplashPending("Baro Pressure", 4);
  float raw_baro = getOBD2Data(PID_BARO);
  bool  baro_ok  = (raw_baro >= MIN_VALID_BARO && raw_baro <= MAX_VALID_BARO);
  baro_kpa = baro_ok ? raw_baro : DEFAULT_BARO_KPA;
  char baroLbl[32]; sprintf(baroLbl, "Baro  (%.1f kPa)", baro_kpa);
  drawSplashStep(baroLbl, baro_ok, 4);

  delay(1200);

  last_can_success_time  = millis();
  last_can_success_count = 0;
  last_can_health_check  = millis();
  battery_voltage        = getBatteryVoltage();
  last_battery_check     = millis();
  last_activity_time     = millis();

  drawBaseUI();
}


// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
  M5.update();
  uint32_t now = millis();

  // --- 1. Feed GPS ---
  while (GPS_Serial.available() > 0) gps.encode(GPS_Serial.read());
  if (gps.speed.isUpdated()) {
    current_gps_mph = gps.speed.mph();
  }
  // Zero speed when fix is lost — prevents stale reading holding launch guard open
  gps_has_fix    = gps.location.isValid();
  gps_satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  if (!gps_has_fix) current_gps_mph = 0.0f;

  // --- 2. Screen dimming ---
  if (!screen_dimmed && (now - last_activity_time > SCREEN_DIM_MS)) {
    M5.Display.setBrightness(BRIGHTNESS_DIM);
    screen_dimmed = true;
  }

  // --- 3. Touch handling ---
  if (M5.Touch.getCount() > 0) {
    auto t = M5.Touch.getDetail();

    // Wake screen on any touch
    if (screen_dimmed) {
      M5.Display.setBrightness(BRIGHTNESS_FULL);
      screen_dimmed      = false;
      last_activity_time = now;
      last_touch_time    = now; // consume this touch as a wake-up only
    } else {
      last_activity_time = now;

      if (in_settings) {
        // Settings screen handles its own touch
        if (t.wasPressed()) handleSettingsTouch();
      } else if (in_dtc_screen) {
        // DTC screen handles its own touch
        if (t.wasPressed()) handleDTCTouch();
      } else {
        // Track press start for long-press detection
        if (t.wasPressed()) {
          touch_press_start = now;
          touch_held        = true;
        }
        // Long press → settings
        if (touch_held && (now - touch_press_start > LONGPRESS_MS)) {
          touch_held = false;
          in_settings = true;
          drawSettingsScreen();
        }
        // Short tap
        if (t.wasReleased() && touch_held) {
          touch_held = false;
          if (now - touch_press_start < LONGPRESS_MS) {
            // In STAGED state a tap starts the light tree sequence
            if (tree_state == TREE_STAGED) {
              tree_state      = TREE_AMBER1;
              tree_stage_time = now;
              drawLightTree();
            } else if (tree_state == TREE_GREEN) {
              // At GREEN: tap acts as manual launch trigger (IMU may not have fired)
              // Treat reaction time as 0 since it's a manual tap not a true launch
              reaction_time_s = (now - tree_green_time) / 1000.0f;
              triggerLaunch(now);
            } else {
              // Any other state — reset run
              resetRun();
              last_touch_time = now;
            }
          }
        }
      }
    }
  } else {
    touch_held = false;
  }

  if (in_settings) {
    drawSettingsIMU();  // live refresh of axis values every loop
    return;
  }
  if (in_dtc_screen) return; // DTC screen runs its own draw loop

  // --- 4. Status bar (1 Hz) ---
  if (now - last_status_update > 1000) {
    drawStatusBar();
    last_status_update = now;
  }

  // --- 5. Battery (2 min) ---
  if (now - last_battery_check > BATTERY_CHECK_MS) {
    battery_voltage    = getBatteryVoltage();
    last_battery_check = now;
    drawStatusBar();
  }

  // --- 6. CAN health ---
  checkCANHealth();

  // --- 7. OBD2 telemetry ---
  // Skip polling during active tree countdown (AMBER1→GREEN) — OBD2 timeouts can
  // block for up to 400 ms per loop which disrupts the 500 ms stage timing and
  // prevents the IMU launch-detect from running responsively at TREE_GREEN.
  bool tree_counting = (tree_state >= TREE_AMBER1 && tree_state <= TREE_GREEN);

  float map_kpa = -1.0f;
  float rpm     = -1.0f;
  if (!tree_counting) {
    map_kpa = getOBD2Data(PID_MAP);
    rpm     = getOBD2Data(PID_RPM);
  }

  poll_counter++;
  if (!tree_counting && poll_counter >= 10) {
    float raw_iat = getOBD2Data(PID_IAT);
    if (raw_iat >= 0.0f) {
      intake_temp_f = ((raw_iat - 40.0f) * 9.0f / 5.0f) + 32.0f;
      iat_valid     = true;
    }
    poll_counter = 0;
  }

  // --- 8. Light tree sequencer ---
  float ax, ay, az;
  bool imu_ok = M5.Imu.getAccel(&ax, &ay, &az);
  float accel_fwd = 0.0f;
  if (imu_ok) {
    // Select axis from settings
    accel_fwd = (setting_imu_axis == 0) ? ax :
                (setting_imu_axis == 1) ? ay : az;
  }

  if (!is_launched && !reached_14_mile) {
    bool conditions_met = gps_has_fix &&
                          gps_satellites >= 4 &&
                          current_gps_mph < LAUNCH_SPEED_THRESHOLD;

    switch (tree_state) {

      case TREE_IDLE:
        if (conditions_met) {
          tree_state = TREE_STAGED;
          drawLightTree();
          drawStagedBanner();
        }
        break;

      case TREE_STAGED:
        // If conditions drop (drove off, lost fix) un-stage
        if (!conditions_met) {
          tree_state = TREE_IDLE;
          eraseLightTree();
          drawReadyBanner();
        }
        break;

      case TREE_AMBER1:
        if (now - tree_stage_time >= TREE_STAGE_MS) {
          tree_state      = TREE_AMBER2;
          tree_stage_time = now;
          drawLightTree();
        }
        break;

      case TREE_AMBER2:
        if (now - tree_stage_time >= TREE_STAGE_MS) {
          tree_state      = TREE_AMBER3;
          tree_stage_time = now;
          drawLightTree();
        }
        break;

      case TREE_AMBER3:
        if (now - tree_stage_time >= TREE_STAGE_MS) {
          tree_state      = TREE_GREEN;
          tree_green_time = now;
          tree_stage_time = now;
          drawLightTree();
        }
        break;

      case TREE_GREEN:
        // Detect motion — forward accel exceeds threshold
        if (imu_ok && accel_fwd > setting_launch_accel) {
          reaction_time_s = (now - tree_green_time) / 1000.0f;
          triggerLaunch(now);
        }
        break;

      case TREE_DONE:
        break;
    }
  }

  // --- 9. Performance math & logging ---
  if (is_launched) {
    float elapsed_s = (now - launch_start_time) / 1000.0f;
    float boost_psi = (map_kpa >= 0.0f) ? (map_kpa - baro_kpa) * 0.145038f : 0.0f;

    if (boost_psi > peak_boost_psi) peak_boost_psi = boost_psi;
    if (rpm > peak_rpm)             peak_rpm        = rpm;

    accumulateDistance();
    float distance_ft = distance_traveled_m * 3.28084f;

    if (!reached_60 && current_gps_mph >= 60.0f) {
      time_0_60  = elapsed_s;
      reached_60 = true;
    }

    if (!reached_14_mile && distance_traveled_m >= QUARTER_MILE_M) {
      time_14_mile    = elapsed_s;
      trap_speed_mph  = current_gps_mph;   // capture trap speed at finish
      reached_14_mile = true;
      is_launched     = false;

      // Flush remaining buffer
      if (logFile && !sd_log_failed && sd_buffer_index > 0) {
        for (uint8_t i = 0; i < sd_buffer_index; i++) logFile.println(sd_buffer[i]);
        logFile.flush();
      }
      if (logFile) logFile.close();

      drawFinishBanner();
    }

    // SD logging (buffered, 1 s rate)
    if (is_launched && logFile && !sd_log_failed) {
      log_counter++;
      if (log_counter >= LOG_INTERVAL) {
        float log_iat = iat_valid ? intake_temp_f : 0.0f;
        char logLine[140];
        sprintf(logLine, "%.2f,%.1f,%d,%.2f,%.2f,%.0f,%.1f,%.6f,%.6f",
          elapsed_s, current_gps_mph, (int)rpm,
          boost_psi, peak_boost_psi,
          log_iat, distance_ft,
          gps.location.lat(), gps.location.lng());
        sd_buffer[sd_buffer_index++] = String(logLine);

        if (sd_buffer_index >= SD_BUFFER_SIZE) {
          for (uint8_t i = 0; i < SD_BUFFER_SIZE; i++) {
            if (logFile.println(sd_buffer[i]) == 0) {
              logFile.close(); sd_log_failed = true; sd_ready = false; break;
            }
          }
          if (!sd_log_failed) logFile.flush();
          sd_buffer_index = 0;
        }
        log_counter = 0;
      }
    }
  }

  // --- 10. Render ---
  // Don't draw over the tree while it's active
  bool tree_active = (tree_state >= TREE_STAGED && tree_state <= TREE_GREEN);

  updateLeftPanel(now);
  updateRightPanel(rpm);

  if (!tree_active) {
    if (map_kpa >= 0.0f) {
      float boost_psi = (map_kpa - baro_kpa) * 0.145038f;
      updateGauge(boost_psi, rpm);
    }
  }

  delay(10);
}


// ================================================================
//  GPS DISTANCE ACCUMULATION
// ================================================================
void accumulateDistance() {
  if (!gps.location.isValid() || !gps.location.isUpdated()) return;
  double lat = gps.location.lat();
  double lng = gps.location.lng();
  if (lastLat == 0.0 && lastLng == 0.0) { lastLat = lat; lastLng = lng; return; }
  float seg = (float)TinyGPSPlus::distanceBetween(lastLat, lastLng, lat, lng);
  // At 10 Hz GPS, max legitimate step at 150 mph ~6.7 m; 15 m filter catches glitches
  if (seg < GPS_MAX_STEP_M) distance_traveled_m += seg;
  lastLat = lat; lastLng = lng;
}


// ================================================================
//  BASE UI SKELETON
// ================================================================
void drawBaseUI() {
  int W = M5.Display.width();
  int H = M5.Display.height();

  M5.Display.fillScreen(COL_BG);

  // Status bar
  M5.Display.fillRect(0, 0, W, STATUS_H, COL_PANEL);
  M5.Display.drawFastHLine(0, STATUS_H, W, COL_AMBER);  // amber accent line

  // Bottom banner
  M5.Display.fillRect(0, H - BOTTOM_H, W, BOTTOM_H, COL_PANEL);
  M5.Display.drawFastHLine(0, H - BOTTOM_H, W, COL_BORDER);

  // Left panel
  M5.Display.fillRect(0, STATUS_H, LEFT_W, H - STATUS_H - BOTTOM_H, COL_PANEL);
  M5.Display.drawFastVLine(LEFT_W, STATUS_H, H - STATUS_H - BOTTOM_H, COL_BORDER);
  // Left panel amber accent top bar
  M5.Display.fillRect(0, STATUS_H, LEFT_W, 2, COL_AMBER_DIM);

  // Right panel
  M5.Display.fillRect(W - RIGHT_W, STATUS_H, RIGHT_W, H - STATUS_H - BOTTOM_H, COL_PANEL);
  M5.Display.drawFastVLine(W - RIGHT_W, STATUS_H, H - STATUS_H - BOTTOM_H, COL_BORDER);
  M5.Display.fillRect(W - RIGHT_W, STATUS_H, RIGHT_W, 2, COL_AMBER_DIM);

  drawGaugeFace();
  drawLeftPanelLabels();
  drawRightPanelLabels();
  drawStatusBar();
  drawReadyBanner();

  tree_state = TREE_IDLE;
  last_boost_psi = -999.0f;
  last_rpm_val   = -1.0f;
  old_peak_rpm   = -1.0f;
}

void drawGaugeFace() {
  // Draw the static gauge face into the gauge sprite, then push to display.
  // Called once at startup and after eraseLightTree.
  gaugeSprite.fillSprite(COL_GAUGE_BG);

  // ── Outer glow rings ─────────────────────────────────────────
  gaugeSprite.drawCircle(GCX, GCY, ARC_BOOST_OUT + 22, COL_BORDER);
  gaugeSprite.drawCircle(GCX, GCY, ARC_BOOST_OUT + 21, COL_BORDER);
  gaugeSprite.drawCircle(GCX, GCY, ARC_BOOST_OUT + 20, 0x10A2); // very dim glow

  // ── RPM arc background band ───────────────────────────────────
  // Draw coloured arc segments for the RPM zones
  // 0 → redline-1000: dark grey band
  // redline-1000 → redline: amber warning band
  // redline → 8000: red danger band
  float rpmWarnStart = ((float)(setting_rpm_redline - 1000) / 8000.0f) * ARC_SWEEP_DEG + ARC_START_DEG;
  float rpmRedStart  = ((float)setting_rpm_redline          / 8000.0f) * ARC_SWEEP_DEG + ARC_START_DEG;
  float arcEnd       = ARC_START_DEG + ARC_SWEEP_DEG;

  // Normal zone
  gaugeSprite.drawArc(GCX, GCY, ARC_RPM_OUT, ARC_RPM_IN,
                      ARC_START_DEG, rpmWarnStart, 0x18C3);
  // Warning zone
  gaugeSprite.drawArc(GCX, GCY, ARC_RPM_OUT, ARC_RPM_IN,
                      rpmWarnStart, rpmRedStart, 0x5220);
  // Redline zone
  gaugeSprite.drawArc(GCX, GCY, ARC_RPM_OUT, ARC_RPM_IN,
                      rpmRedStart, arcEnd, 0x4000);

  // ── Boost arc background band ─────────────────────────────────
  // Vacuum zone: dark blue-grey
  float boostZeroAngle = (15.0f / 35.0f) * ARC_SWEEP_DEG + ARC_START_DEG;
  float boost15Angle   = (30.0f / 35.0f) * ARC_SWEEP_DEG + ARC_START_DEG;

  gaugeSprite.drawArc(GCX, GCY, ARC_BOOST_OUT, ARC_BOOST_IN,
                      ARC_START_DEG, boostZeroAngle, 0x0861);
  // Positive boost zone: dark amber
  gaugeSprite.drawArc(GCX, GCY, ARC_BOOST_OUT, ARC_BOOST_IN,
                      boostZeroAngle, boost15Angle, 0x2200);
  // Over-boost zone: dark red
  gaugeSprite.drawArc(GCX, GCY, ARC_BOOST_OUT, ARC_BOOST_IN,
                      boost15Angle, arcEnd, 0x3000);

  // ── Separator ring ────────────────────────────────────────────
  gaugeSprite.drawCircle(GCX, GCY, ARC_SEP_R,     0x3186);
  gaugeSprite.drawCircle(GCX, GCY, ARC_SEP_R + 1, 0x2104);

  // ── Boost tick marks ─────────────────────────────────────────
  for (int psi = -15; psi <= 20; psi++) {
    float angle = (psi + 15.0f) * (ARC_SWEEP_DEG / 35.0f) + ARC_START_DEG;
    float rad   = angle * PI / 180.0f;
    float cosA  = cosf(rad), sinA = sinf(rad);
    bool  major = (psi % 5 == 0);
    int   r_out = ARC_TICK_OUT;
    int   r_in  = major ? ARC_TICK_MAJ : ARC_TICK_MIN;
    uint16_t col = (psi >= 15) ? COL_RED   :
                   (psi >= 0)  ? 0xFCA0    : 0x528A;  // brighter for visibility
    // Draw tick 2px wide
    for (int t = 0; t < 2; t++) {
      float tr = (rad + t * 0.005f);
      gaugeSprite.drawLine(
        GCX + (int)(r_out * cosf(tr)), GCY + (int)(r_out * sinf(tr)),
        GCX + (int)(r_in  * cosf(tr)), GCY + (int)(r_in  * sinf(tr)), col);
    }
    if (major) {
      char lbl[6]; sprintf(lbl, "%d", psi);
      gaugeSprite.setTextDatum(middle_center);
      gaugeSprite.setTextSize(1);
      gaugeSprite.setTextColor(col, COL_GAUGE_BG);
      gaugeSprite.drawString(lbl,
        GCX + (int)(ARC_LABEL_R * cosA),
        GCY + (int)(ARC_LABEL_R * sinA));
    }
  }

  // ── RPM tick marks ────────────────────────────────────────────
  for (int r = 0; r <= 8000; r += 500) {
    float angle = ((float)r / 8000.0f) * ARC_SWEEP_DEG + ARC_START_DEG;
    float rad   = angle * PI / 180.0f;
    float cosA  = cosf(rad), sinA = sinf(rad);
    bool  major = (r % 1000 == 0);
    int   r_out = ARC_RPM_TICK_OUT;
    int   r_in  = major ? ARC_RPM_TICK_MAJ : ARC_RPM_TICK_MIN;
    uint16_t col = (r >= setting_rpm_redline)           ? COL_RED      :
                   (r >= setting_rpm_redline - 1000)    ? COL_AMBER_DIM : 0x39C7;
    gaugeSprite.drawLine(
      GCX + (int)(r_out * cosA), GCY + (int)(r_out * sinA),
      GCX + (int)(r_in  * cosA), GCY + (int)(r_in  * sinA), col);
    if (major && r > 0) {
      char lbl[4]; sprintf(lbl, "%d", r / 1000);
      gaugeSprite.setTextDatum(middle_center);
      gaugeSprite.setTextSize(1);
      gaugeSprite.setTextColor(col, COL_GAUGE_BG);
      gaugeSprite.drawString(lbl,
        GCX + (int)(ARC_RPM_LABEL_R * cosA),
        GCY + (int)(ARC_RPM_LABEL_R * sinA));
    }
  }

  // ── Centre labels ─────────────────────────────────────────────
  gaugeSprite.setTextDatum(middle_center);
  gaugeSprite.setTextSize(2);
  gaugeSprite.setTextColor(0x4A69, COL_GAUGE_BG);
  gaugeSprite.drawString("BOOST", GCX, GCY - 70);
  gaugeSprite.drawString("PSI",   GCX, GCY + 70);

  // ── Inner circle fill (clean dark face) ──────────────────────
  // Draw a filled circle to give the gauge face depth
  // This is drawn UNDER the text via the arc background above,
  // so we just add a subtle inner ring
  gaugeSprite.drawCircle(GCX, GCY, ARC_BOOST_IN - 1, 0x1082);
  gaugeSprite.drawCircle(GCX, GCY, ARC_BOOST_IN - 2, 0x0841);

  // RPM label
  gaugeSprite.setTextSize(1);
  gaugeSprite.setTextColor(0x2965, COL_GAUGE_BG);
  gaugeSprite.drawString("RPM x1000", GCX, GCY + GCY - 20);

  pushGaugeSprite();
}

void drawLeftPanelLabels() {
  int H      = M5.Display.height();
  int panelH = H - STATUS_H - BOTTOM_H;
  leftSprite.fillSprite(COL_PANEL);

  int W = LEFT_W;
  int pcx = W / 2;
  int top = 12;
  int mid = panelH / 2;
  int bot = mid + 100;

  // Section dividers — subtle gradient-style using two lines
  leftSprite.drawFastHLine(8,  mid - 1, W - 16, COL_BORDER);
  leftSprite.drawFastHLine(8,  mid,     W - 16, 0x2945);
  leftSprite.drawFastHLine(8,  bot - 1, W - 16, COL_BORDER);
  leftSprite.drawFastHLine(8,  bot,     W - 16, 0x2945);

  // Labels
  leftSprite.setTextDatum(middle_center);
  leftSprite.setTextSize(1);
  leftSprite.setTextColor(0x4A69, COL_PANEL);
  leftSprite.drawString("ELAPSED", pcx, top + 4);
  leftSprite.drawString("0  -  6 0   M P H", pcx, mid + 14);
  leftSprite.drawString("1 / 4   M I L E", pcx, bot + 14);

  // Default dashes
  leftSprite.setTextSize(3);
  leftSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
  leftSprite.setTextPadding(W - 10);
  leftSprite.drawString("--.-s", pcx, top + 44);
  leftSprite.drawString("--.-s", pcx, mid + 48);
  leftSprite.drawString("--.-s", pcx, bot + 48);
  leftSprite.setTextSize(2);
  leftSprite.drawString("RT  ---", pcx, top + 76);
  leftSprite.setTextPadding(0);

  pushLeftSprite();
}

void drawRightPanelLabels() {
  int H     = M5.Display.height();
  int W     = RIGHT_W;
  int pcx   = W / 2;
  int panelH = H - STATUS_H - BOTTOM_H;
  int secH  = panelH / 3;
  rightSprite.fillSprite(COL_PANEL);

  // Section dividers
  rightSprite.drawFastHLine(8, secH - 1, W - 16, COL_BORDER);
  rightSprite.drawFastHLine(8, secH,     W - 16, 0x2945);
  rightSprite.drawFastHLine(8, 2*secH-1, W - 16, COL_BORDER);
  rightSprite.drawFastHLine(8, 2*secH,   W - 16, 0x2945);

  rightSprite.setTextDatum(middle_center);
  rightSprite.setTextSize(1);
  rightSprite.setTextColor(0x4A69, COL_PANEL);
  rightSprite.drawString("R P M", pcx, secH / 2 - 22);
  rightSprite.drawString("M P H", pcx, secH + secH / 2 - 22);
  rightSprite.drawString("I A T   °F", pcx, 2 * secH + secH / 2 - 22);

  rightSprite.setTextSize(3);
  rightSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
  rightSprite.setTextPadding(W - 10);
  rightSprite.drawString("----",  pcx, secH / 2 + 12);
  rightSprite.drawString("--.-",  pcx, secH + secH / 2 + 12);
  rightSprite.drawString("---",   pcx, 2 * secH + secH / 2 + 12);
  rightSprite.setTextPadding(0);

  pushRightSprite();
}


// ================================================================
//  LIGHT TREE
//  Drawn over the centre gauge area.
//  Layout (portrait stack centred on cx, cy):
//    3 amber bulbs (r=28 each, 70px spacing)
//    1 green bulb  (r=28, 70px below last amber)
//  Lit stages:
//    TREE_STAGED  : all off, label "STAGED — TAP TO START"
//    TREE_AMBER1  : top amber lit
//    TREE_AMBER2  : top + middle amber
//    TREE_AMBER3  : all three ambers
//    TREE_GREEN   : all three ambers off, green lit
// ================================================================
#define TREE_BULB_R    28
#define TREE_BULB_SEP  70
#define TREE_START_Y   (cy - 105)  // top bulb centre

void drawLightTree() {
  // Background panel behind the tree — must be tall enough to include the label below the green bulb
  // Green bulb centre: TREE_START_Y + 3*TREE_BULB_SEP; label at bulb_bottom + 20px
  // Total height needed: 3*SEP + 2*R + 20(label_margin) + 16(text_height) + 10(bottom_pad)
  int treeW = TREE_BULB_R * 2 + 24;
  int treeH = TREE_BULB_SEP * 3 + TREE_BULB_R * 2 + 60;  // +60 ensures label row is inside panel
  int treeX = cx - treeW / 2;
  int treeY = TREE_START_Y - TREE_BULB_R - 10;

  M5.Display.fillRoundRect(treeX, treeY, treeW, treeH, 8, COL_PANEL);
  M5.Display.drawRoundRect(treeX, treeY, treeW, treeH, 8, COL_BORDER);

  bool a1 = (tree_state >= TREE_AMBER1 && tree_state <= TREE_AMBER3);
  bool a2 = (tree_state >= TREE_AMBER2 && tree_state <= TREE_AMBER3);
  bool a3 = (tree_state == TREE_AMBER3);
  bool go  = (tree_state == TREE_GREEN);

  // Amber bulbs
  for (int i = 0; i < 3; i++) {
    int by     = TREE_START_Y + i * TREE_BULB_SEP;
    bool lit   = (i == 0) ? a1 : (i == 1) ? a2 : a3;
    uint16_t c = lit ? COL_BULB_AMBER : COL_BULB_OFF;
    M5.Display.fillCircle(cx, by, TREE_BULB_R, c);
    M5.Display.drawCircle(cx, by, TREE_BULB_R, lit ? COL_AMBER : COL_BORDER);
    if (lit) {
      // Glow ring
      M5.Display.drawCircle(cx, by, TREE_BULB_R + 3, COL_AMBER_DIM);
    }
  }

  // Green bulb
  int gy = TREE_START_Y + 3 * TREE_BULB_SEP;
  uint16_t gc = go ? COL_BULB_GREEN : COL_BULB_OFF;
  M5.Display.fillCircle(cx, gy, TREE_BULB_R, gc);
  M5.Display.drawCircle(cx, gy, TREE_BULB_R, go ? COL_GREEN : COL_BORDER);
  if (go) M5.Display.drawCircle(cx, gy, TREE_BULB_R + 3, 0x03E0); // dim green glow

  // Label below tree
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  int labelY = gy + TREE_BULB_R + 20;

  if (tree_state == TREE_STAGED) {
    M5.Display.setTextColor(COL_YELLOW, COL_PANEL);
    M5.Display.drawString("STAGED", cx, labelY);
  } else if (tree_state == TREE_GREEN) {
    M5.Display.setTextColor(COL_GREEN, COL_PANEL);
    M5.Display.drawString("GO! / TAP", cx, labelY);
  } else {
    // Show countdown number
    int remaining = 3 - (int)(tree_state - TREE_AMBER1);
    char buf[4]; sprintf(buf, "%d", remaining > 0 ? remaining : 0);
    M5.Display.setTextColor(COL_AMBER, COL_PANEL);
    M5.Display.drawString(buf, cx, labelY);
  }
}

// Restore the gauge face where the tree was drawn
void eraseLightTree() {
  // drawGaugeFace() redraws into the gauge sprite and pushes — that's all we need.
  // The next updateGauge() call will paint the live values on top.
  drawGaugeFace();
}


// ================================================================
//  STATUS BAR
// ================================================================
void drawStatusBar() {
  int W = M5.Display.width();
  M5.Display.fillRect(0, 0, W, STATUS_H, COL_PANEL);
  M5.Display.drawFastHLine(0, STATUS_H, W, COL_AMBER);  // amber accent
  M5.Display.setTextSize(2);

  // GPS
  M5.Display.setTextDatum(middle_left);
  if (gps_has_fix) {
    char buf[24]; sprintf(buf, "GPS  %d SAT", gps_satellites);
    M5.Display.setTextColor(COL_GREEN, COL_PANEL);
    M5.Display.drawString(buf, 8, STATUS_H / 2);
  } else {
    M5.Display.setTextColor(COL_YELLOW, COL_PANEL);
    M5.Display.drawString("GPS  NO FIX", 8, STATUS_H / 2);
  }

  // OBD2
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(obd2_connected ? COL_GREEN : COL_RED, COL_PANEL);
  M5.Display.drawString(obd2_connected ? "OBD2  LIVE" : "OBD2  OFF", W / 2, STATUS_H / 2);

  // Battery + SD
  M5.Display.setTextDatum(middle_right);
  uint16_t bc = (battery_voltage < 6.5f) ? COL_RED :
                (battery_voltage < 7.0f) ? COL_YELLOW : COL_GREEN;
  char buf[32];
  sprintf(buf, "%s  %.1fV", sd_ready ? "SD" : "--", battery_voltage);
  M5.Display.setTextColor(bc, COL_PANEL);
  M5.Display.drawString(buf, W - 8, STATUS_H / 2);
}


// ================================================================
//  BOTTOM BANNERS
// ================================================================
void drawReadyBanner() {
  int W = M5.Display.width(), H = M5.Display.height();
  M5.Display.fillRect(0, H - BOTTOM_H, W, BOTTOM_H, COL_PANEL);
  M5.Display.drawFastHLine(0, H - BOTTOM_H, W, COL_BORDER);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  if (!gps_has_fix) {
    M5.Display.setTextColor(COL_YELLOW, COL_PANEL);
    M5.Display.drawString("AWAITING GPS LOCK  |  HOLD TO SETTINGS", W / 2, H - BOTTOM_H / 2);
  } else {
    M5.Display.setTextColor(COL_DKGREY, COL_PANEL);
    M5.Display.drawString("READY  |  PULL FORWARD TO STAGE  |  HOLD FOR SETTINGS", W / 2, H - BOTTOM_H / 2);
  }
}

void drawStagedBanner() {
  int W = M5.Display.width(), H = M5.Display.height();
  M5.Display.fillRect(0, H - BOTTOM_H, W, BOTTOM_H, 0x2200);
  M5.Display.drawFastHLine(0, H - BOTTOM_H, W, COL_YELLOW);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_YELLOW, 0x2200);
  M5.Display.drawString("STAGED  |  TAP TO START TREE  |  TAP AGAIN TO RESET", W / 2, H - BOTTOM_H / 2);
}

void drawLaunchBanner() {
  int W = M5.Display.width(), H = M5.Display.height();
  M5.Display.fillRect(0, H - BOTTOM_H, W, BOTTOM_H, 0x0320);
  M5.Display.drawFastHLine(0, H - BOTTOM_H, W, COL_GREEN);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_GREEN, 0x0320);
  char buf[48];
  if (sd_ready) sprintf(buf, "LAUNCHED  |  LOGGING  RUN %d  |  TAP TO RESET", run_number);
  else          sprintf(buf, "LAUNCHED  |  NO SD  |  TAP TO RESET");
  M5.Display.drawString(buf, W / 2, H - BOTTOM_H / 2);
}

void drawFinishBanner() {
  int W = M5.Display.width(), H = M5.Display.height();
  M5.Display.fillRect(0, H - BOTTOM_H, W, BOTTOM_H, 0x2000);
  M5.Display.drawFastHLine(0, H - BOTTOM_H, W, COL_AMBER);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_AMBER, 0x2000);
  char buf[64];
  sprintf(buf, "FINISH  |  TRAP %.1f mph  |  TAP TO RESET", trap_speed_mph);
  M5.Display.drawString(buf, W / 2, H - BOTTOM_H / 2);
}


// ================================================================
//  LEFT PANEL  — sprite-based, pushed atomically
// ================================================================
void updateLeftPanel(uint32_t now) {
  int H      = M5.Display.height();
  int panelH = H - STATUS_H - BOTTOM_H;
  int W      = LEFT_W;
  int pcx    = W / 2;
  int top    = 12;
  int mid    = panelH / 2;
  int bot    = mid + 100;

  leftSprite.fillSprite(COL_PANEL);

  // Section dividers
  leftSprite.drawFastHLine(8, mid - 1, W - 16, COL_BORDER);
  leftSprite.drawFastHLine(8, mid,     W - 16, 0x2945);
  leftSprite.drawFastHLine(8, bot - 1, W - 16, COL_BORDER);
  leftSprite.drawFastHLine(8, bot,     W - 16, 0x2945);

  // Section labels
  leftSprite.setTextDatum(middle_center);
  leftSprite.setTextSize(1);
  leftSprite.setTextColor(0x4A69, COL_PANEL);
  leftSprite.drawString("ELAPSED",           pcx, top + 4);
  leftSprite.drawString("0  -  6 0   M P H", pcx, mid + 14);
  leftSprite.drawString("1 / 4   M I L E",   pcx, bot + 14);

  leftSprite.setTextPadding(W - 10);

  // ── Elapsed ───────────────────────────────────────────────────
  leftSprite.setTextSize(3);
  if (reached_14_mile) {
    char buf[16]; sprintf(buf, "%.2fs", time_14_mile);
    leftSprite.setTextColor(COL_AMBER, COL_PANEL);
    leftSprite.drawString(buf, pcx, top + 44);
  } else if (is_launched) {
    char buf[16]; sprintf(buf, "%.2fs", (now - launch_start_time) / 1000.0f);
    leftSprite.setTextColor(COL_GREEN, COL_PANEL);
    leftSprite.drawString(buf, pcx, top + 44);
  } else {
    leftSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
    leftSprite.drawString("--.-s", pcx, top + 44);
  }

  // ── Reaction time ─────────────────────────────────────────────
  leftSprite.setTextSize(2);
  if (reaction_time_s >= 0.0f) {
    char buf[20]; sprintf(buf, "RT  %.3fs", reaction_time_s);
    leftSprite.setTextColor(COL_CYAN, COL_PANEL);
    leftSprite.drawString(buf, pcx, top + 76);
  } else {
    leftSprite.setTextColor(0x2965, COL_PANEL);
    leftSprite.drawString("RT  ---", pcx, top + 76);
  }

  // ── 0-60 ──────────────────────────────────────────────────────
  leftSprite.setTextSize(3);
  if (reached_60) {
    char buf[16]; sprintf(buf, "%.2fs", time_0_60);
    leftSprite.setTextColor(COL_CYAN, COL_PANEL);
    leftSprite.drawString(buf, pcx, mid + 48);
  } else {
    leftSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
    leftSprite.drawString("--.-s", pcx, mid + 48);
  }

  // ── 1/4 mile ─────────────────────────────────────────────────
  if (reached_14_mile) {
    char buf[16]; sprintf(buf, "%.2fs", time_14_mile);
    leftSprite.setTextColor(COL_AMBER, COL_PANEL);
    leftSprite.drawString(buf, pcx, bot + 48);
  } else {
    leftSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
    leftSprite.drawString("--.-s", pcx, bot + 48);
  }

  // ── Trap speed ────────────────────────────────────────────────
  leftSprite.setTextSize(2);
  if (reached_14_mile && trap_speed_mph > 0.0f) {
    char buf[20]; sprintf(buf, "%.1f mph", trap_speed_mph);
    leftSprite.setTextColor(COL_AMBER, COL_PANEL);
    leftSprite.drawString(buf, pcx, bot + 78);
  }

  leftSprite.setTextPadding(0);
  pushLeftSprite();
}


// ================================================================
//  RIGHT PANEL  — sprite-based
// ================================================================
void updateRightPanel(float rpm) {
  int H     = M5.Display.height();
  int W     = RIGHT_W;
  int pcx   = W / 2;
  int panelH = H - STATUS_H - BOTTOM_H;
  int secH  = panelH / 3;

  rightSprite.fillSprite(COL_PANEL);

  // Section dividers
  rightSprite.drawFastHLine(8, secH - 1,   W - 16, COL_BORDER);
  rightSprite.drawFastHLine(8, secH,        W - 16, 0x2945);
  rightSprite.drawFastHLine(8, 2*secH - 1, W - 16, COL_BORDER);
  rightSprite.drawFastHLine(8, 2*secH,     W - 16, 0x2945);

  // Section labels
  rightSprite.setTextDatum(middle_center);
  rightSprite.setTextSize(1);
  rightSprite.setTextColor(0x4A69, COL_PANEL);
  rightSprite.drawString("R P M",     pcx, secH / 2 - 22);
  rightSprite.drawString("M P H",     pcx, secH + secH / 2 - 22);
  rightSprite.drawString("I A T  °F", pcx, 2 * secH + secH / 2 - 22);

  rightSprite.setTextSize(3);
  rightSprite.setTextPadding(W - 10);

  // ── RPM ───────────────────────────────────────────────────────
  if (rpm > 0.0f) {
    char buf[10]; sprintf(buf, "%d", (int)rpm);
    rightSprite.setTextColor((rpm > setting_rpm_redline) ? COL_RED : COL_AMBER_BRT, COL_PANEL);
    rightSprite.drawString(buf, pcx, secH / 2 + 12);
  } else {
    rightSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
    rightSprite.drawString("----", pcx, secH / 2 + 12);
  }

  // ── Speed ─────────────────────────────────────────────────────
  {
    char buf[10]; sprintf(buf, "%.1f", current_gps_mph);
    rightSprite.setTextColor(COL_AMBER_BRT, COL_PANEL);
    rightSprite.drawString(buf, pcx, secH + secH / 2 + 12);
  }

  // ── IAT ───────────────────────────────────────────────────────
  if (iat_valid) {
    char buf[10]; sprintf(buf, "%.0f", intake_temp_f);
    uint16_t c = (intake_temp_f > setting_iat_crit)  ? COL_RED    :
                 (intake_temp_f > setting_iat_warn)   ? COL_YELLOW : COL_AMBER_BRT;
    rightSprite.setTextColor(c, COL_PANEL);
    rightSprite.drawString(buf, pcx, 2 * secH + secH / 2 + 12);
  } else {
    rightSprite.setTextColor(COL_AMBER_DIM, COL_PANEL);
    rightSprite.drawString("---", pcx, 2 * secH + secH / 2 + 12);
  }

  rightSprite.setTextPadding(0);
  pushRightSprite();
}


// ================================================================
//  BOOST GAUGE  — sprite-based, full redraw each frame
// ================================================================

inline float boostAngleDeg(float psi) {
  if (psi < -15.0f) psi = -15.0f;
  if (psi >  20.0f) psi =  20.0f;
  return (psi + 15.0f) * (ARC_SWEEP_DEG / 35.0f) + ARC_START_DEG;
}
inline float rpmAngleDeg(float rpm) {
  if (rpm < 0.0f)    rpm = 0.0f;
  if (rpm > 8000.0f) rpm = 8000.0f;
  return rpm / 8000.0f * ARC_SWEEP_DEG + ARC_START_DEG;
}
inline float boostAngleRad(float psi) { return boostAngleDeg(psi) * PI / 180.0f; }
inline float rpmAngleRad(float rpm)   { return rpmAngleDeg(rpm)   * PI / 180.0f; }

// Draw a needle into the gauge sprite
static void spriteNeedle(float angleDeg, int r_inner, int r_outer,
                         uint16_t col, int width) {
  float rad  = angleDeg * PI / 180.0f;
  float cosA = cosf(rad), sinA = sinf(rad);
  int ix = GCX + (int)(r_inner * cosA);
  int iy = GCY + (int)(r_inner * sinA);
  int ox = GCX + (int)(r_outer * cosA);
  int oy = GCY + (int)(r_outer * sinA);
  for (int d = -(width/2); d <= width/2; d++) {
    gaugeSprite.drawLine(ix + d, iy, ox + d, oy, col);
    gaugeSprite.drawLine(ix, iy + d, ox, oy + d, col);
  }
}

void updateGauge(float psi, float rpm) {
  // ── Rebuild gauge sprite from face ────────────────────────────
  // Start from stored face then overlay live elements.
  // For performance: redraw face ticks/arcs directly (fast on sprite).
  drawGaugeFace();   // fills sprite and pushes — we'll add overlays then push again

  // ── Live boost arc highlight ───────────────────────────────────
  // Draw a bright arc from zero to current PSI on top of the face
  if (psi > -15.0f) {
    float startDeg = boostAngleDeg(0.0f);
    float endDeg   = boostAngleDeg(psi);
    if (psi < 0.0f) { startDeg = boostAngleDeg(psi); endDeg = boostAngleDeg(0.0f); }
    uint16_t liveCol = (psi < 0.0f) ? 0x07BF :   // blue-cyan for vacuum
                       (psi > 15.0f) ? COL_RED  : COL_AMBER;
    // Bright live arc — 3px wide
    for (int t = -1; t <= 1; t++) {
      gaugeSprite.drawArc(GCX, GCY,
        ARC_BOOST_OUT - t, ARC_BOOST_IN + t,
        (psi < 0.0f) ? startDeg : startDeg,
        (psi < 0.0f) ? endDeg   : endDeg,
        liveCol);
    }
  }

  // ── Live RPM arc highlight ─────────────────────────────────────
  if (rpm > 0.0f) {
    float endDeg = rpmAngleDeg(rpm);
    uint16_t rpmLiveCol = (rpm >= setting_rpm_redline) ? COL_RED : COL_CYAN;
    for (int t = -1; t <= 1; t++) {
      gaugeSprite.drawArc(GCX, GCY,
        ARC_RPM_OUT - t, ARC_RPM_IN + t,
        ARC_START_DEG, endDeg, rpmLiveCol);
    }
  }

  // ── Peak RPM marker ───────────────────────────────────────────
  if (peak_rpm > 0.0f) {
    float pkDeg = rpmAngleDeg(peak_rpm);
    float pkRad = pkDeg * PI / 180.0f;
    float cosA  = cosf(pkRad), sinA = sinf(pkRad);
    // Bright tick beyond outer arc
    for (int d = -2; d <= 2; d++) {
      gaugeSprite.drawLine(GCX + (int)(ARC_PEAK_IN  * cosA) + d,
                           GCY + (int)(ARC_PEAK_IN  * sinA),
                           GCX + (int)(ARC_PEAK_OUT * cosA) + d,
                           GCY + (int)(ARC_PEAK_OUT * sinA), COL_CYAN);
      gaugeSprite.drawLine(GCX + (int)(ARC_PEAK_IN  * cosA),
                           GCY + (int)(ARC_PEAK_IN  * sinA) + d,
                           GCX + (int)(ARC_PEAK_OUT * cosA),
                           GCY + (int)(ARC_PEAK_OUT * sinA) + d, COL_CYAN);
    }
    // Peak value label
    char pkbuf[8]; sprintf(pkbuf, "%d", (int)peak_rpm);
    gaugeSprite.setTextDatum(middle_center);
    gaugeSprite.setTextSize(1);
    gaugeSprite.setTextColor(COL_CYAN, COL_GAUGE_BG);
    gaugeSprite.drawString(pkbuf,
      GCX + (int)(296 * cosA), GCY + (int)(296 * sinA));
  }

  // ── Boost needle ──────────────────────────────────────────────
  float boostDeg = boostAngleDeg(psi);
  uint16_t nCol  = (psi > 14.0f) ? COL_RED : COL_AMBER_BRT;
  spriteNeedle(boostDeg, NEEDLE_R_INNER, NEEDLE_R_OUTER, nCol, 4);

  // ── RPM needle ────────────────────────────────────────────────
  if (rpm > 0.0f) {
    float rpmDeg  = rpmAngleDeg(rpm);
    uint16_t rCol = (rpm >= setting_rpm_redline) ? COL_RED : COL_CYAN;
    spriteNeedle(rpmDeg, RPM_NEEDLE_IN, RPM_NEEDLE_OUT, rCol, 2);
  }

  // ── Pivot ─────────────────────────────────────────────────────
  gaugeSprite.fillCircle(GCX, GCY, 7, COL_PANEL);
  gaugeSprite.drawCircle(GCX, GCY, 8, COL_AMBER);
  gaugeSprite.drawCircle(GCX, GCY, 9, 0x8420);  // subtle outer glow

  // ── Centre PSI value ──────────────────────────────────────────
  char valStr[10]; dtostrf(psi, 5, 1, valStr);
  uint16_t valCol = (psi < 0.0f)  ? 0x07DF :
                    (psi > 15.0f) ? COL_RED : COL_AMBER_BRT;

  // Shadow pass
  gaugeSprite.setTextDatum(middle_center);
  gaugeSprite.setTextSize(7);
  gaugeSprite.setTextColor(0x0841, COL_GAUGE_BG);
  gaugeSprite.setTextPadding(260);
  gaugeSprite.drawString(valStr, GCX + 1, GCY + 1);
  // Live value
  gaugeSprite.setTextColor(valCol, COL_GAUGE_BG);
  gaugeSprite.drawString(valStr, GCX, GCY);
  gaugeSprite.setTextPadding(0);

  // ── Peak boost label ──────────────────────────────────────────
  if (peak_boost_psi > 0.0f) {
    gaugeSprite.setTextSize(2);
    gaugeSprite.setTextPadding(160);
    char pk[16]; sprintf(pk, "PK  %.1f", peak_boost_psi);
    gaugeSprite.setTextColor(0x528A, COL_GAUGE_BG);
    gaugeSprite.drawString(pk, GCX, GCY - 108);
    gaugeSprite.setTextPadding(0);
  }

  // ── BOOST / PSI centre labels ─────────────────────────────────
  gaugeSprite.setTextSize(2);
  gaugeSprite.setTextColor(0x4A69, COL_GAUGE_BG);
  gaugeSprite.setTextDatum(middle_center);
  gaugeSprite.drawString("BOOST", GCX, GCY - 70);
  gaugeSprite.drawString("PSI",   GCX, GCY + 70);

  pushGaugeSprite();
}

// Legacy stubs — kept so the tree erase path still compiles
void drawNeedle(float) {}
void drawRPMNeedle(float) {}
void drawRPMPeakMarker(float) {}


// ================================================================
//  TRIGGER LAUNCH
//  Called from both IMU detection and manual tap-at-green.
//  reaction_time_s must be set by the caller before calling this.
// ================================================================
void triggerLaunch(uint32_t now) {
  tree_state          = TREE_DONE;
  is_launched         = true;
  launch_start_time   = now;
  peak_boost_psi      = 0.0f;
  peak_rpm            = 0.0f;
  sd_buffer_index     = 0;
  log_counter         = 0;
  distance_traveled_m = 0.0f;
  startLat = gps.location.lat();
  startLng = gps.location.lng();
  lastLat  = startLat;
  lastLng  = startLng;

  if (sd_ready) {
    char filename[32];
    run_number = 1;
    while (run_number < 1000) {
      sprintf(filename, "/run_%d.csv", run_number);
      if (!SD_MMC.exists(filename)) break;
      run_number++;
    }
    if (run_number >= 1000) {
      run_number = 1;
      sprintf(filename, "/run_%d.csv", run_number);
      SD_MMC.remove(filename);
    }
    logFile = SD_MMC.open(filename, FILE_WRITE);
    if (logFile)
      logFile.println(
        "Time_s,GPS_Speed_mph,RPM,Boost_psi,Peak_Boost_psi,"
        "IAT_F,Distance_ft,Lat,Lng");
  }

  eraseLightTree();
  drawLaunchBanner();
}


// ================================================================
//  RESET RUN
// ================================================================
void resetRun() {
  if (is_launched && logFile) {
    if (!sd_log_failed && sd_buffer_index > 0) {
      for (uint8_t i = 0; i < sd_buffer_index; i++) logFile.println(sd_buffer[i]);
      logFile.flush();
    }
    logFile.close();
  }

  is_launched         = false;
  reached_60          = false;
  reached_14_mile     = false;
  time_0_60           = 0.0f;
  time_14_mile        = 0.0f;
  trap_speed_mph      = 0.0f;
  peak_boost_psi      = 0.0f;
  peak_rpm            = 0.0f;
  reaction_time_s     = -1.0f;
  distance_traveled_m = 0.0f;
  launch_start_time   = 0;
  sd_log_failed       = false;
  sd_buffer_index     = 0;
  lastLat             = 0.0;
  lastLng             = 0.0;
  iat_valid           = false;
  intake_temp_f       = -999.0f;
  poll_counter        = 0;
  tree_state          = TREE_IDLE;
  last_boost_psi      = -999.0f;
  last_rpm_val        = -1.0f;
  old_peak_rpm        = -1.0f;

  drawBaseUI();
}


// ================================================================
//  DTC LOOKUP TABLE  (PROGMEM)
//  Sorted ascending by numeric code so binary search works.
//  Code encoding: P0xxx = 0x0xxx, P1xxx = 0x1xxx, P2xxx = 0x2xxx,
//                 P3xxx = 0x3xxx, B0xxx = 0x4xxx, B1xxx = 0x5xxx,
//                 C0xxx = 0x8xxx, U0xxx = 0xC000+
//  formatDTC() encodes the two raw OBD2 bytes into this scheme.
// ================================================================
struct DTCEntry {
  uint16_t    code;
  const char* desc;
};

// P0xxx — SAE Generic Powertrain
static const DTCEntry PROGMEM dtc_table[] = {
  {0x0010,"Camshaft Position Actuator A Circuit Bank 1"},
  {0x0011,"Camshaft Position Timing Over-Advanced Bank 1"},
  {0x0012,"Camshaft Position Timing Over-Retarded Bank 1"},
  {0x0013,"Camshaft Position Actuator Circuit Bank 1"},
  {0x0014,"Camshaft Position Actuator A Circuit Open Bank 1"},
  {0x0015,"Camshaft Position Actuator A Circuit Low Bank 1"},
  {0x0016,"Camshaft Position Actuator A Circuit High Bank 1"},
  {0x0020,"Camshaft Position Actuator A Circuit Bank 2"},
  {0x0021,"Camshaft Position Timing Over-Advanced Bank 2"},
  {0x0022,"Camshaft Position Timing Over-Retarded Bank 2"},
  {0x0023,"Camshaft Position Actuator Circuit Bank 2"},
  {0x0024,"Camshaft Position Actuator A Circuit Open Bank 2"},
  {0x0025,"Camshaft Position Actuator A Circuit Low Bank 2"},
  {0x0026,"Camshaft Position Actuator A Circuit High Bank 2"},
  {0x0100,"Mass or Volume Air Flow Sensor A Circuit"},
  {0x0101,"Mass or Volume Air Flow Sensor A Circuit Range/Performance"},
  {0x0102,"Mass or Volume Air Flow Sensor A Circuit Low"},
  {0x0103,"Mass or Volume Air Flow Sensor A Circuit High"},
  {0x0104,"Mass or Volume Air Flow Sensor A Circuit Intermittent"},
  {0x0105,"Manifold Absolute Pressure/Barometric Pressure Sensor Circuit"},
  {0x0106,"MAP/Baro Sensor Circuit Range/Performance"},
  {0x0107,"MAP/Baro Sensor Circuit Low"},
  {0x0108,"MAP/Baro Sensor Circuit High"},
  {0x0109,"MAP/Baro Sensor Circuit Intermittent"},
  {0x010A,"Mass or Volume Air Flow Sensor B Circuit"},
  {0x010B,"Mass or Volume Air Flow Sensor B Circuit Range/Performance"},
  {0x010C,"Mass or Volume Air Flow Sensor B Circuit Low"},
  {0x010D,"Mass or Volume Air Flow Sensor B Circuit High"},
  {0x010E,"Mass or Volume Air Flow Sensor B Circuit Intermittent"},
  {0x010F,"Intake Air Temperature Sensor 1 Circuit"},
  {0x0110,"Intake Air Temperature Sensor 1 Circuit Range/Performance"},
  {0x0111,"Intake Air Temperature Sensor 1 Circuit Low"},
  {0x0112,"Intake Air Temperature Sensor 1 Circuit High"},
  {0x0113,"Intake Air Temperature Sensor 1 Circuit Intermittent"},
  {0x0114,"Intake Air Temperature Sensor 2 Circuit"},
  {0x0115,"Intake Air Temperature Sensor 2 Circuit Range/Performance"},
  {0x0116,"Intake Air Temperature Sensor 2 Circuit Low"},
  {0x0117,"Intake Air Temperature Sensor 2 Circuit High"},
  {0x0118,"Intake Air Temperature Sensor 2 Circuit Intermittent"},
  {0x011A,"Coolant Temp / Air Temperature Sensor Cross Circuit"},
  {0x011B,"Coolant Temp Sensor B Circuit"},
  {0x011C,"Coolant Temp Sensor B Range/Performance"},
  {0x011D,"Coolant Temp Sensor B Circuit Low"},
  {0x011E,"Coolant Temp Sensor B Circuit High"},
  {0x011F,"Coolant Temp Sensor B Circuit Intermittent"},
  {0x0120,"Throttle/Pedal Position Sensor A Circuit"},
  {0x0121,"Throttle/Pedal Position Sensor A Range/Performance"},
  {0x0122,"Throttle/Pedal Position Sensor A Low"},
  {0x0123,"Throttle/Pedal Position Sensor A High"},
  {0x0124,"Throttle/Pedal Position Sensor A Intermittent"},
  {0x0125,"Insufficient Coolant Temperature for Closed Loop Fuel Control"},
  {0x0128,"Coolant Temperature Too Low for Closed Loop Fuel Control"},
  {0x0129,"Coolant Temperature Too High"},
  {0x012A,"Insufficient Coolant Temperature for Stable Operation"},
  {0x012B,"Coolant Temperature Sensor Circuit Intermittent"},
  {0x012C,"Insufficient Coolant Temperature for Fuel Enrichment"},
  {0x0130,"O2 Sensor Circuit Bank 1 Sensor 1"},
  {0x0131,"O2 Sensor Circuit Low Voltage Bank 1 Sensor 1"},
  {0x0132,"O2 Sensor Circuit High Voltage Bank 1 Sensor 1"},
  {0x0133,"O2 Sensor Circuit Slow Response Bank 1 Sensor 1"},
  {0x0134,"O2 Sensor Circuit No Activity Bank 1 Sensor 1"},
  {0x0135,"O2 Sensor Heater Circuit Bank 1 Sensor 1"},
  {0x0136,"O2 Sensor Circuit Bank 1 Sensor 2"},
  {0x0137,"O2 Sensor Circuit Low Voltage Bank 1 Sensor 2"},
  {0x0138,"O2 Sensor Circuit High Voltage Bank 1 Sensor 2"},
  {0x0139,"O2 Sensor Circuit Slow Response Bank 1 Sensor 2"},
  {0x013A,"O2 Sensor Circuit No Activity Bank 1 Sensor 2"},
  {0x013B,"O2 Sensor Heater Circuit Bank 1 Sensor 2"},
  {0x013C,"O2 Sensor Circuit Bank 1 Sensor 3"},
  {0x013D,"O2 Sensor Circuit Low Voltage Bank 1 Sensor 3"},
  {0x013E,"O2 Sensor Circuit High Voltage Bank 1 Sensor 3"},
  {0x013F,"O2 Sensor Circuit Slow Response Bank 1 Sensor 3"},
  {0x0150,"O2 Sensor Circuit Bank 2 Sensor 1"},
  {0x0151,"O2 Sensor Circuit Low Voltage Bank 2 Sensor 1"},
  {0x0152,"O2 Sensor Circuit High Voltage Bank 2 Sensor 1"},
  {0x0153,"O2 Sensor Circuit Slow Response Bank 2 Sensor 1"},
  {0x0154,"O2 Sensor Circuit No Activity Bank 2 Sensor 1"},
  {0x0155,"O2 Sensor Heater Circuit Bank 2 Sensor 1"},
  {0x0156,"O2 Sensor Circuit Bank 2 Sensor 2"},
  {0x0157,"O2 Sensor Circuit Low Voltage Bank 2 Sensor 2"},
  {0x0158,"O2 Sensor Circuit High Voltage Bank 2 Sensor 2"},
  {0x0159,"O2 Sensor Circuit Slow Response Bank 2 Sensor 2"},
  {0x015A,"O2 Sensor Circuit No Activity Bank 2 Sensor 2"},
  {0x015B,"O2 Sensor Heater Circuit Bank 2 Sensor 2"},
  {0x0160,"O2 Sensor Circuit Bank 2 Sensor 3"},
  {0x0161,"O2 Sensor Circuit Low Voltage Bank 2 Sensor 3"},
  {0x0162,"O2 Sensor Circuit High Voltage Bank 2 Sensor 3"},
  {0x0163,"O2 Sensor Circuit Slow Response Bank 2 Sensor 3"},
  {0x0170,"Fuel Trim Malfunction Bank 1"},
  {0x0171,"System Too Lean Bank 1"},
  {0x0172,"System Too Rich Bank 1"},
  {0x0173,"System Too Lean Bank 2"},
  {0x0174,"System Too Rich Bank 2"},
  {0x0175,"System Too Lean Bank 3"},
  {0x0176,"System Too Rich Bank 3"},
  {0x0177,"System Too Lean Bank 4"},
  {0x0178,"System Too Rich Bank 4"},
  {0x0179,"Fuel Trim Malfunction Bank 2"},
  {0x017A,"Fuel Trim Malfunction Bank 3"},
  {0x017B,"Fuel Trim Malfunction Bank 4"},
  {0x0201,"Injector Circuit Open — Cylinder 1"},
  {0x0202,"Injector Circuit Open — Cylinder 2"},
  {0x0203,"Injector Circuit Open — Cylinder 3"},
  {0x0204,"Injector Circuit Open — Cylinder 4"},
  {0x0205,"Injector Circuit Open — Cylinder 5"},
  {0x0206,"Injector Circuit Open — Cylinder 6"},
  {0x0207,"Injector Circuit Open — Cylinder 7"},
  {0x0208,"Injector Circuit Open — Cylinder 8"},
  {0x0220,"Throttle/Pedal Position Sensor B Circuit"},
  {0x0221,"Throttle/Pedal Position Sensor B Range/Performance"},
  {0x0222,"Throttle/Pedal Position Sensor B Low"},
  {0x0223,"Throttle/Pedal Position Sensor B High"},
  {0x0224,"Throttle/Pedal Position Sensor B Intermittent"},
  {0x0234,"Turbo/Supercharger Overboost Condition"},
  {0x0235,"Turbo/Supercharger Boost Sensor A Circuit"},
  {0x0236,"Turbo/Supercharger Boost Sensor A Circuit Range/Performance"},
  {0x0237,"Turbo/Supercharger Boost Sensor A Circuit Low"},
  {0x0238,"Turbo/Supercharger Boost Sensor A Circuit High"},
  {0x0239,"Turbo/Supercharger Overboost Condition Bank 2"},
  {0x0243,"Wastegate Solenoid A Circuit"},
  {0x0244,"Wastegate Solenoid A Circuit Range/Performance"},
  {0x0245,"Wastegate Solenoid A Circuit Low"},
  {0x0246,"Wastegate Solenoid A Circuit High"},
  {0x0247,"Wastegate Solenoid B Circuit"},
  {0x0248,"Wastegate Solenoid B Circuit Range/Performance"},
  {0x0249,"Wastegate Solenoid B Circuit Low"},
  {0x024A,"Wastegate Solenoid B Circuit High"},
  {0x0261,"Injector Circuit Low — Cylinder 1"},
  {0x0262,"Injector Circuit High — Cylinder 1"},
  {0x0263,"Injector Circuit Low — Cylinder 2"},
  {0x0264,"Injector Circuit High — Cylinder 2"},
  {0x0265,"Injector Circuit Low — Cylinder 3"},
  {0x0266,"Injector Circuit High — Cylinder 3"},
  {0x0267,"Injector Circuit Low — Cylinder 4"},
  {0x0268,"Injector Circuit High — Cylinder 4"},
  {0x0269,"Injector Circuit Low — Cylinder 5"},
  {0x0270,"Injector Circuit High — Cylinder 5"},
  {0x0271,"Injector Circuit Low — Cylinder 6"},
  {0x0272,"Injector Circuit High — Cylinder 6"},
  {0x0273,"Injector Circuit Low — Cylinder 7"},
  {0x0274,"Injector Circuit High — Cylinder 7"},
  {0x0275,"Injector Circuit Low — Cylinder 8"},
  {0x0276,"Injector Circuit High — Cylinder 8"},
  {0x0299,"Turbo/Supercharger Underboost Condition"},
  {0x02B0,"Charge Air Cooler Coolant Pump Control Circuit"},
  {0x02B1,"Charge Air Cooler Coolant Pump Control Circuit Range/Performance"},
  {0x02B2,"Charge Air Cooler Coolant Pump Control Circuit Low"},
  {0x02B3,"Charge Air Cooler Coolant Pump Control Circuit High"},
  {0x0300,"Random/Multiple Cylinder Misfire Detected"},
  {0x0301,"Cylinder 1 Misfire Detected"},
  {0x0302,"Cylinder 2 Misfire Detected"},
  {0x0303,"Cylinder 3 Misfire Detected"},
  {0x0304,"Cylinder 4 Misfire Detected"},
  {0x0305,"Cylinder 5 Misfire Detected"},
  {0x0306,"Cylinder 6 Misfire Detected"},
  {0x0307,"Cylinder 7 Misfire Detected"},
  {0x0308,"Cylinder 8 Misfire Detected"},
  {0x0309,"Cylinder 9 Misfire Detected"},
  {0x030A,"Cylinder 10 Misfire Detected"},
  {0x030B,"Cylinder 11 Misfire Detected"},
  {0x030C,"Cylinder 12 Misfire Detected"},
  {0x030D,"Misfire Detected — too low to determine cylinder"},
  {0x030E,"Misfire Pattern Fault"},
  {0x0325,"Knock Sensor 1 Circuit Bank 1"},
  {0x0326,"Knock Sensor 1 Circuit Range/Performance Bank 1"},
  {0x0327,"Knock Sensor 1 Circuit Low Bank 1"},
  {0x0328,"Knock Sensor 1 Circuit High Bank 1"},
  {0x0329,"Knock Sensor 1 Circuit Intermittent Bank 1"},
  {0x032A,"Knock Sensor 2 Circuit Bank 2"},
  {0x032B,"Knock Sensor 2 Circuit Range/Performance Bank 2"},
  {0x032C,"Knock Sensor 2 Circuit Low Bank 2"},
  {0x032D,"Knock Sensor 2 Circuit High Bank 2"},
  {0x032E,"Knock Sensor 2 Circuit Intermittent Bank 2"},
  {0x0335,"Crankshaft Position Sensor A Circuit"},
  {0x0336,"Crankshaft Position Sensor A Circuit Range/Performance"},
  {0x0337,"Crankshaft Position Sensor A Circuit No Signal"},
  {0x0338,"Crankshaft Position Sensor A Circuit Intermittent"},
  {0x0339,"Crankshaft Position Sensor B Circuit"},
  {0x033A,"Crankshaft Position Sensor B Circuit Range/Performance"},
  {0x033B,"Crankshaft Position Sensor B Circuit No Signal"},
  {0x033C,"Crankshaft Position Sensor B Circuit Intermittent"},
  {0x0340,"Camshaft Position Sensor A Circuit Bank 1"},
  {0x0341,"Camshaft Position Sensor A Circuit Range/Performance B1"},
  {0x0342,"Camshaft Position Sensor A Circuit Low Bank 1"},
  {0x0343,"Camshaft Position Sensor A Circuit High Bank 1"},
  {0x0344,"Camshaft Position Sensor A Circuit Intermittent Bank 1"},
  {0x0345,"Camshaft Position Sensor A Circuit Bank 2"},
  {0x0346,"Camshaft Position Sensor A Circuit Range/Performance B2"},
  {0x0347,"Camshaft Position Sensor A Circuit Low Bank 2"},
  {0x0348,"Camshaft Position Sensor A Circuit High Bank 2"},
  {0x0349,"Camshaft Position Sensor A Circuit Intermittent Bank 2"},
  {0x034A,"Camshaft Position Sensor B Circuit Bank 1"},
  {0x034B,"Camshaft Position Sensor B Circuit Range/Performance B1"},
  {0x034C,"Camshaft Position Sensor B Circuit Low Bank 1"},
  {0x034D,"Camshaft Position Sensor B Circuit High Bank 1"},
  {0x034E,"Camshaft Position Sensor B Circuit Intermittent Bank 1"},
  {0x034F,"Camshaft Position Sensor B Circuit Bank 2"},
  {0x0400,"Exhaust Gas Recirculation Flow Malfunction"},
  {0x0401,"Exhaust Gas Recirculation Flow Insufficient"},
  {0x0402,"Exhaust Gas Recirculation Flow Excessive"},
  {0x0403,"Exhaust Gas Recirculation Circuit"},
  {0x0404,"Exhaust Gas Recirculation Circuit Range/Performance"},
  {0x0405,"Exhaust Gas Recirculation Sensor A Circuit Low"},
  {0x0406,"Exhaust Gas Recirculation Sensor A Circuit High"},
  {0x0407,"Exhaust Gas Recirculation Sensor B Circuit Low"},
  {0x0408,"Exhaust Gas Recirculation Sensor B Circuit High"},
  {0x0420,"Catalyst System Efficiency Below Threshold Bank 1"},
  {0x0421,"Warm Up Catalyst Efficiency Below Threshold Bank 1"},
  {0x0422,"Main Catalyst Efficiency Below Threshold Bank 1"},
  {0x0423,"Heated Catalyst Efficiency Below Threshold Bank 1"},
  {0x0424,"Catalyst Temperature Below Threshold Bank 1"},
  {0x0425,"Catalyst Temperature Above Threshold Bank 1"},
  {0x0426,"Catalyst Temperature Sensor Circuit Bank 1 Sensor 1"},
  {0x0427,"Catalyst Temperature Sensor Circuit Low Bank 1 Sensor 1"},
  {0x0428,"Catalyst Temperature Sensor Circuit High Bank 1 Sensor 1"},
  {0x0429,"Catalyst Temperature Sensor Circuit Intermittent B1 S1"},
  {0x042A,"Catalyst Temperature Sensor Circuit Bank 1 Sensor 2"},
  {0x042B,"Catalyst Temperature Sensor Circuit Low Bank 1 Sensor 2"},
  {0x042C,"Catalyst Temperature Sensor Circuit High Bank 1 Sensor 2"},
  {0x042D,"Catalyst Temperature Sensor Circuit Intermittent B1 S2"},
  {0x0430,"Catalyst System Efficiency Below Threshold Bank 2"},
  {0x0431,"Warm Up Catalyst Efficiency Below Threshold Bank 2"},
  {0x0432,"Main Catalyst Efficiency Below Threshold Bank 2"},
  {0x0433,"Heated Catalyst Efficiency Below Threshold Bank 2"},
  {0x0434,"Catalyst Temperature Below Threshold Bank 2"},
  {0x0435,"Catalyst Temperature Above Threshold Bank 2"},
  {0x0440,"Evaporative Emission System Malfunction"},
  {0x0441,"Evaporative Emission System Incorrect Purge Flow"},
  {0x0442,"Evaporative Emission System Small Leak"},
  {0x0443,"Evaporative Emission System Purge Control Valve Circuit"},
  {0x0444,"Evaporative Emission System Purge Control Valve Circuit Open"},
  {0x0445,"Evaporative Emission System Purge Control Valve Circuit Shorted"},
  {0x0446,"Evaporative Emission System Vent Control Circuit"},
  {0x0447,"Evaporative Emission System Vent Control Circuit Open"},
  {0x0448,"Evaporative Emission System Vent Control Circuit Shorted"},
  {0x0449,"Evaporative Emission System Vent Valve/Solenoid Circuit"},
  {0x044A,"Evaporative Emission System Vent Valve/Solenoid Circuit Open"},
  {0x044B,"Evaporative Emission System Vent Valve/Solenoid Circuit Shorted"},
  {0x044C,"Evaporative Emission System Pressure Sensor Circuit"},
  {0x044D,"Evaporative Emission System Pressure Sensor Low"},
  {0x044E,"Evaporative Emission System Pressure Sensor High"},
  {0x044F,"Evaporative Emission System Condition Not Present with VSS"},
  {0x0450,"Evaporative Emission System Pressure Sensor/Switch"},
  {0x0451,"Evaporative Emission System Pressure Sensor Range/Performance"},
  {0x0452,"Evaporative Emission System Pressure Sensor Low"},
  {0x0453,"Evaporative Emission System Pressure Sensor High"},
  {0x0454,"Evaporative Emission System Pressure Sensor Intermittent"},
  {0x0455,"Evaporative Emission System Large Leak"},
  {0x0456,"Evaporative Emission System Very Small Leak"},
  {0x0457,"Evaporative Emission System Leak Detected — Fuel Cap"},
  {0x0458,"Evaporative Emission System Purge Control Valve A Circuit Low"},
  {0x0459,"Evaporative Emission System Purge Control Valve A Circuit High"},
  {0x0500,"Vehicle Speed Sensor Malfunction"},
  {0x0501,"Vehicle Speed Sensor Range/Performance"},
  {0x0502,"Vehicle Speed Sensor Circuit Low"},
  {0x0503,"Vehicle Speed Sensor Circuit High"},
  {0x0504,"Vehicle Speed Sensor Intermittent"},
  {0x0505,"Idle Control System RPM Lower Than Expected"},
  {0x0506,"Idle Control System RPM Higher Than Expected"},
  {0x0507,"Idle Air Control System RPM Higher Than Expected"},
  {0x050A,"Idle Control System Malfunction"},
  {0x050B,"Idle Air Control Circuit"},
  {0x050C,"Idle Air Control Circuit Range/Performance"},
  {0x050D,"Idle Air Control Circuit Low"},
  {0x050E,"Idle Air Control Circuit High"},
  {0x052A,"Oil Level Sensor Circuit"},
  {0x052B,"Oil Level Sensor Circuit Range/Performance"},
  {0x052C,"Oil Level Sensor Circuit Low"},
  {0x052D,"Oil Level Sensor Circuit High"},
  {0x052E,"Oil Level Sensor Circuit Intermittent"},
  {0x052F,"Oil Pressure Sensor/Switch Circuit"},
  {0x0600,"Serial Communication Link"},
  {0x0601,"Internal Control Module Memory Check Sum Error"},
  {0x0602,"Control Module Programming Error"},
  {0x0603,"Internal Control Module Keep Alive Memory Error"},
  {0x0604,"Internal Control Module RAM Error"},
  {0x0605,"Internal Control Module ROM Error"},
  {0x0700,"Transmission Control System Malfunction"},
  {0x0701,"Transmission Control System Range/Performance"},
  {0x0702,"Transmission Control System Electrical"},
  {0x0703,"Torque Converter/Brake Switch B Circuit"},
  {0x0704,"Clutch Switch Input Circuit Malfunction"},
  {0x0705,"Transmission Range Sensor Circuit Malfunction"},
  {0x0706,"Transmission Range Sensor Circuit Range/Performance"},
  {0x0707,"Transmission Range Sensor Circuit Low"},
  {0x0708,"Transmission Range Sensor Circuit High"},
  {0x0709,"Transmission Range Sensor Circuit Intermittent"},
  {0x070A,"Transmission Fluid Temp Sensor A Circuit"},
  {0x070B,"Transmission Fluid Temp Sensor A Circuit Range/Performance"},
  {0x070C,"Transmission Fluid Temp Sensor A Circuit Low"},
  {0x070D,"Transmission Fluid Temp Sensor A Circuit High"},
  {0x070E,"Transmission Fluid Temp Sensor A Circuit Intermittent"},
  {0x070F,"Torque Converter Clutch Solenoid Circuit Performance"},
  {0x0710,"Transmission Fluid Temperature Sensor Circuit Malfunction"},
  {0x0711,"Transmission Fluid Temp Sensor Circuit Range/Performance"},
  {0x0712,"Transmission Fluid Temp Sensor Circuit Low"},
  {0x0713,"Transmission Fluid Temp Sensor Circuit High"},
  {0x0714,"Transmission Fluid Temp Sensor Circuit Intermittent"},
  {0x0715,"Torque Converter Clutch Solenoid Circuit"},
  {0x0716,"Torque Converter Clutch Solenoid Circuit Range/Performance"},
  {0x0717,"Torque Converter Clutch Solenoid Circuit Low"},
  {0x0718,"Torque Converter Clutch Solenoid Circuit High"},
  {0x0719,"Torque Converter Clutch Solenoid Circuit Intermittent"},
  {0x0720,"Output Speed Sensor Circuit"},
  {0x0721,"Output Speed Sensor Circuit Range/Performance"},
  {0x0722,"Output Speed Sensor Circuit No Signal"},
  {0x0723,"Output Speed Sensor Circuit Intermittent"},
  {0x0724,"Torque Converter Clutch Solenoid Circuit High"},
  {0x0725,"Engine Speed Input Circuit"},
  {0x0726,"Engine Speed Input Circuit Range/Performance"},
  {0x0727,"Engine Speed Input Circuit No Signal"},
  {0x0728,"Engine Speed Input Circuit Intermittent"},
  {0x0729,"Torque Converter Clutch Solenoid Circuit Low"},
  {0x0730,"Gear Ratio Error"},
  {0x0731,"Gear 1 Incorrect Ratio"},
  {0x0732,"Gear 2 Incorrect Ratio"},
  {0x0733,"Gear 3 Incorrect Ratio"},
  {0x0734,"Gear 4 Incorrect Ratio"},
  {0x0735,"Gear 5 Incorrect Ratio"},
  {0x0736,"Gear 6 Incorrect Ratio"},
  {0x0740,"Torque Converter Clutch System Performance or Stuck Off"},
  {0x0741,"Torque Converter Clutch System Performance"},
  {0x0742,"Torque Converter Clutch System Stuck On"},
  {0x0743,"Torque Converter Clutch Circuit Electrical"},
  {0x0744,"Torque Converter Clutch Circuit Intermittent"},
  {0x0745,"Pressure Control Solenoid A Electrical"},
  {0x0746,"Pressure Control Solenoid A Performance or Stuck Off"},
  {0x0747,"Pressure Control Solenoid A Stuck On"},
  {0x0748,"Pressure Control Solenoid A Circuit Low"},
  {0x0749,"Pressure Control Solenoid A Circuit High"},
  {0x0750,"Shift Solenoid A Malfunction"},
  {0x0751,"Shift Solenoid A Performance or Stuck Off"},
  {0x0752,"Shift Solenoid A Stuck On"},
  {0x0753,"Shift Solenoid A Electrical"},
  {0x0754,"Shift Solenoid A Intermittent"},
  {0x0755,"Shift Solenoid B Malfunction"},
  {0x0756,"Shift Solenoid B Performance or Stuck Off"},
  {0x0757,"Shift Solenoid B Stuck On"},
  {0x0758,"Shift Solenoid B Electrical"},
  {0x0759,"Shift Solenoid B Intermittent"},
  {0x0760,"Shift Solenoid C Malfunction"},
  {0x0761,"Shift Solenoid C Performance or Stuck Off"},
  {0x0762,"Shift Solenoid C Stuck On"},
  {0x0763,"Shift Solenoid C Electrical"},
  {0x0764,"Shift Solenoid C Intermittent"},
  {0x0765,"Shift Solenoid D Malfunction"},
  {0x0766,"Shift Solenoid D Performance or Stuck Off"},
  {0x0767,"Shift Solenoid D Stuck On"},
  {0x0768,"Shift Solenoid D Electrical"},
  {0x0769,"Shift Solenoid D Intermittent"},
  {0x0770,"Shift Solenoid E Malfunction"},
  {0x0771,"Shift Solenoid E Performance or Stuck Off"},
  {0x0772,"Shift Solenoid E Stuck On"},
  {0x0773,"Shift Solenoid E Electrical"},
  {0x0774,"Shift Solenoid E Intermittent"},
  {0x0775,"Pressure Control Solenoid B Electrical"},
  {0x0776,"Pressure Control Solenoid B Performance or Stuck Off"},
  {0x0777,"Pressure Control Solenoid B Stuck On"},
  {0x0778,"Pressure Control Solenoid B Circuit Low"},
  {0x0779,"Pressure Control Solenoid B Circuit High"},
  {0x0780,"Shift Malfunction"},
  {0x0781,"1-2 Shift Malfunction"},
  {0x0782,"2-3 Shift Malfunction"},
  {0x0783,"3-4 Shift Malfunction"},
  {0x0784,"4-5 Shift Malfunction"},
  {0x0785,"Shift/Timing Solenoid Malfunction"},
  {0x0786,"Shift/Timing Solenoid Range/Performance"},
  {0x0787,"Shift/Timing Solenoid Low"},
  {0x0788,"Shift/Timing Solenoid High"},
  {0x0789,"Shift/Timing Solenoid Intermittent"},
  {0x0790,"Normal/Performance Switch Circuit"},
  {0x2096,"Post Catalyst Fuel Trim System Too Lean Bank 1"},
  {0x2097,"Post Catalyst Fuel Trim System Too Rich Bank 1"},
  {0x2098,"Post Catalyst Fuel Trim System Too Lean Bank 2"},
  {0x2099,"Post Catalyst Fuel Trim System Too Rich Bank 2"},
  {0x2100,"Throttle Actuator Control System — Sudden High Idle"},
  {0x2101,"Throttle Actuator Control System — High Idle"},
  {0x2102,"Throttle Actuator Control System — Low Idle"},
  {0x2103,"Throttle Actuator Control System — Forced Engine Shutdown"},
  {0x2104,"Throttle Actuator Control System — Forced Reduced Power"},
  {0x2105,"Throttle Actuator Control System — Maximum Engine Speed"},
  {0x2106,"Throttle Actuator Control System — Mechanical Stuck Open"},
  {0x2107,"Throttle Actuator Control System — Mechanical Stuck Closed"},
  {0x2108,"Throttle Actuator Control System — High Engine Speed"},
  {0x2111,"Throttle Actuator Control System Stuck Open"},
  {0x2112,"Throttle Actuator Control System Stuck Closed"},
  {0x2118,"Throttle Actuator Control Motor Current Range/Performance"},
  {0x2119,"Throttle Actuator Control Throttle Body Range/Performance"},
  {0x2120,"Throttle/Pedal Position Sensor/Switch D Circuit"},
  {0x2121,"Throttle/Pedal Position Sensor D Range/Performance"},
  {0x2122,"Throttle/Pedal Position Sensor D Low"},
  {0x2123,"Throttle/Pedal Position Sensor D High"},
  {0x2127,"Throttle/Pedal Position Sensor E Low"},
  {0x2128,"Throttle/Pedal Position Sensor E High"},
  {0x2135,"Throttle/Pedal Position Sensor A/B Voltage Correlation"},
  {0x2138,"Throttle/Pedal Position Sensor D/E Voltage Correlation"},
  {0x2176,"Throttle Actuator Control System Idle Position Not Learned"},
  {0x2187,"System Too Lean at Idle Bank 1"},
  {0x2188,"System Too Rich at Idle Bank 1"},
  {0x2189,"System Too Lean at Idle Bank 2"},
  {0x2190,"System Too Rich at Idle Bank 2"},
  {0x2191,"System Too Lean at Higher Load Bank 1"},
  {0x2192,"System Too Rich at Higher Load Bank 1"},
  {0x2195,"O2 Sensor Signal Biased/Stuck Lean Bank 1 Sensor 1"},
  {0x2196,"O2 Sensor Signal Biased/Stuck Rich Bank 1 Sensor 1"},
  {0x2197,"O2 Sensor Signal Biased/Stuck Lean Bank 2 Sensor 1"},
  {0x2198,"O2 Sensor Signal Biased/Stuck Rich Bank 2 Sensor 1"},
  {0x2226,"Barometric Pressure Circuit Low"},
  {0x2227,"Barometric Pressure Circuit Range/Performance"},
  {0x2228,"Barometric Pressure Circuit High"},
  {0x2229,"Barometric Pressure Too High"},
  {0x2231,"Short to Ground in Heated Oxygen Sensor Heater B1 S1"},
  {0x2232,"Short to Ground in Heated Oxygen Sensor Heater B1 S2"},
  {0x2237,"Upstream Oxygen Sensor Heater Current B1 S1 Low"},
  {0x2238,"Upstream Oxygen Sensor Heater Current B1 S1 High"},
  {0x2240,"Upstream Oxygen Sensor Heater Current B2 S1 Low"},
  {0x2241,"Upstream Oxygen Sensor Heater Current B2 S1 High"},
  {0x2270,"O2 Sensor Signal Stuck Rich Bank 1 Sensor 2"},
  {0x2271,"O2 Sensor Signal Stuck Lean Bank 1 Sensor 2"},
  {0x2272,"O2 Sensor Signal Stuck Rich Bank 2 Sensor 2"},
  {0x2273,"O2 Sensor Signal Stuck Lean Bank 2 Sensor 2"},
  {0x2300,"Ignition Coil A Primary Control Circuit Low"},
  {0x2301,"Ignition Coil A Primary Control Circuit High"},
  {0x2302,"Ignition Coil B Primary Control Circuit Low"},
  {0x2303,"Ignition Coil B Primary Control Circuit High"},
  {0x2304,"Ignition Coil C Primary Control Circuit Low"},
  {0x2305,"Ignition Coil C Primary Control Circuit High"},
  {0x2306,"Ignition Coil D Primary Control Circuit Low"},
  {0x2307,"Ignition Coil D Primary Control Circuit High"},
  {0x2308,"Ignition Coil E Primary Control Circuit Low"},
  {0x2309,"Ignition Coil E Primary Control Circuit High"},
  {0x2310,"Ignition Coil F Primary Control Circuit Low"},
  {0x2311,"Ignition Coil F Primary Control Circuit High"},
  {0x2312,"Ignition Coil G Primary Control Circuit Low"},
  {0x2313,"Ignition Coil G Primary Control Circuit High"},
  {0x2314,"Ignition Coil H Primary Control Circuit Low"},
  {0x2315,"Ignition Coil H Primary Control Circuit High"},
  {0x3000,"Transmission Control Module Requested MIL Illumination"},
  {0x3190,"Engine Oil Pressure Too Low"},
  {0x3191,"Engine Oil Pressure Too High"},
  {0x3400,"Deactivated Cylinder Mode System Performance"},
  {0x3401,"Cylinder 1 Deactivation/Intake Valve Control Circuit Low"},
  {0x3402,"Cylinder 1 Deactivation/Intake Valve Control Circuit High"},
  {0x3403,"Cylinder 2 Deactivation/Intake Valve Control Circuit Low"},
  {0x3404,"Cylinder 2 Deactivation/Intake Valve Control Circuit High"},
  {0x3405,"Cylinder 3 Deactivation/Intake Valve Control Circuit Low"},
  {0x3406,"Cylinder 3 Deactivation/Intake Valve Control Circuit High"},
  {0x3407,"Cylinder 4 Deactivation/Intake Valve Control Circuit Low"},
  {0x3408,"Cylinder 4 Deactivation/Intake Valve Control Circuit High"},
  {0x4001,"Controller Area Network (CAN) Bus Communication Fault"},
  {0x4010,"Airbag Warning Lamp Circuit"},
  {0x4011,"Driver Frontal Stage 1 Deployment Control"},
  {0x4012,"Driver Frontal Stage 2 Deployment Control"},
  {0x4013,"Passenger Frontal Stage 1 Deployment Control"},
  {0x4014,"Passenger Frontal Stage 2 Deployment Control"},
  {0x4020,"Driver Side Airbag Stage 1 Squib Circuit"},
  {0x4021,"Driver Side Airbag Stage 1 Squib Resistance Low"},
  {0x4022,"Driver Side Airbag Stage 1 Squib Resistance High"},
  {0x4023,"Driver Side Airbag Stage 1 Squib Short to Ground"},
  {0x4024,"Driver Side Airbag Stage 1 Squib Short to Battery"},
  {0x4025,"Driver Side Airbag Stage 1 Squib Open Circuit"},
  {0x4030,"Passenger Airbag Stage 1 Squib Circuit"},
  {0x4031,"Passenger Airbag Stage 1 Squib Resistance Low"},
  {0x4032,"Passenger Airbag Stage 1 Squib Resistance High"},
  {0x4033,"Passenger Airbag Stage 1 Squib Short to Ground"},
  {0x4034,"Passenger Airbag Stage 1 Squib Short to Battery"},
  {0x4035,"Passenger Airbag Stage 1 Squib Open Circuit"},
  {0x4055,"Driver Side Airbag Squib 2 Circuit"},
  {0x4056,"Passenger Side Airbag Squib 2 Circuit"},
  {0x40A0,"Air Conditioning System Performance"},
  {0x40A4,"HVAC Control Module Circuit"},
  {0x40B0,"Climate Control Actuator Circuit"},
  {0x40D0,"Door Lock Control Circuit"},
  {0x40D1,"Door Unlock Control Circuit"},
  {0x40E0,"Horn Output Circuit"},
  {0x40F0,"Power Window Circuit"},
  {0x40F4,"Sunroof Control Circuit"},
  {0x5000,"Serial Communication Link Failure"},
  {0x5010,"Fuel Sender Circuit"},
  {0x5011,"Fuel Level Sensor Circuit Range/Performance"},
  {0x5012,"Fuel Level Sensor Circuit Low"},
  {0x5013,"Fuel Level Sensor Circuit High"},
  {0x5020,"Traction Control/Stability Control Communication Fault"},
  {0x5025,"ABS Brake Booster Circuit"},
  {0x502A,"Driver Seat Position Sensor Circuit"},
  {0x5030,"Passenger Seat Position Sensor Circuit"},
  {0x5035,"Occupant Classification System Fault"},
  {0x5040,"Speed Control Actuator Circuit"},
  {0x5041,"Speed Control Switch Circuit"},
  {0x5042,"Brake Switch Circuit"},
  {0x5050,"Steering Wheel Position Sensor Circuit"},
  {0x5051,"Steering Wheel Angle Sensor Performance"},
  {0x5052,"Steering Wheel Angle Sensor Circuit Low"},
  {0x5053,"Steering Wheel Angle Sensor Circuit High"},
  {0x8000,"ABS Malfunction"},
  {0x8020,"Right Front Wheel Speed Sensor Circuit"},
  {0x8021,"Right Front Wheel Speed Sensor Range/Performance"},
  {0x8022,"Right Front Wheel Speed Sensor Low"},
  {0x8023,"Right Front Wheel Speed Sensor High"},
  {0x8024,"Right Front Wheel Speed Sensor Intermittent"},
  {0x8025,"Left Front Wheel Speed Sensor Circuit"},
  {0x8026,"Left Front Wheel Speed Sensor Range/Performance"},
  {0x8027,"Left Front Wheel Speed Sensor Low"},
  {0x8028,"Left Front Wheel Speed Sensor High"},
  {0x8029,"Left Front Wheel Speed Sensor Intermittent"},
  {0x8030,"Right Rear Wheel Speed Sensor Circuit"},
  {0x8031,"Right Rear Wheel Speed Sensor Range/Performance"},
  {0x8032,"Right Rear Wheel Speed Sensor Low"},
  {0x8033,"Right Rear Wheel Speed Sensor High"},
  {0x8034,"Right Rear Wheel Speed Sensor Intermittent"},
  {0x8035,"Left Rear Wheel Speed Sensor Circuit"},
  {0x8036,"Left Rear Wheel Speed Sensor Range/Performance"},
  {0x8037,"Left Rear Wheel Speed Sensor Low"},
  {0x8038,"Left Rear Wheel Speed Sensor High"},
  {0x8039,"Left Rear Wheel Speed Sensor Intermittent"},
  {0x8040,"Right Front ABS Solenoid 1 Circuit"},
  {0x8041,"Right Front ABS Solenoid 2 Circuit"},
  {0x8042,"Left Front ABS Solenoid 1 Circuit"},
  {0x8043,"Left Front ABS Solenoid 2 Circuit"},
  {0x8044,"Right Rear ABS Solenoid 1 Circuit"},
  {0x8045,"Right Rear ABS Solenoid 2 Circuit"},
  {0x8046,"Left Rear ABS Solenoid 1 Circuit"},
  {0x8047,"Left Rear ABS Solenoid 2 Circuit"},
  {0x8060,"Rear Axle System Malfunction"},
  {0x8061,"Rear Axle System Range/Performance"},
  {0x8062,"Rear Axle System Low"},
  {0x8063,"Rear Axle System High"},
  {0x8064,"Rear Axle System Slow"},
  {0x8070,"Suspension Control System"},
  {0x8076,"Traction Control System"},
  {0x8095,"Right Front Brake Circuit"},
  {0x8096,"Left Front Brake Circuit"},
  {0x8097,"Right Rear Brake Circuit"},
  {0x8098,"Right Rear Brake Circuit Low"},
  {0x8110,"Steering Angle Sensor 1 Performance"},
  {0x8111,"Steering Angle Sensor 1 Circuit"},
  {0x8121,"Traction Control Requested Spark Retard Above Calibration"},
  {0x8125,"Drive Motor A Temperature Too High"},
  {0x8126,"Drive Motor B Temperature Too High"},
  {0x8127,"Transmission Fluid Pressure Sensor/Switch A Circuit Low"},
  {0x8128,"Transmission Fluid Pressure Sensor/Switch A Circuit High"},
  {0x812A,"Front Differential Clutch Actuator Circuit"},
  {0x812B,"Rear Differential Clutch Actuator Circuit"},
  {0x8200,"Four-Wheel Drive System Malfunction"},
  {0x8201,"Transfer Case Position Sensor Circuit"},
  {0x8204,"Transfer Case Position Sensor Circuit High"},
  {0x8206,"Transfer Case Switch Circuits"},
  {0x8240,"Active Suspension Front Right High Solenoid"},
  {0x8241,"Active Suspension Front Left High Solenoid"},
  {0x8242,"Active Suspension Rear Right High Solenoid"},
  {0x8243,"Active Suspension Rear Left High Solenoid"},
  {0xC000,"Loss of Communication with ECM/PCM"},
  {0xC001,"Loss of Communication with ECM/PCM Message Counter Incorrect"},
  {0xC002,"Loss of Communication with ECM/PCM Missing Message"},
  {0xC010,"Loss of Communication with TCM"},
  {0xC020,"Loss of Communication with ABS System"},
  {0xC025,"Loss of Communication with Anti-Lock Brake System Control Module"},
  {0xC030,"Lost Communication with Instrument Panel Cluster Control Module"},
  {0xC040,"Lost Communication with Body Control Module"},
  {0xC041,"Lost Communication with Body Control Module A"},
  {0xC042,"Lost Communication with Body Control Module B"},
  {0xC050,"Lost Communication with Fuel Pump Control Module"},
  {0xC060,"Lost Communication with Transfer Case Control Module"},
  {0xC073,"Lost Communication with All-Wheel Drive Control Module"},
  {0xC074,"Lost Communication with Active Front Steering Control Module"},
  {0xC080,"Lost Communication with Cruise Control Module"},
  {0xC090,"Lost Communication with Airbag System"},
  {0xC094,"Lost Communication with Occupant Restraint Controller"},
  {0xC100,"Lost Communication with Air Suspension Control Module"},
  {0xC110,"Lost Communication with Suspension System Control Module"},
  {0xC130,"Lost Communication with Steering Control Module"},
  {0xC140,"Lost Communication with Electronic Throttle Control"},
  {0xC145,"Lost Communication with Tire Pressure Monitor Module"},
  {0xC146,"Lost Communication with Remote Keyless Entry Module"},
  {0xC151,"Lost Communication with Steering Column Control Module"},
  {0xC155,"Lost Communication with Seat Control Module A"},
  {0xC160,"Lost Communication with Battery Energy Control Module"},
  {0xC168,"Lost Communication with Charging System Control Module"},
  {0xC170,"Lost Communication with Drive Motor Control Module A"},
  {0xC172,"Lost Communication with Drive Motor Control Module B"},
  {0xC173,"Lost Communication with High Voltage Battery Pack Control Module"},
  {0xC174,"Lost Communication with Auxiliary Battery Energy Control Module"},
  {0xC175,"Lost Communication with Generator A"},
  {0xC200,"Lost Communication with Transmission Range Selector Module"}
};

static const int DTC_TABLE_SIZE = sizeof(dtc_table) / sizeof(dtc_table[0]);

// ----------------------------------------------------------------
// formatDTC: convert two OBD2 raw bytes → "P0301\0" style string
// Byte 0 bits[7:6] = category: 00=P, 01=C, 10=B, 11=U
// Byte 0 bits[5:4] = first digit after category letter
// Byte 0 bits[3:0] = second digit
// Byte 1 bits[7:4] = third digit
// Byte 1 bits[3:0] = fourth digit
// ----------------------------------------------------------------
void formatDTC(uint8_t b0, uint8_t b1, char* out) {
  static const char cats[] = "PCBU";
  char cat   = cats[(b0 >> 6) & 0x03];
  uint8_t d1 = (b0 >> 4) & 0x03;
  uint8_t d2 = (b0     ) & 0x0F;
  uint8_t d3 = (b1 >> 4) & 0x0F;
  uint8_t d4 = (b1     ) & 0x0F;
  sprintf(out, "%c%X%X%X%X", cat, d1, d2, d3, d4);
}

// ----------------------------------------------------------------
// encodeDTC: invert formatDTC into the uint16_t key used in the table
// P0xxx = 0x0xxx, P1xxx = 0x1xxx ... P3xxx = 0x3xxx
// B0xxx = 0x4xxx, B1xxx = 0x5xxx
// C0xxx = 0x8xxx
// U0xxx = 0xC000 + xxx
// ----------------------------------------------------------------
uint16_t encodeDTCKey(const char* code) {
  // code is e.g. "P0301"
  uint16_t base = 0;
  switch (code[0]) {
    case 'P': base = 0x0000; break;
    case 'C': base = 0x8000; break;
    case 'B': base = 0x4000; break;
    case 'U': base = 0xC000; break;
    default:  return 0xFFFF;
  }
  // digits 1-4 are hex
  uint16_t num = 0;
  for (int i = 1; i <= 4; i++) {
    char c = code[i];
    uint8_t v = (c >= '0' && c <= '9') ? c - '0' :
                (c >= 'A' && c <= 'F') ? c - 'A' + 10 :
                (c >= 'a' && c <= 'f') ? c - 'a' + 10 : 0;
    num = (num << 4) | v;
  }
  return base | num;
}

// ----------------------------------------------------------------
// lookupDTC: binary search the PROGMEM table
// Returns description string or nullptr if not found
// ----------------------------------------------------------------
const char* lookupDTC(uint16_t key) {
  int lo = 0, hi = DTC_TABLE_SIZE - 1;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    uint16_t k = pgm_read_word(&dtc_table[mid].code);
    if (k == key) return dtc_table[mid].desc;
    if (k <  key) lo = mid + 1;
    else          hi = mid - 1;
  }
  return nullptr;
}

// ----------------------------------------------------------------
// requestDTCs: send Mode 03 and receive / assemble response
// Implements ISO 15765-2 single-frame and multi-frame with FC
// Fills dtc_list[] and sets dtc_count
// Returns true if at least one frame received, false on timeout
// ----------------------------------------------------------------
bool requestDTCs() {
  dtc_count  = 0;
  dtc_scanned = true;

  // Build Mode 03 request
  twai_message_t req;
  req.identifier       = OBD2_BROADCAST_ID;
  req.extd             = 0;
  req.data_length_code = 8;
  req.data[0] = 0x01;  // 1 additional byte
  req.data[1] = 0x03;  // Mode 03: request stored DTCs
  for (int i = 2; i < 8; i++) req.data[i] = 0x55;

  if (twai_transmit(&req, pdMS_TO_TICKS(200)) != ESP_OK) return false;

  // Receive first frame (SF or FF)
  static uint8_t payload[128];  // enough for ~20 codes
  int  payload_len = 0;
  int  total_len   = 0;
  bool got_ff      = false;

  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK) continue;
    if (rx.identifier != OBD2_REPLY_ID) continue;
    if (rx.data_length_code < 2)         continue;

    uint8_t pci = rx.data[0];

    if ((pci & 0xF0) == 0x00) {
      // Single Frame (SF)
      if (rx.data[1] != 0x43) break;  // positive response to Mode 03
      // data[2] = number of DTCs, then pairs follow
      int n = rx.data[2];
      int idx = 3;
      for (int d = 0; d < n && d < MAX_DTCS && idx + 1 < 8; d++, idx += 2) {
        if (rx.data[idx] == 0x00 && rx.data[idx+1] == 0x00) continue;
        formatDTC(rx.data[idx], rx.data[idx+1], dtc_list[dtc_count++]);
      }
      return true;

    } else if ((pci & 0xF0) == 0x10) {
      // First Frame (FF)
      total_len = ((pci & 0x0F) << 8) | rx.data[1];
      if (rx.data[2] != 0x43) break;
      // Copy data bytes 3..7 into payload (skip pci, len, mode)
      for (int i = 3; i < 8 && payload_len < (int)sizeof(payload); i++) {
        payload[payload_len++] = rx.data[i];
      }
      got_ff = true;

      // Send Flow Control (FC) — continue, block size 0, ST 0
      twai_message_t fc;
      fc.identifier       = 0x7E0;  // tester physical address
      fc.extd             = 0;
      fc.data_length_code = 8;
      fc.data[0] = 0x30; fc.data[1] = 0x00; fc.data[2] = 0x00;
      for (int i = 3; i < 8; i++) fc.data[i] = 0x00;
      twai_transmit(&fc, pdMS_TO_TICKS(100));
      break;

    } else if ((pci & 0xF0) == 0x20 && got_ff) {
      // Should not reach here — handled in CF loop below
      break;
    }
  }

  // Receive Consecutive Frames (CF) if we got a First Frame
  if (got_ff) {
    uint8_t expected_sn = 1;
    uint32_t cf_t0 = millis();
    while (payload_len < total_len - 2 && millis() - cf_t0 < 1000) {
      twai_message_t rx;
      if (twai_receive(&rx, pdMS_TO_TICKS(100)) != ESP_OK) continue;
      if (rx.identifier != OBD2_REPLY_ID) continue;
      if ((rx.data[0] & 0xF0) != 0x20)   continue;
      if ((rx.data[0] & 0x0F) != (expected_sn & 0x0F)) continue;
      expected_sn++;
      for (int i = 1; i < 8 && payload_len < (int)sizeof(payload); i++) {
        payload[payload_len++] = rx.data[i];
      }
    }

    // Parse payload — first byte is DTC count, then pairs
    if (payload_len >= 1) {
      int n = payload[0];
      for (int d = 0; d < n && d < MAX_DTCS && (d * 2 + 2) < payload_len; d++) {
        uint8_t b0 = payload[1 + d * 2];
        uint8_t b1 = payload[2 + d * 2];
        if (b0 == 0x00 && b1 == 0x00) continue;
        formatDTC(b0, b1, dtc_list[dtc_count++]);
      }
    }
    return true;
  }

  return (dtc_count > 0);
}

// ----------------------------------------------------------------
// clearDTCs: send Mode 04, wait for positive response 0x44
// Returns true on success
// ----------------------------------------------------------------
bool clearDTCs() {
  twai_message_t req;
  req.identifier       = OBD2_BROADCAST_ID;
  req.extd             = 0;
  req.data_length_code = 8;
  req.data[0] = 0x01;
  req.data[1] = 0x04;  // Mode 04: clear stored DTCs
  for (int i = 2; i < 8; i++) req.data[i] = 0x55;

  if (twai_transmit(&req, pdMS_TO_TICKS(200)) != ESP_OK) return false;

  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(100)) != ESP_OK) continue;
    if (rx.identifier != OBD2_REPLY_ID) continue;
    if (rx.data_length_code < 2)         continue;
    if (rx.data[1] == 0x44) return true;  // positive response
    if (rx.data[1] == 0x7F) return false; // negative response
  }
  return false;
}


// ================================================================
//  DTC SCREEN
//  Layout:
//    Header bar (60px) with title + DTC count
//    Scrollable code list — each card 90px tall
//      Left accent bar colour-coded by category
//      Code in large amber, description in smaller grey
//    Bottom button bar (80px):
//      [SCAN / RESCAN]   [CLEAR CODES]  [BACK]
// ================================================================
#define DTC_HEADER_H   60
#define DTC_BTNBAR_H   80
#define DTC_CARD_H     90
#define DTC_BTN_W     200
#define DTC_BTN_H      52
#define DTC_VISIBLE    ((M5.Display.height() - DTC_HEADER_H - DTC_BTNBAR_H) / DTC_CARD_H)

// Category accent colour from code string
uint16_t dtcCategoryColour(const char* code) {
  switch (code[0]) {
    case 'P': return COL_AMBER;
    case 'B': return COL_CYAN;
    case 'C': return COL_GREEN;
    case 'U': return COL_YELLOW;
    default:  return COL_LTGREY;
  }
}

void drawDTCScreen() {
  int W = M5.Display.width();
  int H = M5.Display.height();

  M5.Display.fillScreen(COL_BG);

  // ── Header ────────────────────────────────────────────────────
  M5.Display.fillRect(0, 0, W, DTC_HEADER_H, COL_PANEL);
  M5.Display.drawFastHLine(0, DTC_HEADER_H, W, COL_AMBER);

  M5.Display.setTextDatum(middle_left);
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(COL_WHITE, COL_PANEL);
  M5.Display.drawString("DIAGNOSTIC CODES", 20, DTC_HEADER_H / 2);

  // DTC count badge (top right)
  if (dtc_scanned) {
    char badge[24];
    if (dtc_count == 0)
      sprintf(badge, "NO CODES");
    else
      sprintf(badge, "%d CODE%s", dtc_count, dtc_count == 1 ? "" : "S");
    M5.Display.setTextDatum(middle_right);
    M5.Display.setTextSize(2);
    uint16_t badgeCol = (dtc_count == 0) ? COL_GREEN : COL_RED;
    M5.Display.setTextColor(badgeCol, COL_PANEL);
    M5.Display.drawString(badge, W - 20, DTC_HEADER_H / 2);
  } else {
    M5.Display.setTextDatum(middle_right);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(COL_DKGREY, COL_PANEL);
    M5.Display.drawString("TAP SCAN", W - 20, DTC_HEADER_H / 2);
  }

  // ── Code list area ────────────────────────────────────────────
  int listTop = DTC_HEADER_H;
  int listH   = H - DTC_HEADER_H - DTC_BTNBAR_H;
  M5.Display.fillRect(0, listTop, W, listH, COL_BG);

  if (!dtc_scanned) {
    // Not yet scanned — prompt
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(COL_DKGREY, COL_BG);
    M5.Display.drawString("Tap SCAN to request codes from ECU", W / 2, listTop + listH / 2);

  } else if (dtc_count == 0) {
    // No codes
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(3);
    M5.Display.setTextColor(COL_GREEN, COL_BG);
    M5.Display.drawString("NO STORED CODES", W / 2, listTop + listH / 2 - 20);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(COL_DKGREY, COL_BG);
    M5.Display.drawString("System is clear", W / 2, listTop + listH / 2 + 20);

  } else {
    // Draw visible cards
    int visible = DTC_VISIBLE;
    for (int i = 0; i < visible; i++) {
      int idx = dtc_scroll + i;
      if (idx >= dtc_count) break;

      int cy_card = listTop + i * DTC_CARD_H;
      uint16_t accentCol = dtcCategoryColour(dtc_list[idx]);

      // Card background
      uint16_t cardBg = (i % 2 == 0) ? 0x2124 : 0x1C1F2E;
      M5.Display.fillRect(0, cy_card, W, DTC_CARD_H, cardBg);
      M5.Display.drawFastHLine(0, cy_card + DTC_CARD_H - 1, W, COL_BORDER);

      // Left accent bar
      M5.Display.fillRect(0, cy_card, 6, DTC_CARD_H, accentCol);

      // Code (large)
      M5.Display.setTextDatum(middle_left);
      M5.Display.setTextSize(3);
      M5.Display.setTextColor(accentCol, cardBg);
      M5.Display.drawString(dtc_list[idx], 20, cy_card + 28);

      // Category label
      const char* catLabel = "";
      switch (dtc_list[idx][0]) {
        case 'P': catLabel = "POWERTRAIN"; break;
        case 'B': catLabel = "BODY";       break;
        case 'C': catLabel = "CHASSIS";    break;
        case 'U': catLabel = "NETWORK";    break;
      }
      M5.Display.setTextSize(1);
      M5.Display.setTextColor(accentCol, cardBg);
      M5.Display.drawString(catLabel, 120, cy_card + 22);

      // Description
      uint16_t key = encodeDTCKey(dtc_list[idx]);
      const char* desc = lookupDTC(key);
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(COL_LTGREY, cardBg);
      M5.Display.setTextDatum(middle_left);
      if (desc) {
        // Truncate to fit within panel width
        char truncDesc[60];
        strncpy(truncDesc, desc, 55);
        truncDesc[55] = '\0';
        if (strlen(desc) > 55) strcat(truncDesc, "...");
        M5.Display.drawString(truncDesc, 20, cy_card + 62);
      } else {
        M5.Display.setTextColor(COL_DKGREY, cardBg);
        M5.Display.drawString("No description available", 20, cy_card + 62);
      }

      // Scroll index indicator (right edge)
      if (dtc_count > visible) {
        M5.Display.setTextDatum(middle_right);
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(COL_DKGREY, cardBg);
        char idxStr[8]; sprintf(idxStr, "%d/%d", idx + 1, dtc_count);
        M5.Display.drawString(idxStr, W - 8, cy_card + DTC_CARD_H / 2);
      }
    }

    // Scroll arrows if needed
    if (dtc_scroll > 0) {
      M5.Display.setTextDatum(top_center);
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(COL_AMBER, COL_BG);
      M5.Display.drawString("^", W - 24, listTop + 2);
    }
    if (dtc_scroll + visible < dtc_count) {
      M5.Display.setTextDatum(bottom_center);
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(COL_AMBER, COL_BG);
      M5.Display.drawString("v", W - 24, listTop + listH - 2);
    }
  }

  // ── Button bar ────────────────────────────────────────────────
  int barY = H - DTC_BTNBAR_H;
  M5.Display.fillRect(0, barY, W, DTC_BTNBAR_H, COL_PANEL);
  M5.Display.drawFastHLine(0, barY, W, COL_BORDER);

  int btnY   = barY + (DTC_BTNBAR_H - DTC_BTN_H) / 2;
  int gap    = (W - 3 * DTC_BTN_W) / 4;
  int scanX  = gap;
  int clearX = gap * 2 + DTC_BTN_W;
  int backX  = gap * 3 + DTC_BTN_W * 2;

  // SCAN button
  M5.Display.fillRoundRect(scanX, btnY, DTC_BTN_W, DTC_BTN_H, 8, COL_AMBER);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_BG, COL_AMBER);
  M5.Display.drawString(dtc_scanned ? "RESCAN" : "SCAN", scanX + DTC_BTN_W / 2, btnY + DTC_BTN_H / 2);

  // CLEAR button
  if (dtc_clear_confirm) {
    // Confirmation state — red with warning text
    M5.Display.fillRoundRect(clearX, btnY, DTC_BTN_W, DTC_BTN_H, 8, COL_RED);
    M5.Display.setTextColor(COL_WHITE, COL_RED);
    M5.Display.setTextSize(1);
    M5.Display.drawString("TAP AGAIN", clearX + DTC_BTN_W / 2, btnY + 16);
    M5.Display.drawString("TO CONFIRM", clearX + DTC_BTN_W / 2, btnY + 32);
    M5.Display.drawString("CLEAR ALL", clearX + DTC_BTN_W / 2, btnY + 44);
  } else {
    uint16_t clearBg = (dtc_count > 0) ? 0x6000 : COL_BORDER;
    M5.Display.fillRoundRect(clearX, btnY, DTC_BTN_W, DTC_BTN_H, 8, clearBg);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor((dtc_count > 0) ? COL_RED : COL_DKGREY, clearBg);
    M5.Display.drawString("CLEAR CODES", clearX + DTC_BTN_W / 2, btnY + DTC_BTN_H / 2);
  }

  // BACK button
  M5.Display.fillRoundRect(backX, btnY, DTC_BTN_W, DTC_BTN_H, 8, COL_PANEL);
  M5.Display.drawRoundRect(backX, btnY, DTC_BTN_W, DTC_BTN_H, 8, COL_BORDER);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_LTGREY, COL_PANEL);
  M5.Display.drawString("BACK", backX + DTC_BTN_W / 2, btnY + DTC_BTN_H / 2);
}

void handleDTCTouch() {
  int W = M5.Display.width();
  int H = M5.Display.height();
  auto t = M5.Touch.getDetail();
  int tx = t.x, ty = t.y;

  int barY   = H - DTC_BTNBAR_H;
  int btnY   = barY + (DTC_BTNBAR_H - DTC_BTN_H) / 2;
  int gap    = (W - 3 * DTC_BTN_W) / 4;
  int scanX  = gap;
  int clearX = gap * 2 + DTC_BTN_W;
  int backX  = gap * 3 + DTC_BTN_W * 2;

  // ── BACK ──────────────────────────────────────────────────────
  if (tx >= backX && tx <= backX + DTC_BTN_W && ty >= btnY && ty <= btnY + DTC_BTN_H) {
    in_dtc_screen    = false;
    dtc_clear_confirm = false;
    drawBaseUI();
    return;
  }

  // ── SCAN ──────────────────────────────────────────────────────
  if (tx >= scanX && tx <= scanX + DTC_BTN_W && ty >= btnY && ty <= btnY + DTC_BTN_H) {
    dtc_clear_confirm = false;
    // Show scanning state
    M5.Display.fillRect(0, DTC_HEADER_H, W, H - DTC_HEADER_H - DTC_BTNBAR_H, COL_BG);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(COL_AMBER, COL_BG);
    M5.Display.drawString("SCANNING ECU...", W / 2, (H - DTC_HEADER_H - DTC_BTNBAR_H) / 2 + DTC_HEADER_H);
    delay(100);
    requestDTCs();
    dtc_scroll = 0;
    drawDTCScreen();
    return;
  }

  // ── CLEAR ─────────────────────────────────────────────────────
  if (tx >= clearX && tx <= clearX + DTC_BTN_W && ty >= btnY && ty <= btnY + DTC_BTN_H) {
    if (dtc_count == 0) return; // nothing to clear

    if (!dtc_clear_confirm) {
      // First tap — arm confirmation
      dtc_clear_confirm = true;
      drawDTCScreen();
      return;
    }

    // Second tap — execute clear
    dtc_clear_confirm = false;

    // Show clearing state
    M5.Display.fillRect(0, DTC_HEADER_H, W, H - DTC_HEADER_H - DTC_BTNBAR_H, COL_BG);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(COL_RED, COL_BG);
    M5.Display.drawString("CLEARING CODES...", W / 2, (H - DTC_HEADER_H - DTC_BTNBAR_H) / 2 + DTC_HEADER_H);
    delay(100);

    bool cleared = clearDTCs();

    if (cleared) {
      dtc_count  = 0;
      dtc_scroll = 0;
      // Auto-rescan to confirm clear
      requestDTCs();
    } else {
      // Show failure briefly
      M5.Display.fillRect(0, DTC_HEADER_H, W, H - DTC_HEADER_H - DTC_BTNBAR_H, COL_BG);
      M5.Display.setTextColor(COL_RED, COL_BG);
      M5.Display.drawString("CLEAR FAILED — ECU refused", W / 2, (H - DTC_HEADER_H - DTC_BTNBAR_H) / 2 + DTC_HEADER_H);
      delay(1500);
    }
    drawDTCScreen();
    return;
  }

  // ── SCROLL (tap upper/lower half of list area) ────────────────
  int listTop = DTC_HEADER_H;
  int listH   = H - DTC_HEADER_H - DTC_BTNBAR_H;
  int visible = DTC_VISIBLE;

  if (ty >= listTop && ty < listTop + listH && dtc_count > visible) {
    if (ty < listTop + listH / 2) {
      // Upper half — scroll up
      if (dtc_scroll > 0) { dtc_scroll--; drawDTCScreen(); }
    } else {
      // Lower half — scroll down
      if (dtc_scroll + visible < dtc_count) { dtc_scroll++; drawDTCScreen(); }
    }
    return;
  }

  // Any other touch in list area dismisses confirm state
  if (dtc_clear_confirm) {
    dtc_clear_confirm = false;
    drawDTCScreen();
  }
}


// ================================================================
//  SETTINGS CONSTANTS  (used by drawSettingsIMU and drawSettingsScreen)
// ================================================================
#define SETT_ROWS    6
#define SETT_ROW_H   80
#define SETT_LIST_Y  80
#define SETT_BTN_W  160
#define SETT_BTN_H   50

// ================================================================
//  SETTINGS — LIVE IMU PANEL
//  Drawn below the row list, above the BACK button.
//  Shows X / Y / Z acceleration values live so the user can see
//  which axis responds to forward movement while settings is open.
//  Also shows a small bar graph for the selected axis vs threshold.
// ================================================================
void drawSettingsIMU() {
  int W = M5.Display.width(), H = M5.Display.height();

  // Panel occupies the gap between row list bottom and back button
  int panelY  = SETT_LIST_Y + SETT_ROWS * SETT_ROW_H + 4;
  int btnY    = H - SETT_BTN_H - 20;
  int panelH  = btnY - panelY - 8;
  if (panelH < 20) return; // not enough room

  // Background
  M5.Display.fillRect(0, panelY, W, panelH, COL_PANEL);

  // Read IMU
  float ax = 0, ay = 0, az = 0;
  bool ok = M5.Imu.getAccel(&ax, &ay, &az);
  float vals[3] = {ax, ay, az};
  const char* labels[3] = {"X", "Y", "Z"};

  // Three axis columns
  int colW = W / 3;
  int midY  = panelY + panelH / 2;

  for (int i = 0; i < 3; i++) {
    int colX  = i * colW;
    bool sel  = (i == (int)setting_imu_axis);
    uint16_t bg = sel ? 0x2945 : COL_PANEL;  // slightly lighter if selected

    M5.Display.fillRect(colX + 2, panelY + 2, colW - 4, panelH - 4, bg);
    if (sel) M5.Display.drawRect(colX + 2, panelY + 2, colW - 4, panelH - 4, COL_AMBER);

    // Axis label
    M5.Display.setTextDatum(top_center);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(sel ? COL_AMBER : COL_DKGREY, bg);
    M5.Display.drawString(labels[i], colX + colW / 2, panelY + 5);

    // Value
    if (ok) {
      char buf[12]; dtostrf(vals[i], 6, 3, buf);
      // Colour by magnitude — green=low, amber=mid, red=above threshold
      float mag = fabs(vals[i]);
      uint16_t vc = (mag > setting_launch_accel) ? COL_GREEN :
                    (mag > setting_launch_accel * 0.5f) ? COL_AMBER : COL_LTGREY;
      M5.Display.setTextDatum(middle_center);
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(vc, bg);
      M5.Display.drawString(buf, colX + colW / 2, midY + 4);

      // Small bar showing magnitude vs threshold (max bar = 1g)
      int barMaxW = colW - 24;
      int barH    = 6;
      int barX    = colX + 12;
      int barY    = panelY + panelH - 14;
      float ratio = mag / 1.0f;  // normalise to 1g max
      if (ratio > 1.0f) ratio = 1.0f;
      int fillW = (int)(ratio * barMaxW);
      M5.Display.fillRect(barX, barY, barMaxW, barH, COL_BORDER);
      if (fillW > 0) M5.Display.fillRect(barX, barY, fillW, barH, vc);
      // Threshold tick
      int tickX = barX + (int)(setting_launch_accel * barMaxW);
      if (tickX < barX + barMaxW)
        M5.Display.drawFastVLine(tickX, barY - 2, barH + 4, COL_AMBER);
    } else {
      M5.Display.setTextDatum(middle_center);
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(COL_RED, bg);
      M5.Display.drawString("ERR", colX + colW / 2, midY + 4);
    }
  }

  // "LAUNCH AXIS" hint text centred
  M5.Display.setTextDatum(top_center);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(COL_DKGREY, COL_PANEL);
  M5.Display.drawString("MOVE DEVICE  —  GREEN = ABOVE THRESHOLD  —  AMBER TICK = THRESHOLD", W / 2, panelY + panelH - 28);
}


// ================================================================
//  SETTINGS SCREEN
// ================================================================

void drawSettingsScreen() {
  int W = M5.Display.width(), H = M5.Display.height();
  M5.Display.fillScreen(COL_BG);

  // Header
  M5.Display.fillRect(0, 0, W, 60, COL_PANEL);
  M5.Display.drawFastHLine(0, 60, W, COL_AMBER);
  M5.Display.setTextDatum(middle_left);
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(COL_WHITE, COL_PANEL);
  M5.Display.drawString("SETTINGS", 20, 30);
  M5.Display.setTextDatum(middle_right);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_DKGREY, COL_PANEL);
  M5.Display.drawString("TAP ROW TO CHANGE  |  TAP BACK TO EXIT", W - 20, 30);

  // Dividers & row content
  const char* axisNames[] = {"X", "Y", "Z"};
  char valBuf[32];

  struct { const char* label; const char* value; } rows[SETT_ROWS];
  sprintf(valBuf, "%s-Axis", axisNames[setting_imu_axis]);
  rows[0] = {"LAUNCH AXIS (IMU)",   valBuf};

  static char v1[16]; sprintf(v1, "%.2f g", setting_launch_accel);
  rows[1] = {"LAUNCH THRESHOLD",    v1};

  static char v2[16]; sprintf(v2, "%d RPM", setting_rpm_redline);
  rows[2] = {"RPM REDLINE",         v2};

  static char v3[16]; sprintf(v3, "%.0f F", setting_iat_warn);
  rows[3] = {"IAT WARN TEMP",       v3};

  static char v4[16]; sprintf(v4, "%.0f F", setting_iat_crit);
  rows[4] = {"IAT CRIT TEMP",       v4};

  rows[5] = {"SCAN / CLEAR CODES",  ">"};   // navigation row — tap to open DTC screen

  for (int i = 0; i < SETT_ROWS; i++) {
    int ry = SETT_LIST_Y + i * SETT_ROW_H;
    M5.Display.drawFastHLine(0, ry, W, COL_BORDER);

    // Label
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextSize(2);
    // Highlight the navigation row
    uint16_t labelCol = (i == 5) ? COL_CYAN : COL_LTGREY;
    M5.Display.setTextColor(labelCol, COL_BG);
    M5.Display.drawString(rows[i].label, 30, ry + SETT_ROW_H / 2 - 10);

    // Value / chevron
    M5.Display.setTextDatum(middle_right);
    if (i == 5) {
      // Navigation row: draw a cyan arrow button instead of a cycling value
      M5.Display.fillRoundRect(W - 100, ry + SETT_ROW_H / 2 - 18, 80, 36, 6, COL_CYAN);
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(COL_BG, COL_CYAN);
      M5.Display.drawString("OPEN", W - 60, ry + SETT_ROW_H / 2 + 4);
    } else {
      M5.Display.setTextSize(3);
      M5.Display.setTextColor(COL_AMBER, COL_BG);
      M5.Display.drawString(rows[i].value, W - 30, ry + SETT_ROW_H / 2 + 4);
      // Tap chevron hint
      M5.Display.setTextSize(2);
      M5.Display.setTextColor(COL_BORDER, COL_BG);
      M5.Display.drawString(">", W - 12, ry + SETT_ROW_H / 2 + 4);
    }
  }

  M5.Display.drawFastHLine(0, SETT_LIST_Y + SETT_ROWS * SETT_ROW_H, W, COL_BORDER);

  // Live IMU readout panel — shows all 3 axes so user can identify correct launch axis
  drawSettingsIMU();

  // Back button
  int bx = W / 2 - SETT_BTN_W / 2;
  int by = H - SETT_BTN_H - 20;
  M5.Display.fillRoundRect(bx, by, SETT_BTN_W, SETT_BTN_H, 8, COL_AMBER);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_BG, COL_AMBER);
  M5.Display.drawString("BACK", W / 2, by + SETT_BTN_H / 2);
}

void handleSettingsTouch() {
  int W = M5.Display.width(), H = M5.Display.height();
  auto t = M5.Touch.getDetail();
  int tx = t.x, ty = t.y;

  // Back button
  int bx = W / 2 - SETT_BTN_W / 2;
  int by = H - SETT_BTN_H - 20;
  if (tx >= bx && tx <= bx + SETT_BTN_W && ty >= by && ty <= by + SETT_BTN_H) {
    in_settings = false;
    drawBaseUI();
    return;
  }

  // Row taps
  for (int i = 0; i < SETT_ROWS; i++) {
    int ry = SETT_LIST_Y + i * SETT_ROW_H;
    if (ty >= ry && ty < ry + SETT_ROW_H) {
      switch (i) {
        case 0: // IMU axis: cycle X→Y→Z→X
          setting_imu_axis = (setting_imu_axis + 1) % 3;
          break;
        case 1: // Launch threshold: 0.05 / 0.10 / 0.15 / 0.20 / 0.25 / 0.30 / 0.35
          setting_launch_accel += 0.05f;
          if (setting_launch_accel > 0.351f) setting_launch_accel = 0.05f;
          break;
        case 2: // RPM redline: 4000-9000 in 500 steps
          setting_rpm_redline += 500;
          if (setting_rpm_redline > 9000) setting_rpm_redline = 4000;
          break;
        case 3: // IAT warn: 100-160 in 10 steps
          setting_iat_warn += 10.0f;
          if (setting_iat_warn > 160.0f) setting_iat_warn = 100.0f;
          if (setting_iat_warn >= setting_iat_crit) setting_iat_crit = setting_iat_warn + 10.0f;
          break;
        case 4: // IAT crit: warn+10 to 200 in 10 steps
          setting_iat_crit += 10.0f;
          if (setting_iat_crit > 200.0f) setting_iat_crit = setting_iat_warn + 10.0f;
          break;
        case 5: // Navigate to DTC screen
          in_settings   = false;
          in_dtc_screen = true;
          dtc_scroll     = 0;
          dtc_clear_confirm = false;
          drawDTCScreen();
          return;
      }
      drawSettingsScreen(); // refresh
      return;
    }
  }
}


// ================================================================
//  GPS 10 Hz CONFIGURATION
// ================================================================
void configureGPS10Hz() {
  delay(500);
  while (GPS_Serial.available()) GPS_Serial.read();

  GPS_Serial.print("$PMTK220,100*2F\r\n");
  delay(200);
  GPS_Serial.print("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
  delay(200);
  GPS_Serial.print("$PCAS02,100*1E\r\n");
  delay(200);

  String response = "";
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    if (GPS_Serial.available()) response += (char)GPS_Serial.read();
  }

  if (response.length() > 0) {
    GPS_Serial.print("$PMTK251,115200*1F\r\n");
    delay(500);
    GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(200);
    GPS_Serial.print("$PMTK220,100*2F\r\n");
    gps_configured = true;
  } else {
    gps_configured = false;
  }
}


// ================================================================
//  CAN / TWAI
// ================================================================
void setupCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) twai_start();
}

void checkCANHealth() {
  uint32_t now = millis();
  if (now - last_can_health_check < CAN_HEALTH_CHECK_MS) return;
  if (can_rx_count > last_can_success_count) {
    last_can_success_count = can_rx_count;
    last_can_success_time  = now;
    can_recovery_attempts  = 0;
  }
  if ((now - last_can_success_time > 10000) &&
      can_recovery_attempts < CAN_MAX_RECOVERY_TRIES)
    performCANRecovery(now);
  last_can_health_check = now;
}

void performCANRecovery(uint32_t now) {
  can_recovery_attempts++;
  // Only draw recovery message if we're on the main screen
  if (!in_settings && !in_dtc_screen) {
    int W = M5.Display.width();
    M5.Display.fillRect(0, 0, W, STATUS_H, COL_PANEL);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(COL_RED, COL_PANEL);
    char buf[48];
    sprintf(buf, "CAN RECOVERY  %d / %d", can_recovery_attempts, CAN_MAX_RECOVERY_TRIES);
    M5.Display.drawString(buf, W / 2, STATUS_H / 2);
  }
  twai_stop();
  twai_driver_uninstall();
  delay(200);
  setupCAN();
  last_can_success_time  = now;
  last_can_success_count = can_rx_count;
  can_err_count++;
  delay(500);
}


// ================================================================
//  OBD2
// ================================================================
float getOBD2Data(uint8_t pid) {
  twai_message_t msg;
  msg.identifier       = OBD2_BROADCAST_ID;
  msg.extd             = 0;
  msg.data_length_code = 8;
  msg.data[0] = 0x02; msg.data[1] = 0x01; msg.data[2] = pid;
  for (int i = 3; i < 8; i++) msg.data[i] = 0x55;

  // Use 0-tick timeout on transmit — if the TX queue is full (e.g. bus-off state)
  // this returns immediately with an error rather than blocking up to 100 ms.
  if (twai_transmit(&msg, pdMS_TO_TICKS(0)) != ESP_OK) {
    if (++can_err_count > 50) obd2_connected = false;
    return -1.0f;
  }

  uint32_t t0 = millis();
  while (millis() - t0 < CAN_TIMEOUT_MS) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(CAN_RECEIVE_WAIT_MS)) != ESP_OK) continue;
    if (rx.identifier != OBD2_REPLY_ID) continue;
    if (rx.data_length_code < 4)        continue;
    if (rx.data[1] != 0x41)             continue;
    if (rx.data[2] != pid)              continue;

    if (pid == PID_RPM) {
      if (rx.data[0] != OBD2_TWO_BYTE_LEN || rx.data_length_code < 5) continue;
      can_rx_count++; obd2_connected = true; can_err_count = 0;
      return ((rx.data[3] * 256.0f) + rx.data[4]) / 4.0f;
    } else {
      if (rx.data[0] != OBD2_SINGLE_BYTE_LEN) continue;
      can_rx_count++; obd2_connected = true; can_err_count = 0;
      return (float)rx.data[3];
    }
  }

  if (++can_err_count > 50) obd2_connected = false;
  return -1.0f;
}

// PID_MAP is used for connection test — reliable single-byte PID.
// PID_SUPPORTED (0x00) returns 4 data bytes; its length byte (0x06)
// would always fail the OBD2_SINGLE_BYTE_LEN check.
void setupOBD2() {
  obd2_connected = (getOBD2Data(PID_MAP) >= 0.0f);
}


// ================================================================
//  BATTERY
// ================================================================
float getBatteryVoltage() {
  return M5.Power.getBatteryVoltage() / 1000.0f;
}
