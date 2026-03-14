# OBD2 Performance Logger
### M5Stack Tab5 · ESP32-P4 · CAN Bus · GPS 10 Hz · Arduino

A full-featured drag strip performance logger and boost monitor built for the M5Stack Tab5. Real-time OBD2 telemetry over CAN bus, 10 Hz GPS timing, a professional light tree with reaction time measurement, RPM and boost gauges with peak trackers, SD card CSV logging, and a built-in diagnostic code scanner — all on a 1280×720 touchscreen display.

---

## Screenshots / Display Layout

```
┌─────────────────────────────────────────────────────────────────────────┐
│  GPS  8 SAT          OBD2  LIVE                    SD  12.4V            │  ← Status bar
├──────────┬──────────────────────────────────────────┬───────────────────┤
│          │                                          │       RPM         │
│ ELAPSED  │         ┌─── RPM arc ───┐               │      4250         │
│  3.42s   │        /  ┌─ Boost ─┐   \              ├───────────────────┤
│ RT 0.38s │       /   │  ticks  │    \              │       MPH         │
│          │      │    │  -15→20 │     │             │      47.3         │
│ 0-60 mph │      │    │   PSI   │     │             ├───────────────────┤
│  4.21s   │      │    │  14.2   │     │             │      IAT  F       │
│          │       \   │         │    /              │       118         │
│ 1/4 MILE │        \  └─────────┘   /               │                   │
│  11.83s  │         └───────────────┘               │                   │
│ 98.4 mph │                                          │                   │
├──────────┴──────────────────────────────────────────┴───────────────────┤
│              LAUNCHED  |  LOGGING RUN 3  |  TAP TO RESET                │  ← Bottom banner
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Features

### Launch System & Light Tree
- Full NHRA-style 3-amber → green countdown sequence
- 500 ms per amber stage
- Auto-stages when GPS fix acquired, 4+ satellites, speed < 3 mph
- Tap screen to start sequence once staged
- **Reaction time** measured from green light to first forward acceleration (IMU)
- Tap at green also works as a manual launch trigger
- Un-stages automatically if conditions drop before sequence starts

### Performance Timing
| Metric | Method |
|---|---|
| 0–60 mph | GPS speed crossing, 10 Hz resolution |
| Quarter mile | GPS path distance accumulation |
| Trap speed | GPS speed captured at exact 402.336 m crossing |
| Reaction time | IMU accel threshold from green light |
| Elapsed timer | Live countdown from launch, freezes at finish |

### Boost Gauge
- Range: −15 to +20 PSI
- 260° sweep arc, colour-coded ticks (grey / amber / red)
- Large centre numeric readout
- Peak boost marker with `PK xx.x` label during run
- Needle erase-and-redraw — no flicker

### RPM Gauge (outer arc)
- Concentric ring outside the boost gauge, same 260° sweep
- 0–8000 RPM, ticks every 500 RPM, labelled every 1000
- Colour zones: grey → amber (1000 RPM before redline) → red (at/above redline)
- Cyan live needle in the outer band
- **Peak RPM marker** — cyan tick that sticks at the highest RPM seen, with value label. Persists until reset.

### OBD2 Telemetry (CAN Bus)
- MAP → Boost PSI (kPa − baro reference × 0.145038)
- RPM (two-byte PID)
- IAT → °F with warn/crit colour thresholds
- Barometric pressure sampled at boot for accurate boost zero
- ISO 15765-2 multi-frame handling for DTC scanner
- Automatic CAN bus recovery (up to 5 restarts)

### GPS
- 10 Hz negotiation at boot (MTK PMTK220 / CASIC PCAS02)
- Upgrades to 115200 baud after successful handshake
- 15 m glitch filter on distance accumulation
- Speed zeroed immediately on fix loss

### Diagnostic Code Scanner
- **Mode 03** — scan stored DTCs with full multi-frame flow control
- **Mode 04** — clear codes with two-tap confirmation
- 584-entry lookup table covering P0xxx, P1xxx, P2xxx, P3xxx, B0xxx, B1xxx, C0xxx, U0xxx
- Colour-coded by category: Amber = Powertrain, Cyan = Body, Green = Chassis, Yellow = Network
- Scrollable card list with full description text
- Auto-rescan after clear to confirm codes are gone

### SD Card Logging
- SDIO 4-bit, auto-numbered run files: `run_1.csv`, `run_2.csv` …
- CSV columns: `Time_s, GPS_Speed_mph, RPM, Boost_psi, Peak_Boost_psi, IAT_F, Distance_ft, Lat, Lng`
- Buffered writes (10 records / ~1 second flush rate)
- File opened on launch, flushed and closed at quarter-mile finish

### Settings Screen (long press 1.5s)
| Setting | Range | Default |
|---|---|---|
| Launch axis (IMU) | X / Y / Z | X |
| Launch threshold | 0.05 – 0.35 g (0.05 steps) | 0.10 g |
| RPM redline | 4000 – 9000 RPM (500 steps) | 6000 |
| IAT warn | 100 – 160 °F (10 steps) | 120 °F |
| IAT crit | warn+10 – 200 °F (10 steps) | 150 °F |
| Scan / Clear Codes | — | navigates to DTC screen |

**Live IMU readout** is shown at the bottom of the settings screen — three columns (X/Y/Z) updating in real time with bar graphs and a threshold tick. Move the device forward to identify the correct launch axis.

### Other
- Screen dims to brightness 60 after 5 minutes idle; any touch wakes it
- Battery voltage with warn/crit thresholds (7.0V / 6.5V)

---

## Hardware

### Platform
| Item | Detail |
|---|---|
| Board | M5Stack Tab5 |
| MCU | ESP32-P4 |
| Display | 1280 × 720, landscape |
| IMU | BMI270 (via M5Unified) |
| Battery | NP-F550 7.4V nominal |

### Wiring

#### CAN Bus (Expansion Header)
| Signal | GPIO |
|---|---|
| CAN TX | GPIO 49 |
| CAN RX | GPIO 50 |

> Requires an external CAN transceiver (e.g. TJA1051). Connect to OBD2 port CAN-H / CAN-L pins. Set `setNoAck(true)` is not needed — the ECU acts as the second node.

#### GPS (HY2.0-4P Grove Port)
| Signal | GPIO | Wire colour |
|---|---|---|
| GPS RX (into Tab5) | GPIO 53 | Yellow |
| GPS TX (into Tab5) | GPIO 54 | White |

#### microSD (SDIO 4-bit, internal)
| Signal | GPIO |
|---|---|
| CLK | GPIO 43 |
| CMD | GPIO 44 |
| DAT0 | GPIO 39 |
| DAT1 | GPIO 40 |
| DAT2 | GPIO 41 |
| DAT3 | GPIO 42 |

---

## Software Dependencies

Install via Arduino Library Manager or boards manager:

| Library | Purpose |
|---|---|
| `M5Unified` | Display, IMU, power |
| `TinyGPS++` | NMEA parsing |
| `SD_MMC` | SDIO SD card (ESP32 built-in) |
| `driver/twai.h` | CAN bus (ESP32 built-in) |
| `freertos/FreeRTOS.h` | RTOS ticks for timeouts |

**Board:** ESP32-P4 (M5Stack Tab5) — select in Arduino IDE boards manager.

---

## Build & Flash

1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. Add ESP32 board support: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Install libraries listed above
4. Open `OBD2_Logger.ino`
5. Select board: **ESP32P4 Dev Module** (or M5Stack Tab5 if available)
6. Set partition scheme to one with enough flash for the DTC table
7. Upload at 921600 baud

---

## Usage

### First Boot
The splash screen runs a checklist:
- **IMU** — BMI270 init
- **microSD** — SDIO mount, shows card size
- **CAN Bus** — TWAI driver start
- **OBD2 ECU** — MAP PID connection test
- **Baro** — barometric reference sample (used as boost zero)

If OBD2 shows FAIL, the car may not be running or the CAN transceiver wiring needs checking. The logger will still function for GPS timing without OBD2.

### Finding the Right Launch Axis
1. Long-press the screen to open **Settings**
2. The bottom panel shows live X / Y / Z accelerometer readings
3. Push the Tab5 firmly in the direction the car accelerates
4. Whichever axis shows green (above threshold) is correct
5. Tap the **LAUNCH AXIS** row to cycle X → Y → Z
6. Tap **BACK** to return

### Running a Pass
1. Drive to the line — the tree arms automatically when GPS fix is good and speed < 3 mph
2. Banner shows **STAGED**
3. Tap the screen to start the light tree countdown
4. Launch on green (IMU detects motion) or tap the screen manually
5. Elapsed, 0–60, and quarter-mile times update live
6. Finish banner shows quarter-mile time and trap speed
7. Tap to reset for the next run

### Reading Codes
1. Long-press → **Settings** → **SCAN / CLEAR CODES**
2. Tap **SCAN** — sends Mode 03, assembles multi-frame response if needed
3. Codes shown as cards with full description text
4. Tap **CLEAR CODES**, then tap again to confirm — sends Mode 04, auto-rescans

---

## Configuration Constants

Key constants at the top of the sketch if you need to adjust them:

```cpp
#define CAN_TIMEOUT_MS         200    // per-PID OBD2 receive timeout
#define CAN_HEALTH_CHECK_MS    5000   // interval between CAN health checks
#define CAN_MAX_RECOVERY_TRIES 5      // auto-restart attempts before giving up
#define LAUNCH_SPEED_THRESHOLD 3.0f   // mph — max speed to arm tree
#define BATTERY_CHECK_MS       120000 // battery poll interval (2 min)
#define QUARTER_MILE_M         402.336f
#define GPS_MAX_STEP_M         15.0f  // glitch filter on GPS steps
#define SCREEN_DIM_MS          300000 // idle timeout before dim (5 min)
#define TREE_STAGE_MS          500    // ms per amber stage
#define LONGPRESS_MS           1500   // long-press duration for settings
```

---

## CSV Log Format

Each run produces `/run_N.csv` on the SD card:

```
Time_s,GPS_Speed_mph,RPM,Boost_psi,Peak_Boost_psi,IAT_F,Distance_ft,Lat,Lng
0.00,0.0,850,0.00,0.00,112,0.0,35.123456,-97.654321
1.00,22.4,4820,8.34,8.34,114,107.2,35.123489,-97.654298
...
```

Import into Excel, Google Sheets, or any data tool. Latitude/Longitude columns allow plotting the run on a map.

---

## Colour Reference (RGB565)

| Constant | Hex | Use |
|---|---|---|
| `COL_BG` | `#181C27` | Background |
| `COL_PANEL` | `#212631` | Side panels |
| `COL_AMBER` | `#FF6800` | Primary live values |
| `COL_CYAN` | `#00FFFF` | Captured milestones, RPM needle |
| `COL_GREEN` | `#00FF00` | Launched state, good status |
| `COL_RED` | `#FF0000` | Over-boost, error, redline |
| `COL_YELLOW` | `#FFFF00` | Warnings, staged state |

---

## Known Limitations

- Settings are not persisted to flash — reset to defaults on power cycle
- DTC descriptions cover SAE J2012 generic codes; manufacturer-specific P1xxx codes may show "No description available"
- GPS timing accuracy is limited by 10 Hz update rate (~±0.1s at 100 mph trap)
- CAN bus must be at 500 kbps (standard OBD2 rate)

---

## License

MIT — do whatever you want with it, attribution appreciated.

---

## Contributing

Pull requests welcome. Key areas that could use work:
- NVS/Preferences persistence for settings across reboots
- Red-light detection (motion before green)
- Freeze-frame data display alongside DTCs
- Additional OBD2 PIDs (coolant temp, throttle position, fuel trims)
- Bluetooth data streaming

---

*Built for the track, not the showroom.*
