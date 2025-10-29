# STM32F411RE Moisture Monitoring System

## Project Overview
This is an embedded moisture monitoring and control system implemented on the STM32F411RE microcontroller with a training shield. The system monitors environmental conditions (moisture, temperature, light) and provides visual feedback through LEDs, a 7-segment display, and an optional OLED screen.

Calibration data is cached in RAM for the current session; SD-card backed persistence has been removed from the build to simplify deployment.

## Hardware Components

### Microcontroller
- **Board**: STM32F411RE (ARM Cortex-M4)
- **Clock**: 16 MHz HSI (default), configurable PLL

### I/O Peripherals
- **4× LEDs**: PA5, PA6, PA7, PB6 (moisture level bar graph)
- **7-Segment Display**: PC7 (A), PA8 (B), PB10 (C), PA9 (D) - BCD encoded
- **4× Buttons**: PA10 (BTN1), PB3 (BTN2), PB5 (BTN3), PB4 (BTN4) - Active LOW with pull-ups
- **OLED Display** (optional): 128×64 SSD1306 via I2C (PB8=SCL, PB9=SDA)
- **UART2**: PA2=TX, PA3=RX (115200 baud, debug console)
- **USART6**: PA11=TX, PA12=RX (115200 baud, telemetry bridge to ESP32)

### Analog Inputs (ADC1)
- **PA0** (CH0): Temperature sensor (normalized 0.0–1.0)
- **PA1** (CH1): Light-dependent resistor (LDR)
- **PB0** (CH8): Soil moisture probe (normalized 0.0–1.0)
- **PA4** (CH4): Potentiometer (calibration/threshold knob)

---

## System States

The system operates as a finite state machine with the following major states:

### 1. **BOOT States** (7-Seg: 0)
- **BOOT_SELFTEST**: Verifies ADC clock and peripherals
- **BOOT_LOADCFG**: Loads default configuration (T_low=0.35, T_high=0.55)
- **BOOT_CALCHECK**: Applies default thresholds and validates calibration parameters

### 2. **IDLE State** (7-Seg: 0)
- **IDLE_STANDBY**: System standby, waiting for periodic monitoring cycle
- **LEDs**: All OFF
- **Action**: Press BTN1 to trigger immediate monitoring, or waits 1 second for auto-cycle

### 3. **MONITORING States** (7-Seg: 1)
- **MON_SENSE_FILTER**: Reads and filters ADC values (EMA filter)
- **MON_MOIST_EVAL**: Evaluates moisture level and updates LED bar graph
- **MON_ENV_COMP**: Environmental compensation using temperature and light
- **MON_NORMAL**: Moisture above high threshold (safe)
- **MON_BORDERLINE**: Moisture between thresholds (warning)
- **MON_PREALERT**: Moisture below low threshold for 5+ consecutive cycles

**LED Bar Graph** (indicates moisture level):
- 0 LEDs: < 30% moisture
- 1 LED:  30–50% moisture
- 2 LEDs: 50–70% moisture
- 3 LEDs: 70–90% moisture
- 4 LEDs: ≥ 90% moisture

### 4. **ALERT States** (7-Seg: 2)
- **ALERT_VISUAL**: Flashing LEDs (4 LEDs blink at 250ms intervals for 3 seconds)
- **ALERT_LATCH**: Alert latched, requires user acknowledgment
- **ALERT_SNOOZE**: Alert snoozed for 60 seconds

### 5. **CALIBRATION States** (7-Seg: 3)
- **CAL_MENU**: Calibration menu (select option)
- **CAL_SET_DRY**: Set dry reference level
- **CAL_SET_WET**: Set wet reference level
- **CAL_SET_THRESH**: Set threshold parameters

### 6. **FAULT State** (7-Seg: 4)
- Critical error detected (e.g., ADC disabled, invalid thresholds)
- Recovery: Hold BTN1 + BTN2 simultaneously (long press)

---

## Button Functions

### Button Layout
```
[BTN1]  [BTN2]  [BTN3]  [BTN4]
PA10    PB3     PB5     PB4
```

### Global Functions (Available in Any State)
- **BTN4 (Long Press)**: Enter Calibration Menu

### State-Specific Functions

#### IDLE / MONITORING States
- **BTN1 (Short Press)**: Trigger immediate monitoring cycle

#### ALERT_LATCH State
- **BTN2 (Short Press)**: Acknowledge alert → Return to monitoring
- **BTN3 (Short Press)**: Snooze alert for 60 seconds

#### ALERT_SNOOZE State
- **BTN1 (Short Press)**: Cancel snooze → Return to monitoring

#### CAL_MENU State
- **BTN1 (Short Press)**: Set dry reference level
- **BTN2 (Short Press)**: Set wet reference level
- **BTN3 (Short Press)**: Set threshold parameters
- **BTN4 (Short Press)**: Save calibration → Return to monitoring
- **BTN4 (Long Press)**: Cancel calibration → Return to idle

#### FAULT State
- **BTN1 + BTN2 (Both Long Press)**: Clear fault and reload configuration

### Button Press Types
- **Short Press**: < 800ms
- **Long Press**: ≥ 800ms
- **Debounce Time**: 20ms

---

## Output Indicators

### 1. **LED Bar Graph** (PA5, PA6, PA7, PB6)
Displays soil moisture level in real-time using PWM-controlled brightness per segment:
- **0 LEDs**: Very low moisture (< 30%)
- **1 LED**: Low moisture (30–50%)
- **2 LEDs**: Medium moisture (50–70%)
- **3 LEDs**: High moisture (70–90%)
- **4 LEDs**: Very high moisture (≥ 90%)

Partial levels fade the active LED proportionally, giving a smooth “fill” indication.

**Alert Mode**: All 4 LEDs blink rapidly (250ms intervals)

### 2. **7-Segment Display (BCD)**
Displays current system mode:
- **0**: Boot/Idle
- **1**: Monitoring
- **2**: Alert
- **3**: Calibration
- **4**: Fault

### 3. **OLED Display** (128×64, Optional)
Shows detailed status in 4 lines:

**Line 1**: "MOISTURE CTRL" (title)
**Line 2**: Current state (e.g., "STATE:NORMAL", "STATE:ALERT VIS")
**Line 3**: Soil moisture and temperature values (e.g., "SM:0.65 T:0.50")
**Line 4**: Threshold values (e.g., "L:0.35 H:0.55")

**Special Screens**:
- **ALERT**: "ALERT ACTIVE" / "B2 ACK B3 SNOZ"
- **SNOOZE**: "SNOOZE 45s" / "B1 CANCEL"
- **CAL_MENU**: "B1 DRY B2 WET" / "B3 THR B4 SAVE"
- **CAL_SET_DRY**: "SET DRY LEVEL" / "VAL:0.20"
- **CAL_SET_WET**: "SET WET LEVEL" / "VAL:0.80"
- **CAL_SET_THRESH**: "SET THRESHOLD" / "L:0.40 H:0.60"
- **FAULT**: "FAULT STATE" / "HOLD B1+B2"

### 4. **UART Console** (115200 baud)
Debug output via PA2 (TX) provides detailed logs:
- Button events: `[Buttons] BTN1 pressed @1234ms`
- LED changes: `[LEDs] L1=1 L2=1 L3=0 L4=0 mask=0x3 @5678ms`
- Sensor readings: `[Sensors] T=0.500 L=0.650 SM=0.450 POT=0.630 @9012ms`
- State transitions: `[FSM] IDLE_STANDBY -> MON_SENSE_FILTER (idle periodic)`
- Threshold evaluations: `[Eval] Tl=0.350 Th=0.550 M=0.300 acc=3 @3456ms`

---

## Operational Workflow

### Normal Operation
1. **Boot**: System runs self-test → loads config → checks calibration
2. **Idle**: System waits in standby
3. **Monitoring**: Every 1 second (or on BTN1 press):
   - Read temperature, light, and moisture sensors
   - Apply EMA filter (α = 1/8)
   - Calculate environmental compensation
   - Update LED bar graph
   - Check if moisture is below threshold
4. **Repeat**: Continue monitoring cycle

### Alert Scenario
1. Moisture drops below `T_low` for 5 consecutive cycles
2. Enter **ALERT_VISUAL**: All LEDs blink for 3 seconds
3. Enter **ALERT_LATCH**: LEDs stay ON, waiting for user action
   - Press **BTN2**: Acknowledge → Resume monitoring
   - Press **BTN3**: Snooze for 60 seconds
4. After snooze expires: Resume monitoring

### Calibration Procedure
1. Press and hold **BTN4** (long press) to enter calibration menu
2. **Set Dry Level**:
   - Press **BTN1** (short)
   - Place sensor in dry environment
   - Current potentiometer reading is stored as dry reference
3. **Set Wet Level**:
   - Press **BTN2** (short)
   - Place sensor in wet environment
   - Current potentiometer reading is stored as wet reference
4. **Set Thresholds** (optional):
   - Press **BTN3** (short)
   - Sets `T_low=0.40`, `T_high=0.60` (hardcoded)
5. Press **BTN4** (short) to save and return to monitoring

> **Note:** Calibration values are cached in RAM only. Power-cycling the board reloads defaults until persistent storage support returns.

### Fault Recovery
If system enters **FAULT** state:
- Check UART console for error message
- Simultaneously press and hold **BTN1 + BTN2** (long press)
- System reloads default configuration

---

## Environmental Compensation

The system adjusts moisture thresholds based on ambient conditions:

**Temperature Compensation**: 
- `Adjustment = (temperature - 0.5) × 0.10`
- Warmer temps raise thresholds (more tolerance for dryness)

**Light Compensation**:
- `Adjustment = (0.5 - light_level) × 0.05`
- Brighter light lowers thresholds (stricter moisture requirements)

**Final Thresholds**:
```
T_low_adjusted  = clamp(T_low + temp_adj + light_adj, 0.0, 1.0)
T_high_adjusted = clamp(T_high + temp_adj + light_adj, 0.0, 1.0)
```

---

## Configuration Parameters

### Default Values
- **Dry Calibration**: 0.20
- **Wet Calibration**: 0.80
- **Low Threshold**: 0.35 (35% normalized moisture)
- **High Threshold**: 0.55 (55% normalized moisture)
- **Alert Accumulation**: 5 consecutive readings below threshold
- **Snooze Duration**: 60,000 ms (60 seconds)
- **Blink Rate**: 250 ms (4 Hz)
- **Monitoring Period**: 1000 ms (1 Hz)

### EMA Filter
- **Alpha**: 1/8 (12.5% new sample, 87.5% history)
- Smooths ADC noise while maintaining responsiveness

---

## Building the Project

### Prerequisites
- STM32CubeIDE 1.19.0 or later
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Make utility

### Build Commands
```bash
cd mini_project/Debug
make all
```

### Flash to Board
Use STM32CubeIDE's built-in programmer or:
```bash
st-flash write mini_project.bin 0x08000000
```

### Debug Console
Connect UART2 (PA2/PA3) to USB-UART adapter:
```bash
screen /dev/ttyUSB0 115200
# or
minicom -D /dev/ttyUSB0 -b 115200
```

The console reports telemetry status lines such as `[Telemetry] publish failed` if the ESP32 bridge does not acknowledge a frame. Continuous failures usually indicate wiring or Wi-Fi issues on the bridge side.

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| LEDs don't light | GPIO not initialized | Check `gpio_init()` in main.c |
| 7-seg shows "4" | FAULT state | Check UART console, hold BTN1+BTN2 |
| No UART output | Wrong baud rate | Verify 115200 8N1 settings |
| Telemetry retries | ESP32 not acknowledging UART frames | Check wiring on PA11/PA12, ensure ESP32 bridge is powered and Wi-Fi connected; monitor ESP32 USB serial for `[Serial]` diagnostics |
| OLED blank | I2C address wrong | Verify SSD1306 address (0x3C or 0x3D) |
| Buttons not working | Wrong pin assignment | Verify active-LOW pull-up config |
| ADC reads 0 | Clock disabled | Check RCC->APB2ENR ADC1EN bit |
| Alert won't clear | Moisture still low | Adjust potentiometer or recalibrate |

---

## File Structure

```
mini_project/
├── Src/
│   ├── main.c              # System initialization, clock config, ADC
│   ├── app_logic.c         # FSM, button handling, sensor processing
│   ├── display.c           # UART printf redirection
│   ├── oled_menu.c         # OLED text compositor (4×16 chars)
│   ├── ssd1306.c           # I2C driver for SSD1306 OLED
│   └── u8g2_port.c         # u8g2 HAL adaptation layer
├── Inc/
│   ├── board_pins.h        # Pin definitions for shield
│   ├── app_logic.h         # FSM API
│   ├── display.h           # Display API
│   ├── oled_menu.h         # OLED menu API
│   └── ssd1306.h           # SSD1306 driver API
└── Debug/
    └── Makefile            # Build automation
```

---

## Author & License
**Project**: Microcontroller Mini-Project (Moisture Monitoring System)  
**Platform**: STM32F411RE + Training Shield  
**Toolchain**: ARM GCC / STM32CubeIDE  

---

## Quick Reference Card

### Button Quick Reference
| Button | State | Action | Result |
|--------|-------|--------|--------|
| BTN1 (short) | IDLE/MON | Manual trigger | Start monitoring cycle |
| BTN1 (long) | SNOOZE | Cancel snooze | Return to monitoring |
| BTN2 (short) | ALERT | Acknowledge | Clear alert |
| BTN2 (short) | CAL_MENU | Set wet | Store wet reference |
| BTN3 (short) | ALERT | Snooze | 60s delay |
| BTN3 (short) | CAL_MENU | Set threshold | Set T_low/T_high |
| BTN4 (short) | CAL_MENU | Save | Exit calibration |
| BTN4 (long) | Any | Enter cal | Open calibration menu |
| BTN1+BTN2 (long) | FAULT | Clear fault | Reset to defaults |

### LED Indicators
- **0 LEDs**: < 30% moisture
- **1 LED**: 30–50%
- **2 LEDs**: 50–70%
- **3 LEDs**: 70–90%
- **4 LEDs**: > 90%
- **Blinking**: ALERT active

### 7-Segment Codes
- **0**: Boot/Idle
- **1**: Monitoring
- **2**: Alert
- **3**: Calibration
- **4**: Fault



# THANKS
Thanks to the following open-source projects and libraries that made this project possible:
- [STM32CubeF4](https://www.st.com/en/embedded-software/stm32cubef4.html) - STM32 HAL and peripheral drivers
- [u8g2](https://github.com/olikraus/u8g2) - Universal graphics library for monochrome displays