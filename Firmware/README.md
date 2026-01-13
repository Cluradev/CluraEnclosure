# Clura Enclosure - Firmware Guide

Welcome to the Clura Enclosure firmware! This guide covers everything you need to flash firmware and customize your enclosure's behavior.

---

## Quick Start

### Flashing Firmware

The easiest way to update your Clura Enclosure is using an SD card.

#### Main PCB Update

1. Copy `monitor.bin` to the root of your SD card
2. Insert the SD card into the main PCB
3. Power on the enclosure
4. Wait for the confirmation message on screen
5. The system will automatically reboot with new firmware

#### Screen Update

1. Copy `firmware.bin` to the root of your SD card
2. Insert the SD card into the screen
3. Power on the screen
4. Wait for flashing to complete
5. **Remove the SD card** and power cycle the screen

> **Note**: Never remove power during flashing!

---

## Customizing Your Enclosure

### The data.clu File

The `data.clu` file on your SD card stores all your settings. The enclosure automatically saves changes when you adjust settings via the touchscreen, but you can also edit it manually.

**Location**: Root of SD card  
**Format**: JSON

> **Important**: Always power off and safely eject the SD card before editing `data.clu` manually!

#### What You Can Easily Modify

**Display Settings**
- `brightness` (0-100): Screen brightness
- `standbyTime`: Minutes before screen turns off

**Fan Control**
- `autoCooling` (true/false): Enable automatic temperature-based cooling
- `autoFiltering` (true/false): Enable automatic air quality-based filtering
- `coolingFanSetSpeed` (0-100): Manual cooling fan speed
- `filterFanSetSpeed` (0-100): Manual filter fan speed

**Safety Features**
- `alarm` (true/false): Enable/disable fire alarm system
- `mute` (true/false): Mute alarm buzzer (alarm still functions)

**Lighting**
- `red`, `green`, `blue` (0-255): LED strip color
- `LedBrightness` (0.0-1.0): LED brightness multiplier
- `ledBar` (true/false): Enable LED bar

**Servo/Flap Control**
- `servoAngle` (0-180): Servo position (0 = open, 43 = closed)
- `autoServo` (true/false): Automatic servo control based on power state

**Data Logging**
- `logging` (true/false): Enable CSV logging to SD card
- `interval` (5, 10, or 60): Logging interval in minutes

**Feature Toggles**
- `usePms`: Enable PM sensor
- `useBme`: Enable temperature/humidity/VOC sensor
- `useMq2`: Enable gas sensor
- `useOptical`: Enable optical smoke sensor
- `useLoadCell`: Enable spool weight sensors
- `useLedStrip`: Enable LED strip

---

## How Key Features Work

### Automatic Cooling

When `autoCooling` is enabled, the cooling fan speed automatically adjusts based on temperature:

- **20°C or below**: Fan off (0%)
- **20-40°C**: Fan speed increases linearly
- **40°C or above**: Fan at maximum (100%)

**Safety Override**: If temperature exceeds 50°C, the servo flap automatically opens regardless of other settings!

### Automatic Filtering

When `autoFiltering` is enabled, the filter fan adjusts based on air quality sensors:

- Monitors both PM2.5/PM10 and PM0.3 particle counts
- Uses whichever sensor detects worse air quality
- Minimum speed is 15% (never fully off in auto mode)
- Speed ranges from 15-100% based on particle concentration

### Fire Alarm System

The enclosure uses three sensors for fire detection:
1. **Optical smoke sensor**: Detects visible smoke
2. **MQ2 gas sensor**: Detects LPG, propane, and smoke
3. **MEMS sensor**: Additional smoke detection

**Important**: The alarm has a 3-minute warm-up period after power-on to prevent false alarms.

When triggered:
- Buzzer sounds (unless `mute` is true)
- LED strip turns red
- Alarm flag is set

---

## Spool Holder System

The dual spool holders track your filament weight in real-time using load cells.

### How It Works

1. **Calibrate the load cells** (one-time setup via touchscreen)
2. **Select your filament brand** from the list (or add custom)
3. **The system calculates**: `Remaining Filament = Current Weight - Empty Spool Weight`

### Filament Profiles

The enclosure comes with 24 pre-configured filament brands stored in `filament.clu`:
- Sunlu, Prusament, Bambu Lab, eSUN, PolyMaker, Hatchbox, and more

Each profile stores:
- Brand name
- Empty spool weight (in grams)

### Adding Custom Filament

Via the touchscreen calibration menu:
1. **Tare**: Remove spool and zero the scale
2. **Weigh**: Place full spool to measure total weight
3. **Name**: Enter your filament brand name
4. **Save**: Profile is added to `filament.clu`

### Tare Function

The tare function zeros out the current weight. Use it when:
- Changing spools
- Recalibrating
- Starting fresh with a new filament

---

## Advanced Configuration

### Compile-Time Settings (parameters.h)

Some settings require recompiling the firmware. These are defined in `Core/Inc/parameters.h`:

**Temperature Thresholds**
- `MAX_TEMP` (50°C): Safety override temperature
- `AUTOCOOL_TEMP_MIN` (20°C): Cooling fan starts
- `AUTOCOOL_TEMP_MAX` (40°C): Cooling fan at max

**Air Quality Thresholds**
- `AUTOFILTER_IMPURITIES_MIN` (5): Filter fan activation threshold
- `AUTOFILTER_IMPURITIES_MAX` (120): Filter fan max speed threshold

**Alarm Thresholds**
- `LPG_THRESHOLD` (2000): MQ2 sensor trigger level
- `SMOKE_THRESHOLD` (500): MEMS sensor trigger level

**Servo Angles**
- `OPEN_ANGLE` (0°): Fully open position
- `CLOSE_ANGLE` (43°): Fully closed position

To change these, edit `parameters.h`, rebuild the firmware in STM32CubeIDE, and reflash.

---

## Data Logging

Enable logging via the touchscreen to track environmental data over time.

**Log File**: `log.csv` on SD card

**Data Recorded**:
- Temperature, humidity, pressure
- PM2.5, PM10, PM0.3 particle counts
- VOC (air quality index)
- Smoke sensor readings
- Fan speeds
- Spool weights

**Intervals**: Choose 5, 10, or 60 minutes

---

## Troubleshooting

**SD Card Not Recognized**
- Format as FAT32
- Insert before powering on
- Check for physical damage

**Sensors Not Working**
- Verify connections
- Enable sensor in Features menu (`useXXX` flags)
- Check System Check page for sensor status

**Load Cells Inaccurate**
- Recalibrate via touchscreen
- Perform tare with empty holder
- Check for mechanical interference

**False Fire Alarms**
- Wait 3 minutes after power-on (warm-up period)
- Check sensor readings in Sensors page
- Adjust thresholds in `parameters.h` if needed

---

## Support

For more information, visit [clura.dev](https://www.clura.dev)

*Last Updated: January 2026*
