/*
 * CLURA ENCLOSURE - RevB firmware
 */

#ifndef INC_PARAMETERS_H_
#define INC_PARAMETERS_H_
#include<stdio.h>
#include <cstdint>         // For uint8_t
#include <cstdio>

// Physical flap travel is a 39 deg swing: 5 deg = OPEN, 44 deg = CLOSED.
// All commanded angles MUST stay within [5, 44] or the servo drives into a hard stop.
// The value 0 is RESERVED as SERVO_OFF (see below), so every real position stays > 0.
// NOTE: these are only defaults - servoOpenAngle / servoCloseAngle in data.clu
// override them at runtime, so the two must describe the same orientation.
#define OPEN_ANGLE       5   // fully-open / vent position
#define CLOSE_ANGLE       44   // closed position

// Manual-mode sentinel: ms.servoAngle == SERVO_OFF means "servo off / not manually positioned".
// In this state the fan-page OPEN/CLOSE button is authoritative (ms.fanServoState).
#define SERVO_OFF         0
#define SERVO_TEMP_HYST   3   // deg C hysteresis for the overtemp vent-open safety
#define SERVO_HOLD_MS     350 // ms to drive the flap before detaching (travel ~450 ms + margin)

#define EXTRA_FAN_SPEED 75 // %, fixed speed of the regulated "extra" fan when on
#define MIN_BRIGHTNESS 15

#define DEFAULT_LEFT_CAL 740
#define DEFAULT_RIGHT_CAL 740
#define DEFAULT_LEFT_OFFSET 8483528
#define DEFAULT_RIGHT_OFFSET 8483528
#define TARE_WEIGHT 1000 // filament weight
#define NUM_OF_FILAMENTS 24
#define LEN_BUFFER 16

/* Fire response. While smoke or gas is detected: stop every fan AND close the
 * flap, so the enclosure starves a fire of oxygen instead of ventilating it.
 * Deliberately outranks the overtemp vent-open - see applyServoControl().
 * 1 = enabled (default). */
#define SMOKE_FIRE_RESPONSE 1

/* Optical smoke sensor -> alarm. Default 0 (deactivated): the sensor input
 * is configured with a pull-up and reads HIGH = "smoke", so a missing or
 * disconnected sensor floats high and would hold the alarm on permanently.
 * Set opticalAlarmEnable = 1 in data.clu once a sensor is fitted. */
#define OPTICAL_ALARM_ENABLE 0

#define LPG_THRESHOLD 2000//MQ2
#define SMOKE_THRESHOLD 750//MEMS
#define COUNTDOWN_TIME 5
#define SMOKE_BEGIN_TIME 600000 //milliseconds

#define MAX_TEMP 55 // deg C: flap forced open above this (see maxTemp in data.clu, 35..80)

#define AUTOCOOL_TEMP_MIN 20
#define AUTOCOOL_TEMP_MAX 40
#define AUTOFILTER_IMPURITIES_MIN 5
#define AUTOFILTER_IMPURITIES_MAX 100
#define AUTOFILTER_P03_MIN 100
#define AUTOFILTER_P03_MAX 2000
/* Filter/sensor replacement warning, in HOURS of filter-fan runtime.
 * Expressed in hours to match the usage figure shown on the settings page */
#define SENSOR_USAGE_WARNING_HOURS 600

#define BME_SAMPLE_INTERVAL 5000
#define GAS_SAMPLE_INTERVAL_MS 250 // ms between MQ2/MEMS ADC sample bursts
#define USAGE_LOG_INTERVAL 900000 // 15 min between usage saves (EEPROM wear)
#define NOT_FOUND -404

// EEPROM (AT24C256 — 32 KB, I²C, A2/A1/A0 all open → address 0x50)
#define EEPROM_I2C_ADDR              0x50U   /**< 7-bit I²C address                      */
#define EEPROM_PAGE_SIZE             64U     /**< Write page size in bytes                */
#define EEPROM_WRITE_DELAY_MS        6U      /**< Write-cycle wait time (datasheet: 5 ms) */
#define EEPROM_CAPACITY_BYTES        32768U  /**< Total capacity (AT24C256)               */

// EEPROM memory map
#define EEPROM_MAGIC_VALUE           0xC1A4U /**< Sentinel to detect valid data           */
#define EEPROM_CONFIG_VERSION        1U      /**< Increment when EepromConfig layout changes */
#define EEPROM_ADDR_MAGIC            0x0000U /**< uint16_t magic word       (2 B)         */
#define EEPROM_ADDR_VERSION          0x0002U /**< uint16_t config version   (2 B)         */
#define EEPROM_ADDR_CONFIG           0x0004U /**< EepromConfig struct       (~73 B)       */
#define EEPROM_ADDR_FILAMENT_BRAND   0x0080U /**< char[24][16] brand names  (384 B)       */
#define EEPROM_ADDR_FILAMENT_WEIGHT  0x0200U /**< int32_t[24] spool weights (96 B)        */
/* Runtime tuning block (see tuning.h). Deliberately placed well clear of the
 * filament weights (which end at 0x0260) and given its own magic/version/CRC,
 * so the existing config block needs no migration and no version bump. */
/* Boot counter for the CSV log. Its own address so incrementing it each boot
 * never rewrites the config or tuning blocks. */
#define EEPROM_ADDR_BOOTID           0x0280U /**< EepromBootId struct       (6 B)         */
/* Machine-usage counter. Its own record so persisting it never rewrites the
 * 73-byte config block (which was wearing the EEPROM out in ~115 days). */
#define EEPROM_ADDR_USAGE            0x0290U
/* Revision of the filament library last imported from filament.clu. Own record
 * so writing it never disturbs the config, usage or tuning blocks. */
#define EEPROM_ADDR_FILAMENT_REV     0x02A0U

#define EEPROM_ADDR_TUNING           0x0300U /**< EepromTuning struct       (~80 B)       */

// PATHS  (SD card — logging and OTA only)
#define CSV_PATH "log.csv"
#define CONFIG_PATH "data.clu"
#define FILAMENT_PATH  "filament.clu"

// COMMAND CODES TYPES
#define BACK   0
#define OK     1
#define CANCEL 2
#define SLIDER 3
#define BUTTON 4

// PAGES
#define SETTINGS 0
#define HOMESCREEN 1
#define LIGHTING 2
#define FANCTRL 3
#define SPOOLHOLDERS 4
#define SENSORS 5
#define OUTPUT_CTRL 6
#define SUPPORT 7
#define LNDSCREEN 8
#define LNDSCREEN2 9
#define CHOOSE_CONFIG 10
#define FEATURES 11
#define SYS_CHECK 12
#define SELECTOR 13
#define CAL_SEQUENCE1 14
#define CAL_SEQUENCE2 15
#define CAL_SEQUENCE3 16
#define CAL_SEQUENCE4 17
#define CAL_FINISH_PAGE 18
#define CALIBRATE1 19
#define CALIBRATE2 20
#define CALIBRATE3 21
#define CALIBRATE4 22
#define CONFIG_SAVE_FAIL 23
#define CALIBRATION_SAVE_SUCCESS 24
#define CALIBRATION_SAVE_FAIL 25
#define FORMAT_SUCCESS 26
#define FORMAT_FAIL 27

#define UPDATE_PAGE1 31
#define UPDATE_PASS 32
#define UPDATE_FAIL 33

// IMAGE IDs For the Check status on the Settings Page
#define MUTE 144
#define UNMUTE 149
#define ALARM 155
#define NO_ALARM 156
#define SENSOR_SAFE_PIC 146
#define SENSOR_UNSAFE_PIC 145
// IMAGE IDs For the Check status on the sensors Page
#define INTERVAL_ON 44
#define INTERVAL_OFF 45
#define MIN_5_LOG 46
#define MIN_10_LOG 47
#define MIN_60_LOG 48

// IMAGE IDs For the Check status on the system check Page
#define SUCCESS_IMAGE_ID 108
#define FAIL_IMAGE_ID 107
#define RUNNING_IMAGE_ID 106
#define NULL_IMAGE_ID 105

// IMAGE IDs For the Features Selection Page
#define PMS_IMAGE_ID 79
#define BME_IMAGE_ID 80
#define OPTICAL_SMOKE_SENSOR_IMAGE_ID 81
#define MQ2_IMAGE_ID 82
#define LOAD_CELL_IMAGE_ID 83
#define FILTER_FAN_IMAGE_ID 84
#define COOLING_FAN_IMAGE_ID 85
#define RGB_LED_IMAGE_ID 86
#define LED_STRIP_IMAGE_ID 87

// IMAGE IDs For the selected Features on the Selection Page
#define SELECTED_PMS_IMAGE_ID 88
#define SELECTED_BME_IMAGE_ID 89
#define SELECTED_OPTICAL_SMOKE_SENSOR_IMAGE_ID 90
#define SELECTED_MQ2_IMAGE_ID 91
#define SELECTED_LOAD_CELL_IMAGE_ID 92
#define SELECTED_FILTER_FAN_IMAGE_ID 93
#define SELECTED_COOLING_FAN_IMAGE_ID 94
#define SELECTED_RGB_LED_IMAGE_ID 95
#define SELECTED_LED_STRIP_IMAGE_ID 96


//


typedef struct {
	bool mute;
	bool alarm;
	int standbyTime;
	int interval;
	bool logging;
	uint8_t brightness;
	int32_t leftSpoolWeight;
	int32_t rightSpoolWeight;
	int32_t temperatureBaseline;
	int32_t impuritiesBaseline;
	int32_t usage_minutes;

//filament and spool
	uint8_t leftSpoolIndex;
	uint8_t rightSpoolIndex;
	uint8_t leftSelectFilament;
	uint8_t rightSelectFilament;
	uint8_t filamentSelectPage;
	uint8_t filamentIndex;
	uint8_t filamentIndexMax;
	uint8_t filamentPage;
	uint8_t filamentPageMax;
	int  leftFilament;
	int  rightFilament;
	long leftCalFactor;
	long rightCalFactor;
	long leftOffset;
	long rightOffset;

	//filament and spool
	uint8_t Impurities;
	uint8_t lightIndex;
	float LedBrightness;
	uint8_t red;
	uint8_t blue;
	uint8_t green;
	bool ledBar;

	int filterFanASpeed;
	int filterFanBSpeed;
	uint8_t filterFanSetSpeed;
	uint8_t coolingFanSpeed;
	uint8_t coolingFanSetSpeed;
	bool autoCooling;
	bool manCooling;
	bool autoFiltering;
	bool manFiltering;
	bool isFanRegOn;

	int pm03;
	int pm2;
	int pm10;
	int pressure;
	int voc;
	int temperature;
	int HumidityBME;
	int HumidityPMS;
	int mqSmokeValue;
	int memsSmokeValue;

	//Servo
	uint8_t servoAngle;
	bool autoServo;
	bool servoPower;
	uint8_t fanServoState; // Fan page servo state (OPEN_ANGLE or CLOSE_ANGLE)

	bool relay;

	uint8_t features; //0:Regular,1:Pro,2:Custom
	bool usePms;
	bool useBme;
	bool useMq2;
	bool useOptical;
	bool useLedStrip;
	bool useLedBar;
	bool useLoadCell;
	bool useMems;
	bool useFilterFan;
	bool useCoolingFan;
	char qrUrl[64];
	bool calibrationComplete;
	uint8_t currentPage;
} MachineState;

extern MachineState ms;

extern char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER];
extern int filamentWeight[NUM_OF_FILAMENTS];
#endif /* INC_PARAMETERS_H_ */
