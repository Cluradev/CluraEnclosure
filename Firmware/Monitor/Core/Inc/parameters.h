/*
 * CLURA ENCLOSURE - RevB firmware
 */

#ifndef INC_PARAMETERS_H_
#define INC_PARAMETERS_H_
#include<stdio.h>
#include <cstdint>         // For uint8_t
#include <cstdio>

#define SERVO_ON_ANGLE  90
#define SERVO_OFF_ANGLE  0
#define OPEN_ANGLE      0
#define CLOSE_ANGLE      49

#define MIN_BRIGHTNESS 15

#define DEFAULT_LEFT_CAL 740
#define DEFAULT_RIGHT_CAL 740
#define DEFAULT_LEFT_OFFSET 8483528
#define DEFAULT_RIGHT_OFFSET 8483528
#define TARE_WEIGHT 1000 // filament weight
#define NUM_OF_FILAMENTS 24
#define LEN_BUFFER 16

#define LPG_THRESHOLD 2000//MQ2
#define SMOKE_THRESHOLD 750//MEMS
#define COUNTDOWN_TIME 5
#define SMOKE_BEGIN_TIME 300000 //milliseconds

#define MAX_TEMP 30 // Maximum temperature in Celsius before automatic flap opening (safety feature) Change to 50c

#define AUTOCOOL_TEMP_MIN 20
#define AUTOCOOL_TEMP_MAX 40
#define AUTOFILTER_IMPURITIES_MIN 5
#define AUTOFILTER_IMPURITIES_MAX 100
#define AUTOFILTER_P03_MIN 100
#define AUTOFILTER_P03_MAX 2000
#define SENSOR_USAGE_WARNING_THRSHLD 18000

#define BME_SAMPLE_INTERVAL 5000
#define USAGE_LOG_INTERVAL 10000
#define NOT_FOUND -404

// PATHS
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
