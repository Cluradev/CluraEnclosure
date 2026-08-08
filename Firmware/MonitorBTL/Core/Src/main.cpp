/*
 * CLURA ENCLOSURE - RevB firmware
 */

/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SmokeSensor.h"
#include "A_SmokeSensor.h"
#include "Servo.h"
#include "parameters.h"
#include "HX711.h"
#include "Screen.h"
#include "Motor.h"
#include "WS281x.h"
#include "Output.h"
#include <cstdio>
#include <string.h>
#include "bme680/bme68x_necessary_functions.h"
#include "debug_utils.h"
#include "eeprom_storage.h"
#include "tuning.h"
#include "filament_store.h"
#include "AT24C_EEPROM.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "fatfs.h"
#include "cJSON.h"
#ifdef __cplusplus
}
#endif
#define WAIT HAL_Delay(80)
#define AUTO_FAN_ACTION(autoButton,manButton,manSwitchtext,autoFlag,manFlag) \
        	tjc.setVal(autoButton, autoFlag);\
        	HAL_Delay(20);\
			tjc.setVal(manButton, manFlag);\
			HAL_Delay(20);\
			manFlag?tjc.setText(manSwitchtext, "ON"):tjc.setText(manSwitchtext, "OFF");\
			HAL_Delay(20);

#define SHOW_CONT_BTN   tjc.setAph("p8", 0);\
						tjc.setAph("p9", 0);\
						tjc.setAph("p10", 127);\
						tjc.setAph("p12", 0)
#define HIDE_CONT_BTN   tjc.setAph("p8", 127);\
						tjc.setAph("p9", 127);\
						tjc.setAph("p10", 0);\
						tjc.setAph("p12", 127)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define PMS_FRAME_LENGTH 32
#define SCREEN_BUFFER_SIZE 32
#define DATA_BUFFER_SIZE 20
#define LED_COUNT 100
#define MINUTE_CONST 60000 //TODO: Change back to 60000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* DATA BUFFERS  */
volatile uint8_t pms_data[PMS_FRAME_LENGTH]; // Buffer for pms_data frame
volatile uint8_t rxBuffer[SCREEN_BUFFER_SIZE];  // Buffer to store received data

uint8_t dataBuffer[DATA_BUFFER_SIZE];
uint8_t rxDataScreen;                 // Single-byte buffer for interrupt
uint8_t rxDataPms;                 // Single-byte buffer for interrupt
volatile uint16_t rxIndexScreen = 0;           // Current index in the buffer
volatile uint16_t rxIndexPms = 0;           // Current index in the buffer
volatile uint8_t dataReceivedScreen = 0;
volatile uint8_t dataReceivedPms = 0;
volatile uint8_t endCount = 0;

//uint8_t pulseReg = 60;
//uint8_t pulse60 = 60;
int8_t countdown = COUNTDOWN_TIME;         // WAIT TIMER BEFORE CONTINUE POPS UP
int osc_per_rev = 2;
bool refreshFlag = 1;
bool screenDimmed = false;
bool forceUpdate = true;

uint32_t standbyTimestamp = 0;
uint32_t dataLoggerTimestamp = 0;
/* Boot session id, read from EEPROM at startup. Logged with every CSV row so
 * rows stay orderable across power cycles (HAL_GetTick restarts at 0). */
uint32_t g_bootId = 1;
/* Actual commanded filter-fan speed (0-100). Set by every branch of the fan
 * control block, so it reflects auto AND manual mode - unlike
 * ms.filterFanSetSpeed, which only the manual slider ever writes. */
int filterFanDisplaySpeed = 0;

/* Overtemp latch, owned by applyServoControl() but read by the fan control
 * block, which runs later in the same pass. */
bool g_ventOpenOvertemp = false;

/* Machine-usage accounting. usage_minutes = g_usageBase + accumulated runtime.
 * Persisted in its own EEPROM record (see EEPROM_SaveUsage). */
uint32_t g_usageBase    = 0;   /* minutes already banked in EEPROM      */
uint32_t g_usageAccumMs = 0;   /* runtime accumulated since that base   */
uint32_t g_usageSaved   = 0;   /* last value actually written to EEPROM */


 bool alarm = 0;
bool refreshed_alarm = 0;
bool refresh_led_flag=1;
short coolingFanState = 0;
const int numOfButtons = 4;
int leftIDMaps[numOfButtons] = { 6, 8, 10, 12 }; //IDs of left check boxes to manage the states
int rightIDMaps[numOfButtons] = { 7, 9, 11, 13 }; //IDs of right check boxes to manage the states
int leftIDNums[numOfButtons] = { 0, 2, 4, 6, }; //Numbers of check boxes to manage the states
int rightIDNums[numOfButtons] = { 1, 3, 5, 7 }; //Numbers of check boxes to manage the states
volatile unsigned long filterFanACounter = 0;
volatile unsigned long filterFanBCounter = 0;
extern volatile bool ws281x_done;
bool sw = 0; //value to hold the switch state for the loop flap/gate
bool checkedPms = false;
bool checkedBme = false;
bool checkedOSS = false;
bool checkedMq2 = false;
bool checkedLC = false;
bool checkedFFan = false;
bool checkedCFan = false;
bool checkedRGB = false;
bool checkedLED = false;
bool runOnce = true;

bool bmeIsFound = false;



struct bme68x_data data;

typedef struct {
	uint16_t pm1_0_cf1;
	uint16_t pm2_5_cf1;
	uint16_t pm10_cf1;
	uint16_t pm1_0_atm;
	uint16_t pm2_5_atm;
	uint16_t pm10_atm;
	uint16_t particles_0_3;
	uint16_t particles_0_5;
	uint16_t particles_1_0;
	uint16_t particles_2_5;
	uint16_t particles_5_0;
	uint16_t particles_10;
} PmsData;

PmsData pms_info { 0 };

/* PERIPHERAL OBJECTS   */

HX711 loadCellLeft(HX1DT_GPIO_Port, HX1DT_Pin, HX1SCK_GPIO_Port, HX1SCK_Pin);
HX711 loadCellRight(HX2DT_GPIO_Port, HX2DT_Pin, HX2SCK_GPIO_Port, HX2SCK_Pin);
SmokeSensor smokeSensor(ALARM_GPIO_Port, ALARM_Pin, MUTE_GPIO_Port, MUTE_Pin);
A_SmokeSensor mqSensor( MQ2_GPIO_Port, MQ2_Pin, &hadc1, ADC_CHANNEL_6);
A_SmokeSensor memsSensor( MEMS_SMOKE_GPIO_Port, MEMS_SMOKE_Pin, &hadc1,
ADC_CHANNEL_7);
Servo servo(&htim5, TIM_CHANNEL_1);
Output ssr1(SSR1_GPIO_Port, SSR1_Pin);
Output ssr2(SSR2_GPIO_Port, SSR2_Pin);
Output buzzer(BUZZER_GPIO_Port, BUZZER_Pin);
Screen tjc(&huart2);

Motor coolingFan(&htim3, 3);
Motor filterFan(&htim3, 4);
Motor fanReg(&htim2, 3);

WS281x ledStrip(&htim1, TIM_CHANNEL_1);
ScreenEvent event;

// RUNTIME PARAMS
char debugBuffer[128];
bool recievingPms = false;
bool saveFlag = false; //save config? this is a runtime variable do not modify
int leftCalWeight = 1200; // weight variable read by the sensor during the calibration
int rightCalWeight = 1200;
int customSpoolWeight = 0;
uint32_t smokeBeginTimestamp = 0;
uint32_t BME_Timestamp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
bool isValidPmsData(volatile uint8_t *buffer, int size);
bool readBme();
void debug_pms_data(PmsData *data);
void process_pms_data(volatile uint8_t *buffer, PmsData *data);
void handleHomeScreen(ScreenEvent event);
void handleFanControl(ScreenEvent event);
void handleSensors(ScreenEvent event);
void handleSpoolHolders(ScreenEvent event);
void handleOutput(ScreenEvent event);
void handleLighting(ScreenEvent event);
void handleSupport(ScreenEvent event);
void handleLndScreen2(ScreenEvent event);
void handleConfig(ScreenEvent event);
void handleCustomFeatures(ScreenEvent event);
void handleSystemCheck(ScreenEvent event);
void handleSelector(ScreenEvent event);

void handleCalSequence1(ScreenEvent event);
void handleCalSequence2(ScreenEvent event);
void handleCalSequence3(ScreenEvent event);
void handleCalSequence4(ScreenEvent event);

void handleCalibrate1(ScreenEvent event);
void handleCalibrate2(ScreenEvent event);
void handleCalibrate3(ScreenEvent event);
void handleCalibrate4(ScreenEvent event);

void handleDone(ScreenEvent event);
void handleSettings(ScreenEvent event);
void handleConfigSaveFail(ScreenEvent event);
void handleCalibrationSaveFail(ScreenEvent event);
void handleCalibrationSaveSuccess(ScreenEvent event);
void handlePageEvent(ScreenEvent event);
void handleFormatFail(ScreenEvent event);
void handleFormatSuccess(ScreenEvent event);

float constrain(float val, float min, float max);
void runMachine(void);
void applyServoControl(void);
void updateUsageCounter(void);
bool csvInit(void);
bool csvLog(uint32_t bootId, int t, int temperature, int humidity, int pressure, int smoke,int pm03,
		int pm2_5, int pm10, int voc, int lpg,int fanSpd,int lWeight, int rWeight);
/* SD card: csvDelete used by handleSensors (format button) */
bool csvDelete(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* D-20: one file-scope FATFS shared by the csv helpers.
 * f_mount() stores this pointer in FatFs' internal drive table and keeps using
 * it after the call returns, so the stack-local FATFS each of these used to
 * declare left a dangling pointer the moment the function exited. It only ever
 * worked because every one of them re-mounts before touching the card - a trap
 * for any future code that does not. */
static FATFS s_csvFatFs;

bool csvInit(void) {
	FATFS &FatFs = s_csvFatFs;
	FIL file;
	FRESULT fres;
	//check if log is in sd car d
	fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK)
		return false;
	fres = f_open(&file, CSV_PATH, FA_READ);
	if (fres != FR_OK) {
		//create if not in sd card
		fres = f_open(&file, CSV_PATH, FA_WRITE | FA_CREATE_ALWAYS);
		if (fres == FR_OK) {
			UINT bw;

			// **FIXED LINE:**
			// Reordered the header to match the csvLog function's parameters
			const char *header =
					"BootID,MinSinceBoot,Temperature,Humidity,Pressure,Smoke,PM 0.3,PM 2.5,PM 10,VOC,LPG,Fan Speed,Left Weight,Right Weight\n";

			f_write(&file, header, strlen(header), &bw);
			f_close(&file);
			return true;

		}
	} else {
		f_close(&file);
		return true;

	}
	return false;
}
bool csvDelete(void) {
	FATFS &FatFs = s_csvFatFs;
	FIL file;
	FRESULT fres;
	//check if log is in sd car d
	fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK)
		return false;
	fres = f_open(&file, CSV_PATH, FA_READ);
	if (fres == FR_NO_FILE) {
		return true;
	}
	fres = f_close(&file);
	fres = f_unlink(CSV_PATH);
	if (fres == FR_OK) {
		return true;
	}
	return false;
}
bool csvLog(uint32_t bootId, int t, int temperature, int humidity, int pressure, int smoke, int pm03,
		int pm2_5, int pm10, int voc, int lpg, int fanSpd, int lWeight, int rWeight) {
	FATFS &FatFs = s_csvFatFs;

	FIL file;
	FRESULT fres;
	UINT bw;
	char line[256];
	fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK)
		return false;
	// Open the CSV file in append mode
	fres = f_open(&file, CSV_PATH, FA_WRITE | FA_OPEN_APPEND);
	if (fres == FR_OK) {
		// **FIXED LINE:**
		// Updated to include all 13 parameters in the correct order.
		/* D-14: `smoke` and `lpg` used to be written the other way round, so the
		 * Smoke column carried MQ2 gas readings and the LPG column carried MEMS
		 * smoke readings. Rows logged before this change stay swapped. */
		snprintf(line, sizeof(line), "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				(unsigned long)bootId, t, temperature, humidity, pressure, smoke, pm03,
				pm2_5, pm10, voc, lpg, fanSpd, lWeight, rWeight);

		f_write(&file, line, strlen(line), &bw);
		f_close(&file);
		return true;

	}
	return false;

}


/* ──────────────────────────────────────────────────────────────────────────
 * NOTE: saveConfig / loadConfig / saveFilamentData / loadFilamentData
 *       have been removed.  Config and filament data are now stored on the
 *       AT24C256 EEPROM via EEPROM_SaveConfig / EEPROM_LoadConfig /
 *       EEPROM_SaveFilamentData / EEPROM_LoadFilamentData (eeprom_storage.cpp).
 *
 *       SD card is retained ONLY for:
 *         • csvInit / csvLog / csvDelete  (sensor data logging)
 *         • OTA firmware binary           (reserved, future use)
 * ────────────────────────────────────────────────────────────────────────── */

void handlePageEvent(ScreenEvent event) {


	switch (event.pageNumber) {
	case HOMESCREEN:
		handleHomeScreen(event);
		break;
	case FANCTRL:
		handleFanControl(event);
		break;
	case SENSORS:
		handleSensors(event);
		break;
	case SPOOLHOLDERS:
		handleSpoolHolders(event);
		break;
	case LIGHTING:
		handleLighting(event);
		break;
	case SETTINGS:
		handleSettings(event);
		break;
	case OUTPUT_CTRL:
		handleOutput(event);
		break;
	case SUPPORT:
		handleSupport(event);
		break;
	case LNDSCREEN2:
		handleLndScreen2(event);
		break;
	case CHOOSE_CONFIG:
		handleConfig(event);
		break;
	case FEATURES:
		handleCustomFeatures(event);
		break;
	case SYS_CHECK:
		handleSystemCheck(event);
		break;
	case SELECTOR:
		handleSelector(event);
		break;
	case CAL_SEQUENCE1:
		handleCalSequence1(event);
		break;
	case CAL_SEQUENCE2:
		handleCalSequence2(event);
		break;
	case CAL_SEQUENCE3:
		handleCalSequence3(event);
		break;
	case CAL_SEQUENCE4:
		handleCalSequence4(event);
		break;
	case CAL_FINISH_PAGE:
		handleDone(event);
		break;
	case CALIBRATE1:
		handleCalibrate1(event);
		break;
	case CALIBRATE2:
		handleCalibrate2(event);
		break;
	case CALIBRATE3:
		handleCalibrate3(event);
		break;
	case CALIBRATE4:
		handleCalibrate4(event);
		break;
	case CALIBRATION_SAVE_FAIL:
		handleCalibrationSaveFail(event);
		break;
	case CALIBRATION_SAVE_SUCCESS:
		handleCalibrationSaveSuccess(event);
		break;
	case CONFIG_SAVE_FAIL:
		handleConfigSaveFail(event);
		break;
	case FORMAT_FAIL:
			handleFormatFail(event);
			break;
	case FORMAT_SUCCESS:
		handleFormatSuccess(event);
			break;
		break;
	}
}

//------------------------------------CALIBRATION--------------------------------------

float constrain(float val, float min, float max) {
	if (val < min) {
		val = min;
	} else if (val > max) {
		val = max;
	}
	return val;
}
//------------------------------------HOMESCREEN------------------------------------
void handleHomeScreen(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		tjc.setPage(OUTPUT_CTRL, ms);
		break;
	case 3:
		tjc.setPage(SETTINGS, ms);
		break;
	case 4:
		tjc.setPage(FANCTRL, ms);
		break;
	case 6:
		tjc.setPage(LIGHTING, ms);
		break;
	case 5:
		tjc.setPage(SPOOLHOLDERS, ms);
		break;
	case 7:
		tjc.setPage(SENSORS, ms);
		break;
	default:
		break;
	}
forceUpdate =true;
}

// Servo control. Called every loop pass. The fan-page OPEN/CLOSE button (ms.fanServoState) is
// the only control. While moving we re-assert the pulse every pass (the technique that reliably
// drove the servo); g_tune.servoHoldMs after a new command we detach (compare = 0) to stop the
// holding current. The boot drive is intentionally disabled here (firstCall) - the servo stays
// idle until the button commands it.
void applyServoControl() {
	static uint8_t  lastAngle = 0xFF;   // last angle we started driving to (0xFF = none yet)
	static uint32_t moveStart = 0;      // tick when the current move started
	static bool     driving   = false;  // are we still asserting the pulse?
	static bool     firstCall = true;
	bool           &ventOpen  = g_ventOpenOvertemp;  // overtemp latch (file scope)

	/* ── Overtemp safety ───────────────────────────────────────────────────
	 * If the enclosure exceeds maxTemp, force the flap open regardless of what
	 * the fan-page button says, and keep it there until the temperature has
	 * dropped by servoTempHyst. The hysteresis stops a reading sitting right on
	 * the threshold from repeatedly cycling the servo.
	 * Requires the BME680 to be enabled and reporting - with no temperature
	 * source there is nothing to act on, so the latch is cleared. */
	if (ms.useBme && ms.temperature != NOT_FOUND) {
		if (ms.temperature > (int)g_tune.maxTemp) {
			ventOpen = true;
		} else if (ms.temperature < (int)g_tune.maxTemp - (int)g_tune.servoTempHyst) {
			ventOpen = false;
		}
	} else {
		ventOpen = false;
	}

	/* ── Flap priority ─────────────────────────────────────────────────────
	 *
	 *   1. SMOKE   -> close.  Highest priority. A fire raises temperature too,
	 *                 so without this the overtemp rule below would open the
	 *                 vent and feed the fire the oxygen the fan cutoff was
	 *                 trying to deny it. Smoke is the stronger signal: heat
	 *                 alone means "hot", heat WITH smoke means "burning".
	 *   2. OVERTEMP-> open.   Hot but no smoke: vent the heat as before.
	 *   3. otherwise the fan-page OPEN/CLOSE button decides.
	 *
	 * Uses the raw `alarm` flag, so muting the buzzer cannot reopen the vent. */
	uint8_t desired;
	if (alarm && g_tune.smokeFireResponse) {
		desired = g_tune.servoCloseAngle;
	} else if (ventOpen) {
		desired = g_tune.servoOpenAngle;
	} else {
		desired = ms.fanServoState;
	}

	if (firstCall) {
		// Adopt the boot state WITHOUT driving the servo (boot drive deactivated).
		firstCall = false;
		lastAngle = desired;
	}

	if (desired != lastAngle) {
		// New command from the button: (re)start the move and the hold timer.
		lastAngle = desired;
		moveStart = HAL_GetTick();
		driving = true;
	}

	if (driving) {
		if ((HAL_GetTick() - moveStart) < g_tune.servoHoldMs) {
			servo.setAngle(desired);   // keep re-asserting the pulse toward the target
		} else {
			servo.detach();            // travel time elapsed -> cut the pulse, stop re-asserting
			driving = false;
		}
	}
}

/*
 * Machine-usage accounting.
 *
 * Accumulates elapsed time while the filter fan is actually running and
 * persists it to its own small EEPROM record.
 *
 * Replaces a version that recomputed usage from a bootTick which was reset on
 * every fan stop, while subtracting a reset marker measured on a different time
 * base. That made the counter run backwards whenever the fan was cycled, and go
 * negative after the reset button was used. Accumulating forward cannot do
 * either.
 *
 * Writes are the other half of the fix: the old code called EEPROM_SaveConfig()
 * every 10 s while the fan ran - 8640 writes/day, enough to wear the EEPROM out
 * in about 115 days. Now it writes its own 10-byte record, at most every
 * usageLogInterval (15 min) and only when the value has actually changed, plus
 * once when the fan stops so the tail of each run is not lost.
 */
void updateUsageCounter(void)
{
	static uint32_t lastTick     = 0;
	static uint32_t lastSaveTick = 0;
	static bool     wasRunning   = false;

	uint32_t now = HAL_GetTick();

	/* Gate on the ACTUAL commanded speed so auto-filtering counts too. Using
	 * ms.filterFanSetSpeed meant a machine left in auto mode accrued no usage
	 * at all, so the sensor-replacement warning never fired on the units that
	 * ran the most. */
	bool running = (filterFanDisplaySpeed > 0);

	if (running) {
		if (!wasRunning) {
			lastTick     = now;    /* start of a run - do not count the idle gap */
			lastSaveTick = now;
		}
		g_usageAccumMs += (now - lastTick);
		lastTick = now;

		/* Roll whole minutes into the running total and keep only the
		 * sub-minute remainder in the accumulator. Letting milliseconds pile up
		 * indefinitely would wrap the uint32 after 49.7 days of CONTINUOUS
		 * running and throw the counter back to its starting value - reachable
		 * now that auto-filtering counts toward usage too. */
		while (g_usageAccumMs >= MINUTE_CONST) {
			g_usageAccumMs -= MINUTE_CONST;
			g_usageBase++;
		}
		ms.usage_minutes = (int32_t)g_usageBase;
	}

	bool stopping = (wasRunning && !running);
	bool changed  = ((uint32_t)ms.usage_minutes != g_usageSaved);

	if (changed && (stopping ||
	                (running && (now - lastSaveTick >= g_tune.usageLogInterval)))) {
		if (EEPROM_SaveUsage((uint32_t)ms.usage_minutes)) {
			g_usageSaved = (uint32_t)ms.usage_minutes;
			lastSaveTick = now;
		}
	}

	wasRunning = running;
}

/* Optical smoke sensor, gated by the tuning flag.
 *
 * The input is configured with a pull-up and treats HIGH as "smoke", so an
 * absent or disconnected sensor floats high and reports smoke forever. With
 * opticalAlarmEnable = 0 the sensor is ignored entirely and only the MQ2 and
 * MEMS sensors can raise the alarm. */
static bool opticalSmokeActive(void)
{
	return (g_tune.opticalAlarmEnable != 0) && smokeSensor.isSmokeDetected();
}

/* Median of three - rejects a single spike without the lag of an average. */
static long medianOf3(long a, long b, long c)
{
	if ((a > b) != (a > c)) return a;
	if ((b > a) != (b > c)) return b;
	return c;
}

void runMachine() {
#define ADD_UNIT(component,val,unit,buf) \
		do{\
		snprintf(buf, sizeof(buf), "%ld%s",(int32_t)val, unit);\
		tjc.setText(component, buf);\
		}while(0)
	static bool lastSmoke = false;
	static int lastLogging = -1;
	static int lastInterval = -1;
	// Initialize to default values - will be updated on first run
	static int last_pm2 = NOT_FOUND;
	static int last_pm10 = NOT_FOUND;
	static int last_pressure = NOT_FOUND;
	static int last_voc = NOT_FOUND;
	static int last_temperature = NOT_FOUND;
	static uint8_t last_HumidityBME = 0;
	static int last_mqSmokeValue = NOT_FOUND;
	static int last_memsSmokeValue = NOT_FOUND;
	static int last_isFanRegOn = -1;
	static int last_autoFiltering = -1;
	static int last_autoCooling = -1;
	static int last_manCooling = -1;
	static int last_manFiltering = -1;
	static int last_filterFanSetSpeed = -1;
	static int last_coolingFanSetSpeed = -1;
	static uint8_t last_lightIndex = -1;
	static uint8_t last_brightness = -1;
	static uint8_t last_ledBar = -1;
	static long last_FanBSpeed = -1;
	static long last_FanASpeed = -1;
	static long last_LWeight = 9999;
	static long last_RWeight = 9999;
	static int last_fanSpeed = 0;
	static uint32_t calsequenceMillis = 0;
	static long leftWeight = NOT_FOUND;
	static long rightWeight = NOT_FOUND;
	static uint8_t lastFilamentPage = -1;
	static uint8_t red;
	static uint8_t green;
	static uint8_t blue;
	static uint8_t last_red = -1;
	static uint8_t last_green = -1;
	static uint8_t last_blue = -1;
	static bool lastServoTextState = (!ms.autoServo && ms.fanServoState == g_tune.servoOpenAngle);
//	static bool alarm = 0;
//	static bool refreshed_alarm = 0;

	red = ms.red * ms.LedBrightness;
	green = ms.green * ms.LedBrightness;
	blue = ms.blue * ms.LedBrightness;

	if (ms.useLedStrip) {
		// Only turn red if alarm is active AND warm-up period has elapsed
		if (ms.alarm && alarm && !refreshed_alarm && 
		    (HAL_GetTick() - smokeBeginTimestamp >= g_tune.smokeBeginTime)) {
			/* D-10: show() pushes the WHOLE strip, so calling it inside the
			 * per-pixel loop sent 100 full DMA frames (~300 ms of blocked main
			 * loop) instead of one (~3 ms). Fill first, then send once. */
			for (int i = 0; i < LED_COUNT; ++i) {
					ledStrip.setPixelColor(i, 255, 0, 0);   // set to red for alarm (R,B,G)
			}
			ledStrip.show();
			refreshed_alarm=1;
		}else{
			if (last_red != red || last_green != green || last_blue != blue||refresh_led_flag) {
					for (int i = 0; i < LED_COUNT; ++i) {
						ledStrip.setPixelColor(i, red, blue, green);   // First LED red
					}
					ledStrip.show();
					last_red = red;
					last_green = green;
					last_blue = blue;
					refresh_led_flag = 0;
						}
			refreshed_alarm = 0;
		}
	}

	if (ms.useLedBar) {
		ms.ledBar ? ssr1.high() : ssr1.low();
	}

	ms.relay ? ssr2.high() : ssr2.low();
	/* D-18: each readRaw() is 64 blocking ADC conversions. Reading both sensors
	 * on every loop pass dominated the loop time for no benefit - the sensors
	 * cannot change meaningfully faster than this. */
	static uint32_t gasSampleTimestamp = 0;
	if (HAL_GetTick() - gasSampleTimestamp >= GAS_SAMPLE_INTERVAL_MS) {
		gasSampleTimestamp = HAL_GetTick();
		if (ms.useMq2) {
			ms.mqSmokeValue = mqSensor.readRaw();
		}
		if (ms.useMems) {
			ms.memsSmokeValue = memsSensor.readRaw();
		}
	}
	ms.filamentPageMax = ((ms.filamentIndexMax + 4) / 4) - 1;
	// Bounds check to prevent array overflow
	if (ms.leftFilament >= 0 && ms.leftFilament < NUM_OF_FILAMENTS) {
		ms.leftSpoolWeight = filamentWeight[ms.leftFilament];
	} else {
		ms.leftSpoolWeight = NOT_FOUND;
	}
	if (ms.rightFilament >= 0 && ms.rightFilament < NUM_OF_FILAMENTS) {
		ms.rightSpoolWeight = filamentWeight[ms.rightFilament];
	} else {
		ms.rightSpoolWeight = NOT_FOUND;
	}
	// Warm-up period: ignore sensor readings for g_tune.smokeBeginTime after power-on
	// Sensor readings are unreliable during the first 60 seconds
	if (HAL_GetTick() - smokeBeginTimestamp >= g_tune.smokeBeginTime) {
		// Warm-up period has elapsed, now check sensors
		if ((ms.mqSmokeValue > g_tune.lpgThreshold || ms.memsSmokeValue > g_tune.smokeThreshold || opticalSmokeActive())) {
			alarm = 1;
			if (!ms.mute) {
				buzzer.high();
			} else {
				buzzer.low();
			}
		} else {
			buzzer.low();
			alarm = 0;
		}
	} else {
		// Warm-up period: explicitly disable alarm regardless of sensor readings
		buzzer.low();
		alarm = 0;
	}
	/* D-15: median of the last three readings, taken across successive loop
	 * passes rather than back-to-back. The HX711 runs at ~10 SPS, so three
	 * consecutive reads would block ~200 ms per cell per pass; sampling once
	 * per pass and keeping a 3-deep window gives the same spike rejection for
	 * free. An error reading restarts the window so it cannot be averaged in. */
	if (ms.useLoadCell) {
		static long    lHist[3], rHist[3];
		static uint8_t lIdx = 0, rIdx = 0, lCount = 0, rCount = 0;

		if (loadCellLeft.isReady()) {
			long raw = loadCellLeft.getWeight();
			if (raw == NOT_FOUND) {
				leftWeight = NOT_FOUND;
				lCount = 0;
			} else {
				lHist[lIdx] = raw;
				lIdx = (uint8_t)((lIdx + 1) % 3);
				if (lCount < 3) lCount++;
				leftWeight = (lCount == 3)
				           ? medianOf3(lHist[0], lHist[1], lHist[2])
				           : raw;
			}
		}
		if (loadCellRight.isReady()) {
			long raw = loadCellRight.getWeight();
			if (raw == NOT_FOUND) {
				rightWeight = NOT_FOUND;
				rCount = 0;
			} else {
				rHist[rIdx] = raw;
				rIdx = (uint8_t)((rIdx + 1) % 3);
				if (rCount < 3) rCount++;
				rightWeight = (rCount == 3)
				            ? medianOf3(rHist[0], rHist[1], rHist[2])
				            : raw;
			}
		}
	}

	if (!tjc.dimmed
			&& HAL_GetTick() - standbyTimestamp
					>= (uint32_t) (ms.standbyTime * 60000)) {
		EEPROM_SaveConfig(&ms);
		tjc.off();
	}
	
	// Safety feature: Automatic flap opening if temperature exceeds g_tune.maxTemp
	// This overrides all other servo commands
	applyServoControl();

	if (ms.autoCooling) {
		// Linearly map temperature (10°C to 40°C) to fan speed (0 to 100%)
		float tempMin = g_tune.autoCoolTempMin;
		float tempMax = g_tune.autoCoolTempMax;
		float fanMinSpeed = 0.0;
		float fanMaxSpeed = 100.0;

		// Clamp the temperature within the range
		float temp = constrain(ms.temperature, tempMin, tempMax);

		// Linear mapping
		float fanSpeed = fanMinSpeed
				+ (temp - tempMin) * (fanMaxSpeed - fanMinSpeed)
						/ (tempMax - tempMin);

		coolingFan.setSpeed((uint8_t) fanSpeed);

	} else if (ms.manCooling) {
		coolingFan.setSpeed(ms.coolingFanSetSpeed);
	} else {
		coolingFan.setSpeed(0);
	}
	if (ms.autoFiltering) {
		const float fanMinSpeed = 20.0f;
		const float fanMaxSpeed = 100.0f;

		float autoSpeed = 0.0f;
		bool aboveThreshold = false;

		// PM2.5 (stored in ms.pm2)
		if (ms.pm2 != NOT_FOUND
				&& ms.pm2 >= g_tune.autoFilterImpMin) {
			float pm2Constrained = constrain(ms.pm2, g_tune.autoFilterImpMin,
					g_tune.autoFilterImpMax);
			float mapped = fanMinSpeed
					+ (pm2Constrained - g_tune.autoFilterImpMin)
							* (fanMaxSpeed - fanMinSpeed)
							/ (g_tune.autoFilterImpMax - g_tune.autoFilterImpMin);
			autoSpeed = mapped;
			aboveThreshold = true;
		}

		// PM0.3
		if (ms.pm03 != NOT_FOUND
				&& ms.pm03 >= g_tune.autoFilterP03Min) {
			float pm03Constrained = constrain(ms.pm03, g_tune.autoFilterP03Min,
					g_tune.autoFilterP03Max);
			float mapped = fanMinSpeed
					+ (pm03Constrained - g_tune.autoFilterP03Min)
							* (fanMaxSpeed - fanMinSpeed)
							/ (g_tune.autoFilterP03Max - g_tune.autoFilterP03Min);
			autoSpeed = mapped > autoSpeed ? mapped : autoSpeed;
			aboveThreshold = true;
		}

		if (!aboveThreshold) {
			filterFan.setSpeed(0);
			filterFanDisplaySpeed = 0;
		} else {
			filterFan.setSpeed((int) autoSpeed);
			filterFanDisplaySpeed = autoSpeed;
		}
	} else if (ms.manFiltering) {
		filterFan.setSpeed(ms.filterFanSetSpeed);
		filterFanDisplaySpeed = ms.filterFanSetSpeed;

	} else {
		filterFan.setSpeed(0);
		filterFanDisplaySpeed = 0;

	}
	/* The extra fan is on/off from the UI; its speed comes from the tuning
	 * block. Until D-06 it was asking for 100% and getting ~15%. */
	fanReg.setSpeed(ms.isFanRegOn ? g_tune.extraFanSpeed : 0);

	/* ── Fire safety: smoke detected -> every fan off ───────────────────────
	 *
	 * Moving air feeds oxygen to a fire and drives smoke through the filter and
	 * out into the room, so all three fans are cut while smoke or gas is
	 * present. Deliberately applied AFTER the auto/manual branches above so no
	 * control path can bypass it.
	 *
	 * Gated on `alarm` (the raw detection), NOT on ms.mute or ms.alarm - those
	 * only silence the buzzer and the red LEDs respectively. Muting an alarm
	 * must never re-enable the fans.
	 *
	 * filterFanDisplaySpeed is zeroed too, so the homescreen reads 0% and the
	 * usage counter stops accruing while the fans are stopped. */
	if (alarm && g_tune.smokeFireResponse) {
		coolingFan.setSpeed(0);
		filterFan.setSpeed(0);
		fanReg.setSpeed(0);
		filterFanDisplaySpeed = 0;
	} else if (g_ventOpenOvertemp) {
		/* Overtemp and NO smoke: run every fan flat out to pull the heat out,
		 * alongside the flap being held open by applyServoControl(). Overrides
		 * whatever auto/manual asked for, including the extra fan's on/off
		 * switch - this is a thermal emergency, not a comfort setting.
		 * The smoke branch above takes precedence, so a fire never gets this. */
		coolingFan.setSpeed(100);
		filterFan.setSpeed(100);
		fanReg.setSpeed(100);
		filterFanDisplaySpeed = 100;
	}

	if (ms.currentPage == HOMESCREEN) {
		if (HAL_GetTick() - BME_Timestamp >= g_tune.bmeSampleInterval && ms.useBme) {
				readBme();
				BME_Timestamp = HAL_GetTick();
			}

		if ((last_pm2 != ms.pm2)
				|| (last_temperature != ms.temperature || refreshFlag)
				|| (last_memsSmokeValue != ms.memsSmokeValue)
				|| (last_LWeight != leftWeight) || (last_RWeight != rightWeight)
				|| (last_fanSpeed != filterFanDisplaySpeed)  /* actual, not the manual setpoint */

				) {
			last_pm2 = ms.pm2;
			last_temperature = ms.temperature;
			last_memsSmokeValue = ms.memsSmokeValue;
			last_LWeight = leftWeight;
			last_RWeight = rightWeight;
			last_fanSpeed = filterFanDisplaySpeed;
			/* Net filament weight = measured - spool tare.
			 *
			 * Not clamped to zero: an empty holder reads as the negative tare
			 * of the selected spool (e.g. -202 for a 202 g spool), which shows
			 * at a glance that the holder is empty and which spool is set. The
			 * spool-holders page renders it the same way.
			 *
			 * NOT_FOUND is passed straight through so a load-cell error stays
			 * distinguishable from a real reading (it used to be clamped to 0,
			 * which looked exactly like an empty holder). */
			long lw = (leftWeight  == NOT_FOUND) ? (long)NOT_FOUND
			                                     : (leftWeight  - ms.leftSpoolWeight);
			long rw = (rightWeight == NOT_FOUND) ? (long)NOT_FOUND
			                                     : (rightWeight - ms.rightSpoolWeight);
			
			if(!ms.useLoadCell){
				lw =NOT_FOUND;
				rw =NOT_FOUND;
			}
			char buff[20];
			ADD_UNIT("t0", ms.memsSmokeValue, "", buff);
			ADD_UNIT("t1", ms.temperature, "C", buff);
			ADD_UNIT("t2", ms.pm2, "", buff);
			ADD_UNIT("t3", (lw), "g", buff);
			ADD_UNIT("t4", filterFanDisplaySpeed, "%", buff);
			ADD_UNIT("t5", (rw), "g", buff);
			refreshFlag = false;
		}

	}

	if (ms.currentPage == SENSORS) {
		if (HAL_GetTick() - BME_Timestamp >= g_tune.bmeSampleInterval && ms.useBme) {
				readBme();
				BME_Timestamp = HAL_GetTick();
			}

		if ((last_pm2 != ms.pm2) || forceUpdate) {
			last_pm2 = ms.pm2;
			tjc.setVal("n4", ms.pm2);
		}
//#TODO: Change pm03 back to pm10
		if ((last_pm10 != ms.pm03) || forceUpdate) {
			last_pm10 = ms.pm03;
			tjc.setVal("n5", ms.pm03);
		}

		if ((last_pressure != ms.pressure) || forceUpdate) {
			last_pressure = ms.pressure;
			tjc.setVal("n2", ms.pressure);

		}
		if ((last_voc != ms.voc) || forceUpdate) {
			last_voc = ms.voc;
			tjc.setVal("n6", ms.voc);
		}
		if ((last_temperature != ms.temperature) || forceUpdate) {
			last_temperature = ms.temperature;
			tjc.setVal("n0", ms.temperature);
		}
		if ((last_HumidityBME != ms.HumidityBME) || forceUpdate) {
			last_HumidityBME = ms.HumidityBME;
			tjc.setVal("n1", ms.HumidityBME);
		}

		if ((last_mqSmokeValue != ms.mqSmokeValue) || forceUpdate) {
			last_mqSmokeValue = ms.mqSmokeValue;
			tjc.setVal("n3", ms.mqSmokeValue);

			if ((last_memsSmokeValue != ms.memsSmokeValue) || forceUpdate) {
				last_memsSmokeValue = ms.memsSmokeValue;
				tjc.setVal("n7", ms.memsSmokeValue);
			}
			if (lastSmoke != opticalSmokeActive() || forceUpdate) {
				opticalSmokeActive() ?
						tjc.setText("t0", "Smoke") : tjc.setText("t0", "Clear");
				lastSmoke = opticalSmokeActive();
			}
		}

		if (lastLogging != ms.logging || forceUpdate) {
			ms.logging ?
					tjc.setPic("b3", INTERVAL_ON) :
					tjc.setPic("b3", INTERVAL_OFF);
			lastLogging = ms.logging;
		}
		if (lastInterval != ms.interval || forceUpdate) {
			if (ms.interval == 5) {
				tjc.setPic("b1", MIN_5_LOG);
			} else if (ms.interval == 10) {
				tjc.setPic("b1", MIN_10_LOG);
			} else {
				tjc.setPic("b1", MIN_60_LOG);
			}
			lastInterval = ms.interval;
		}
		forceUpdate = false;

	}

	if (ms.currentPage == FEATURES) {

		if (refreshFlag) {
			ms.usePms ?
					tjc.setPic("p1", SELECTED_PMS_IMAGE_ID) :
					tjc.setPic("p1", PMS_IMAGE_ID);

			ms.useBme ?
					tjc.setPic("p2", SELECTED_BME_IMAGE_ID) :
					tjc.setPic("p2", BME_IMAGE_ID);
			ms.useOptical ?
					tjc.setPic("p3", SELECTED_OPTICAL_SMOKE_SENSOR_IMAGE_ID) :
					tjc.setPic("p3", OPTICAL_SMOKE_SENSOR_IMAGE_ID);
			ms.useMq2 ?
					tjc.setPic("p4", SELECTED_MQ2_IMAGE_ID) :
					tjc.setPic("p4", MQ2_IMAGE_ID);
			ms.useLoadCell ?
					tjc.setPic("p5", SELECTED_LOAD_CELL_IMAGE_ID) :
					tjc.setPic("p5", LOAD_CELL_IMAGE_ID);
			ms.useFilterFan ?
					tjc.setPic("p6", SELECTED_FILTER_FAN_IMAGE_ID) :
					tjc.setPic("p6", FILTER_FAN_IMAGE_ID);
			ms.useCoolingFan ?
					tjc.setPic("p7", SELECTED_COOLING_FAN_IMAGE_ID) :
					tjc.setPic("p7", COOLING_FAN_IMAGE_ID);
			ms.useLedBar ?
					tjc.setPic("p8", SELECTED_RGB_LED_IMAGE_ID) :
					tjc.setPic("p8", RGB_LED_IMAGE_ID);

			ms.useLedStrip ?
					tjc.setPic("p9", SELECTED_LED_STRIP_IMAGE_ID) :
					tjc.setPic("p9", LED_STRIP_IMAGE_ID);
			refreshFlag = 0;
		}
	}
	if (ms.currentPage == SYS_CHECK) {
		uint32_t timestamp = HAL_GetTick();
		uint32_t timeout = 3000;
		bool success = false;
		bool success2 = false;

		if (runOnce) {
			if (ms.useCoolingFan) {
				SHOW_CONT_BTN
				;
				coolingFanState = 0;
				coolingFan.setSpeed(100);
			} else {

			}
			runOnce = false;
		}

		if (!checkedPms) {
			if (ms.usePms) {
				tjc.setPic("p1", RUNNING_IMAGE_ID);

				while (HAL_GetTick() - timestamp <= timeout) {
					success = isValidPmsData(pms_data, PMS_FRAME_LENGTH);
					if (success) {
						tjc.setPic("p1", SUCCESS_IMAGE_ID);
						break;
					}
				}
				if (!success) {
					tjc.setPic("p1", FAIL_IMAGE_ID);
				}
			} else {
				HAL_Delay(200);
				tjc.setPic("p1", NULL_IMAGE_ID);

			}
			checkedPms = true;
		}

		if (!checkedBme) {
			if (ms.useBme) {
				if (readBme()) {
					tjc.setPic("p2", SUCCESS_IMAGE_ID);
				} else {
					tjc.setPic("p2", FAIL_IMAGE_ID);
				}
			} else {
				tjc.setPic("p2", NULL_IMAGE_ID);
			}
			checkedBme = true;
			timestamp = HAL_GetTick();
			success = false;
		}

		if (!checkedOSS) {
			if (ms.useOptical) {
				while (HAL_GetTick() - timestamp <= timeout) {
					success = !smokeSensor.isSmokeDetected();
					if (success) {
						tjc.setPic("p3", SUCCESS_IMAGE_ID);
						break;
					}
				}
				if (!success) {
					tjc.setPic("p3", FAIL_IMAGE_ID);
				}
			} else {
				tjc.setPic("p3", NULL_IMAGE_ID);

			}
			checkedOSS = true;
			timestamp = HAL_GetTick();
			success = false;
		}

		if (!checkedMq2) {
			if (ms.useMq2) {
				while (HAL_GetTick() - timestamp <= timeout) {
					success = !mqSensor.isSmokeDetected();
					if (success) {
						tjc.setPic("p4", SUCCESS_IMAGE_ID);
						break;
					}
				}
				if (!success) {
					tjc.setPic("p4", FAIL_IMAGE_ID);
				}
			} else {
				tjc.setPic("p4", NULL_IMAGE_ID);

			}
			checkedMq2 = true;
			timestamp = HAL_GetTick();
			success = false;
		}

		if (!checkedLC) {
			if (ms.useLoadCell) {
				while (HAL_GetTick() - timestamp <= timeout) {
					success = loadCellLeft.isReady();
					success2 = loadCellRight.isReady();
					if (success && success2) {
						tjc.setPic("p5", SUCCESS_IMAGE_ID);
						break;
					}
					HAL_Delay(10);
				}
				if (!(success && success2)) {
					tjc.setPic("p5", FAIL_IMAGE_ID);
				}
			} else {
				tjc.setPic("p5", NULL_IMAGE_ID);

			}
			checkedLC = true;
		}

		if (!checkedFFan) {
			if (ms.useFilterFan) {
				int _thresh = 100;
				filterFan.setSpeed(0);
				HAL_Delay(3000);
				int speedA = ms.filterFanASpeed;
				int speedB = ms.filterFanBSpeed;
				filterFan.setSpeed(75);
				HAL_Delay(5000);
				int speedA1 = ms.filterFanASpeed;
				int speedB1 = ms.filterFanBSpeed;
				filterFan.setSpeed(0);
				if ((speedA1 - speedA > _thresh)
						|| (speedB1 - speedB > _thresh)) {
					tjc.setPic("p6", SUCCESS_IMAGE_ID);
				} else {
					tjc.setPic("p6", FAIL_IMAGE_ID);

				}
			} else {
				tjc.setPic("p6", NULL_IMAGE_ID);

			}
			checkedFFan = true;
			HIDE_CONT_BTN
			;

		}

//		if (coolingFanState == 0) { // cooling fan  flag has 4 states 0 -> not interacted with button, 1 -> not spinning, 2 -> spinning
		if (ms.useCoolingFan) {
			if (coolingFanState == 0) {
				coolingFan.setSpeed(100);

			} else if (coolingFanState == 2) {
				tjc.setPic("p7", FAIL_IMAGE_ID);
				coolingFan.setSpeed(0);
				checkedCFan = true;
			} else if (coolingFanState == 1) {
				tjc.setPic("p7", SUCCESS_IMAGE_ID);
				coolingFan.setSpeed(0);
				checkedCFan = true;
			}
		} else {
			tjc.setPic("p7", NULL_IMAGE_ID);
//				checkedCFan = true;

		}
//		}

	}

// #TODO  Change from loop to sending only the selected spool index

	if (ms.currentPage == SPOOLHOLDERS) {

		int t0filamentIndex = 0; // filament name
		int t1filamentIndex = 0; // filament name
		int t2filamentIndex = 0; // filament name
		int t3filamentIndex = 0; // filament name

		bool lChecked = (ms.leftFilament <= ((ms.filamentPage * 4) + 3))
				&& (ms.leftFilament >= ((ms.filamentPage * 4)));
		bool rChecked = (ms.rightFilament <= ((ms.filamentPage * 4) + 3))
				&& (ms.rightFilament >= ((ms.filamentPage * 4)));

		if (lastFilamentPage != ms.filamentPage || forceUpdate) {

			t0filamentIndex = (4 * ms.filamentPage);
			t1filamentIndex = (4 * ms.filamentPage) + 1;
			t2filamentIndex = (4 * ms.filamentPage) + 2;
			t3filamentIndex = (4 * ms.filamentPage) + 3;

			// Bounds check to prevent array overflow
			if (t0filamentIndex >= 0 && t0filamentIndex < NUM_OF_FILAMENTS) {
				tjc.setText("t0", filamentBrand[t0filamentIndex]);
			} else {
				tjc.setText("t0", "");
			}
			if (t1filamentIndex >= 0 && t1filamentIndex < NUM_OF_FILAMENTS) {
				tjc.setText("t1", filamentBrand[t1filamentIndex]);
			} else {
				tjc.setText("t1", "");
			}
			if (t2filamentIndex >= 0 && t2filamentIndex < NUM_OF_FILAMENTS) {
				tjc.setText("t2", filamentBrand[t2filamentIndex]);
			} else {
				tjc.setText("t2", "");
			}
			if (t3filamentIndex >= 0 && t3filamentIndex < NUM_OF_FILAMENTS) {
				tjc.setText("t3", filamentBrand[t3filamentIndex]);
			} else {
				tjc.setText("t3", "");
			}
			tjc.setVal("pg", ms.filamentPage);
			lastFilamentPage = ms.filamentPage;

			for (int i = 0; i < numOfButtons; ++i) {
				tjc.setCheckboxState(leftIDNums[i],
						(i == ms.leftSpoolIndex && lChecked) ? 1 : 0);
				tjc.setCheckboxState(rightIDNums[i],
						(i == ms.rightSpoolIndex && rChecked) ? 1 : 0);
			}
		}
		if ((last_LWeight != leftWeight)|| forceUpdate) {
			/* No clamp to zero here: an empty holder should read as the
			 * negative tare of the selected spool (e.g. -202 for a 202 g
			 * spool), which tells you at a glance that the holder is empty
			 * and which spool is configured. */
			long lw = (leftWeight == NOT_FOUND) ? (long)NOT_FOUND
			                                    : (leftWeight - ms.leftSpoolWeight);
			lw = ms.useLoadCell?lw:NOT_FOUND;
			tjc.setVal("n0", (lw));
			last_LWeight = leftWeight;
		}
		if ((last_RWeight != rightWeight)|| forceUpdate) {
			long rw = (rightWeight == NOT_FOUND) ? (long)NOT_FOUND
			                                     : (rightWeight - ms.rightSpoolWeight);
			rw = ms.useLoadCell?rw:NOT_FOUND;
			tjc.setVal("n1", (rw));
			last_RWeight = rightWeight;
		}
		forceUpdate = false;

	}
	if (ms.currentPage == SETTINGS) {
		int usage_hour = ms.usage_minutes/60;

static int last_usage_hour = 0;
		if (refreshFlag) {
			tjc.setVal("sbt", ms.standbyTime);
			tjc.setVal("br", ms.brightness);

			/* Threshold is in hours (same unit the page displays), usage is
			 * tracked in minutes - convert before comparing. */
			int hours_pic = (usage_hour >= (int32_t)g_tune.sensorUsageWarnHours)
			                ? SENSOR_UNSAFE_PIC : SENSOR_SAFE_PIC;
			tjc.setPic("usage_bg", hours_pic);
			int mute_pic  = ms.mute?MUTE:UNMUTE;
			tjc.setPic("mute_bt", mute_pic);
			int alarm_pic  = ms.alarm?ALARM:NO_ALARM;
			tjc.setPic("alarm_bt", alarm_pic);
			tjc.setVal("n2", usage_hour);
			refreshFlag = 0;
		}
if(last_usage_hour != usage_hour){
	tjc.setVal("n2", usage_hour);
	last_usage_hour = usage_hour;
}

	}
	if (ms.currentPage == OUTPUT_CTRL) {

		if (refreshFlag) {
			tjc.setVal("bt0", ms.ledBar);
			tjc.setVal("bt1", ms.relay);
			tjc.setVal("bt2", ms.autoServo);
			tjc.setVal("bt3", ms.servoPower);
			refreshFlag = 0;
		}
		if (lastServoTextState != (!ms.autoServo && ms.fanServoState == g_tune.servoOpenAngle)) {
			if ((!ms.autoServo && ms.fanServoState == g_tune.servoOpenAngle)) {
				tjc.setText("t0", "ON");
			}else{
				tjc.setText("t0", "OFF");

			}
			lastServoTextState = (!ms.autoServo && ms.fanServoState == g_tune.servoOpenAngle);
		}
	}
	if (ms.currentPage == FANCTRL) {
		static int last_sw_state = -1;

		if (last_autoCooling != ms.autoCooling || forceUpdate) {
			tjc.setVal("va1", ms.autoCooling);
			last_autoCooling = ms.autoCooling;
		}

		if (last_manCooling != ms.manCooling || forceUpdate) {
			tjc.setVal("mc", ms.manCooling);
			last_manCooling = ms.manCooling;
		}
		if (last_manFiltering != ms.manFiltering || forceUpdate) {
			tjc.setVal("mf", ms.manFiltering);
			last_manFiltering = ms.manFiltering;
		}

		if (last_autoFiltering != ms.autoFiltering || forceUpdate) {
			tjc.setVal("fAuto", ms.autoFiltering);
			last_autoFiltering = ms.autoFiltering;
		}
		if (last_filterFanSetSpeed != ms.filterFanSetSpeed || forceUpdate) {
			tjc.setVal("h0", ms.filterFanSetSpeed);
			tjc.setVal("n0", ms.filterFanSetSpeed);
			last_filterFanSetSpeed = ms.filterFanSetSpeed;
		}
		if (last_coolingFanSetSpeed != ms.coolingFanSetSpeed || forceUpdate) {
			tjc.setVal("h2", ms.coolingFanSetSpeed);
			tjc.setVal("n1", ms.coolingFanSetSpeed);
			last_coolingFanSetSpeed = ms.coolingFanSetSpeed;
		}
		if (last_isFanRegOn != ms.isFanRegOn || forceUpdate) {
			tjc.setVal("sw1", ms.isFanRegOn);
			last_isFanRegOn = ms.isFanRegOn;
		}
		if (last_sw_state != (int) sw || forceUpdate) {
			tjc.setVal("bt1", sw);
			last_sw_state = (int) sw;
		}

		if (last_FanASpeed != ms.filterFanASpeed || forceUpdate) {
			tjc.setVal("n2", ms.filterFanASpeed);
			last_FanASpeed = ms.filterFanASpeed;
		}
		if (last_FanBSpeed != ms.filterFanBSpeed || forceUpdate) {
			tjc.setVal("n3", ms.filterFanBSpeed);
			last_FanBSpeed = ms.filterFanBSpeed;
		}
		 forceUpdate =false;
	}
	if (ms.currentPage == LIGHTING) {
		if (last_lightIndex != ms.lightIndex || forceUpdate) {
			tjc.setVal("selection", ms.lightIndex);
			last_lightIndex = ms.lightIndex;
		}

		if ((int) (ms.LedBrightness * 100) != last_brightness || forceUpdate) {

			tjc.setVal("h0", (int) (ms.LedBrightness * 100));
			last_brightness = (int) (ms.LedBrightness * 100);
		}

		if (last_ledBar != ms.ledBar || forceUpdate) {
			tjc.setVal("led", ms.ledBar);
			last_ledBar = ms.ledBar;
		}
		 forceUpdate=false;
		calsequenceMillis = HAL_GetTick();

	}

	if (ms.currentPage == CAL_SEQUENCE1) {
		if (HAL_GetTick() - calsequenceMillis >= 900 && countdown >= 0) {
			tjc.setVal("n0", countdown);
			countdown--;
			if (countdown == 0) {
				tjc.setAph("p2", 127);
			}
			calsequenceMillis = HAL_GetTick();
			HAL_Delay(90);
		}

	}
	if (ms.currentPage == CAL_SEQUENCE3) {
		if (HAL_GetTick() - calsequenceMillis >= 900 && countdown >= 0) {
			tjc.setVal("n0", countdown);
			countdown--;
			if (countdown == 0) {
				tjc.setAph("p2", 127);
			}
			calsequenceMillis = HAL_GetTick();
			HAL_Delay(90);
		}

	}

	if (ms.currentPage == CAL_SEQUENCE4) {

		tjc.setText("t0", ms.leftCalFactor);
		tjc.setText("t1", ms.rightCalFactor);
	}

	if (ms.currentPage == CALIBRATE1) {
		if (HAL_GetTick() - calsequenceMillis >= 900 && countdown >= 0) {
			tjc.setVal("n0", countdown);
			countdown--;
			if (countdown == 0) {
				tjc.setAph("p2", 127);
			}
			calsequenceMillis = HAL_GetTick();
			HAL_Delay(90);
		}

	}
	if (ms.currentPage == CALIBRATE2) {
		if (HAL_GetTick() - calsequenceMillis >= 900 && countdown >= 0) {
			tjc.setVal("n0", countdown);
			countdown--;
			if (countdown == 0) {
				tjc.setAph("p2", 127);
			}
			calsequenceMillis = HAL_GetTick();
			HAL_Delay(90);
		}

	}

	if (ms.currentPage == CALIBRATE4) {
		// Bounds check to prevent array overflow
		if (ms.filamentIndexMax >= 0 && ms.filamentIndexMax < NUM_OF_FILAMENTS) {
			tjc.setText("t0", filamentBrand[ms.filamentIndexMax]);
			char weightStr[16];
			snprintf(weightStr, sizeof(weightStr), "%d", filamentWeight[ms.filamentIndexMax]);
			tjc.setText("t1", weightStr);
		} else {
			tjc.setText("t0", "");
			tjc.setText("t1", "");
		}
		HAL_Delay(90);
	}
}

//------------------------------------BEGIN CALIBRATION PAGE------------------------------------
void handleLndScreen2(ScreenEvent event) {
	switch (event.objectID) {

	case 2:
		tjc.setPage(CHOOSE_CONFIG, ms);
		break;

	default:
		break;
	}
}
//------------------------------------CONFIG PAGE------------------------------------

void handleConfig(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		ms.features = 0; //REgular
		ms.useBme = true;
		ms.useCoolingFan = true;
		ms.useFilterFan = true;
		ms.useLedBar = true;
		ms.useLedStrip = true;
		ms.useLoadCell = true;
		ms.useMems = true;
		ms.useMq2 = true;
		ms.useOptical = false;
		ms.usePms = false;
		break;
	case 3:
		ms.features = 1; //pro
		ms.useBme = true;
		ms.useCoolingFan = true;
		ms.useFilterFan = true;
		ms.useLedBar = true;
		ms.useLedStrip = true;
		ms.useLoadCell = true;
		ms.useMems = true;
		ms.useMq2 = true;
		ms.useOptical = true;
		ms.usePms = true;
		break;
	case 4:
		ms.features = 2; //Custom
		break;
	case 5:
		tjc.setPage(LNDSCREEN2, ms); //back
		break;
	default:
		break;
	}
	ms.features == 2 ? tjc.setPage(FEATURES, ms) : tjc.setPage(SYS_CHECK, ms);

}
//------------------------------------  CUSTOM FEATURES PAGE------------------------------------
void handleCustomFeatures(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		ms.usePms = !ms.usePms;
		break;
	case 3:
		ms.useBme = !ms.useBme;
		break;
	case 4:
		ms.useOptical = !ms.useOptical;
		break;
	case 5:
		ms.useMq2 = !ms.useMq2;
		break;
	case 6:
		ms.useLoadCell = !ms.useLoadCell;
		break;
	case 7:
		ms.useFilterFan = !ms.useFilterFan;
		break;
	case 8:
		ms.useCoolingFan = !ms.useCoolingFan;
		break;
	case 9:
		ms.useLedBar = !ms.useLedBar;
		break;
	case 10:
		ms.useLedStrip = !ms.useLedStrip;
		break;
	case 11:
		tjc.setPage(SYS_CHECK, ms);
		break;
	case 12:
		tjc.setPage(CHOOSE_CONFIG, ms);
		checkedPms = false;
		checkedBme = false;
		checkedOSS = false;
		checkedMq2 = false;
		checkedLC = false;
		checkedFFan = false;
		checkedCFan = false;
		checkedRGB = false;
		checkedLED = false;
		coolingFanState = 0;
		runOnce = true;
		break;
	default:
		break;
	}
}

//------------------------------------  SYSTEM CHECK PAGE------------------------------------
void handleSystemCheck(ScreenEvent event) {
	switch (event.objectID) {

	case 9:
		tjc.setPage(FEATURES, ms);
		checkedPms = false;
		checkedBme = false;
		checkedOSS = false;
		checkedMq2 = false;
		checkedLC = false;
		checkedFFan = false;
		checkedCFan = false;
		checkedRGB = false;
		checkedLED = false;
		coolingFanState = 0;
		runOnce = true;

		break;
	case 13:

		ms.useLoadCell ?
				tjc.setPage(SELECTOR, ms) : tjc.setPage(CAL_FINISH_PAGE, ms);
		break;
	case 12: // SPINNING
		SHOW_CONT_BTN
		;
		coolingFanState = 2;
		break;
	case 11:
		coolingFanState = 1; // NOT SPINNING
		SHOW_CONT_BTN
		;
//		tjc.setPage(SELECTOR, ms);
		break;
	default:
		break;
	}
}
//------------------------------------  SELECTOR PAGE------------------------------------
void handleSelector(ScreenEvent event) {
	switch (event.objectID) {

	case 4:
		tjc.setPage(CHOOSE_CONFIG, ms);
		checkedPms = false;
		checkedBme = false;
		checkedOSS = false;
		checkedMq2 = false;
		checkedLC = false;
		checkedFFan = false;
		checkedCFan = false;
		checkedRGB = false;
		checkedLED = false;
		coolingFanState = 0;
		runOnce = true;

		break;
	case 2:
		/* Restore factory seeds from the tuning block (data.clu). The offsets
		 * are restored too - previously only the scale was, which left a
		 * half-reset calibration with the old zero point still in place. */
		ms.leftCalFactor  = g_tune.defaultLeftCal;
		ms.rightCalFactor = g_tune.defaultRightCal;
		ms.leftOffset     = g_tune.defaultLeftOffset;
		ms.rightOffset    = g_tune.defaultRightOffset;
		loadCellLeft.setScale(ms.leftCalFactor);
		loadCellRight.setScale(ms.rightCalFactor);
		loadCellLeft.setOffset(ms.leftOffset);
		loadCellRight.setOffset(ms.rightOffset);
		tjc.setPage(CAL_FINISH_PAGE, ms);
		break;
	case 3: // SPINNING
		tjc.setPage(CAL_SEQUENCE1, ms);

		break;

	default:
		break;
	}
}
//------------------------------------  CALIBRATION SEQUENCE 1 PAGE------------------------------------
void handleCalSequence1(ScreenEvent event) {
	switch (event.objectID) {
	case 4:
		if (ms.calibrationComplete) {
					tjc.setPage(SPOOLHOLDERS, ms);
				} else {
					tjc.setPage(SELECTOR, ms);
				}
		//		countdown=5;// reset countdown timer
		break;
	case 5:
		loadCellLeft.tare();
		loadCellRight.tare();
		tjc.setPage(CAL_SEQUENCE2, ms);
		break;
	}
	countdown = g_tune.countdownTime; // reset countdown timer

}
//------------------------------------  CALIBRATION SEQUENCE 2 PAGE------------------------------------

void handleCalSequence2(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		/* D-16: the weights live in dataBuffer[0..3], which is only populated
		 * for as many bytes as the frame actually carried. Without this guard a
		 * short frame silently calibrated against stale/uninitialised bytes. */
		if (event.commandCode >= 4) {
			leftCalWeight = (dataBuffer[1] * 256) + dataBuffer[0]; //convert data from the Tjc screen to integers
			rightCalWeight = (dataBuffer[3] * 256) + dataBuffer[2];
			tjc.setPage(CAL_SEQUENCE3, ms);
		}
		break;
	case 3:
		tjc.setPage(CAL_SEQUENCE1, ms);
		break;
	}
	countdown = g_tune.countdownTime; // reset countdown timer

}
//------------------------------------  CALIBRATION SEQUENCE 3 PAGE------------------------------------

void handleCalSequence3(ScreenEvent event) {
//
	switch (event.objectID) {
	case 3:
		tjc.setPage(CAL_SEQUENCE2, ms);
		break;
	case 6:
		loadCellLeft.calibrate(leftCalWeight);
		loadCellRight.calibrate(rightCalWeight);
		ms.leftCalFactor = loadCellLeft.getScale();
		ms.rightCalFactor = loadCellRight.getScale();
		ms.leftOffset = loadCellLeft.getOffset();
		ms.rightOffset = loadCellRight.getOffset();
		loadCellLeft.setScale(ms.leftCalFactor);
		loadCellRight.setScale(ms.rightCalFactor);
		tjc.setPage(CAL_SEQUENCE4, ms);
		break;
	}
}
//------------------------------------  CALIBRATION SEQUENCE 3 PAGE------------------------------------
void handleCalSequence4(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		tjc.setPage(CAL_SEQUENCE3, ms);
		break;
	case 3:
		tjc.setPage(CAL_FINISH_PAGE, ms);
		break;
	}
	countdown = g_tune.countdownTime; // reset countdown timer

}

//------------------------------------  CALIBRATE  1 PAGE------------------------------------
void handleCalibrate1(ScreenEvent event) {
	switch (event.objectID) {
	case 6:
		tjc.setPage(SPOOLHOLDERS, ms);
		break;
	case 5:
		loadCellLeft.tare();
		loadCellRight.tare();
		tjc.setPage(CALIBRATE2, ms);
		break;
	}
	countdown = g_tune.countdownTime; // reset countdown timer

}
//------------------------------------  CALIBRATE  2 PAGE------------------------------------
void handleCalibrate2(ScreenEvent event) {
	switch (event.objectID) {
	case 5: {
		long int l = loadCellLeft.getWeight();
		long r = loadCellRight.getWeight();
		customSpoolWeight = l > r ? l : r;
		customSpoolWeight -= g_tune.tareWeight;
		customSpoolWeight = customSpoolWeight < 0 ? 0 : customSpoolWeight;
		tjc.setPage(CALIBRATE3, ms);
		break;
	}
	case 6:
		tjc.setPage(CALIBRATE1, ms);
		break;
	}
	countdown = g_tune.countdownTime; // reset countdown timer

}
//------------------------------------  CALIBRATE 3 PAGE------------------------------------
void handleCalibrate3(ScreenEvent event) {
	switch (event.objectID) {

	case 2: {
		/* The name arrives in dataBuffer, which only holds as many bytes as the
		 * frame actually carried. Without this guard a short frame registers a
		 * spool named from stale bytes left by a previous message. */
		if (event.commandCode <= 0) {
			break;
		}
		int nameLen = event.commandCode;
		if (nameLen > LEN_BUFFER - 1) nameLen = LEN_BUFFER - 1;

		// Clamp filamentIndexMax to valid range before increment
		if (ms.filamentIndexMax >= NUM_OF_FILAMENTS) {
			ms.filamentIndexMax = NUM_OF_FILAMENTS - 2;
		}
		// Increment and check bounds before array access
		ms.filamentIndexMax++;
		if (ms.filamentIndexMax < NUM_OF_FILAMENTS) {
			/* Clear first, then copy only the bytes actually received. The old
			 * code memcpy'd a fixed LEN_BUFFER bytes, so a short name kept
			 * stale trailing bytes and a 16-character one was left WITHOUT a
			 * null terminator - which then ran past the entry when rendered
			 * with %s or written out to filament.clu. */
			memset(filamentBrand[ms.filamentIndexMax], 0, LEN_BUFFER);
			memcpy(filamentBrand[ms.filamentIndexMax], (char*) dataBuffer, nameLen);
			filamentWeight[ms.filamentIndexMax] = customSpoolWeight;
		} else {
			// Revert if overflow would occur
			ms.filamentIndexMax = NUM_OF_FILAMENTS - 1;
		}
		tjc.setPage(CALIBRATE4, ms);
		break;
	}
	case 4:
		tjc.setPage(CALIBRATE2, ms);
		break;
	}

}
//------------------------------------  CALIBRATE  4 PAGE------------------------------------
void handleCalibrate4(ScreenEvent event) {
	switch (event.objectID) {

	case 3:
		if (EEPROM_SaveConfig(&ms)
				&& EEPROM_SaveFilamentData(filamentBrand, filamentWeight)) {
			/* A spool was just registered - mirror the library out to the card
			 * so filament.clu always matches the machine. The stored revision
			 * is preserved, so this never looks like a technician edit. */
			(void)Filament_ExportToSD(filamentBrand, filamentWeight,
			                          ms.filamentIndexMax);
			tjc.setPage(CALIBRATION_SAVE_SUCCESS, ms);
		} else {
			tjc.setPage(CALIBRATION_SAVE_FAIL, ms);
		}
		break;
	}

}

//------------------------------------  DONE PAGE------------------------------------
void handleDone(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		// Persist calibration status before saving so it survives power cycles
		ms.calibrationComplete = true;
		if (EEPROM_SaveConfig(&ms)) {
			tjc.setPage(HOMESCREEN, ms);
		} else {
			tjc.setPage(CONFIG_SAVE_FAIL, ms);
		}
		break;

	}
}
void handleConfigSaveFail (ScreenEvent event) {
	switch (event.objectID) {
	case 1:
			tjc.setPage(HOMESCREEN, ms);
		break;

	}
}
void handleCalibrationSaveFail(ScreenEvent event) {
	switch (event.objectID) {
	case 1:
			tjc.setPage(HOMESCREEN, ms);
			break;
		case 2:
			tjc.setPage(CALIBRATE4, ms);
				break;
		}

	}
void handleFormatFail(ScreenEvent event) {
	switch (event.objectID) {
	case 1:
			tjc.setPage(SENSORS, ms);
			break;
		case 2:
			tjc.setPage(HOMESCREEN, ms);
				break;
		}

	}
void handleFormatSuccess(ScreenEvent event) {
	switch (event.objectID) {
	case 1:
			tjc.setPage(SENSORS, ms);
			break;
		case 2:
			tjc.setPage(HOMESCREEN, ms);
				break;
		}

	}
void handleCalibrationSaveSuccess(ScreenEvent event) {
	switch (event.objectID) {
	case 1:
		tjc.setPage(HOMESCREEN, ms);
		break;
	case 2:
		tjc.setPage(CALIBRATE4, ms);
			break;
	}
}
//------------------------------------LIGHTING------------------------------------

void handleLighting(ScreenEvent event) {
	if ((event.objectID >= 7 && event.objectID <= 13) || event.objectID == 3) {
		ms.red = event.valueB;
		ms.green = event.valueC;
		ms.blue = event.valueD;
		ms.LedBrightness = (float) event.valueE / 100;
		ms.lightIndex = event.objectID;

	}

	else if (event.objectID == 16) { //Led Bar Switch
		ms.ledBar = event.valueA;
//		tjc.setButtonState(6, event.valueA);
	} else if (event.objectID == 4) { //Led Brightness slider
		ms.LedBrightness = (float) event.valueE / 100;
//		event.valueE >= 1 ? tjc.setText("t0", "ON") : tjc.setText("t0", "OFF");
	} else if (event.objectID == 6) {
		tjc.setPage(HOMESCREEN, ms);
	}
}

//------------------------------------OUTPUT CONTROL------------------------------------
void handleOutput(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		ms.ledBar = event.valueA;
		tjc.setVal("bt0", event.valueA);
		break;
	case 3:
		ms.relay = event.valueA;
		tjc.setVal("bt1", event.valueA);
		break;
	case 4:
		ms.autoServo = event.valueA;
		tjc.setVal("bt2", ms.autoServo);
		break;
	case 5:
		ms.servoPower = event.valueA > 0;
		tjc.setVal("bt3", ms.servoPower);
		// Manual servo positioning removed: the servo is controlled only by the fan-page
		// OPEN/CLOSE button (plus auto mode / overtemp). servoPower only affects auto mode.
		break;

	case 6:
		tjc.setPage(HOMESCREEN, ms);
		break;
	case 9: // Servo slider DISABLED: the servo is controlled only by the fan-page OPEN/CLOSE button.
		break;

	default:
		break;
	}

}
//------------------------------------SETTINGS CONTROL------------------------------------
void handleSettings(ScreenEvent event) {
	switch (event.objectID) {
	case 2:
		ms.temperatureBaseline = ms.temperature;
		ms.impuritiesBaseline = ms.voc;
		break;
	case 3:
		tjc.setPage(LNDSCREEN2, ms);
		break;
	case 4:
		HAL_NVIC_SystemReset();
		break;
	case 5:
		tjc.setPage(SUPPORT, ms);
		break;
	case 6:
		ms.standbyTime = event.valueE;
		saveFlag = true;
		break;
	case 7:

		ms.brightness = event.valueE;
		if (ms.brightness < g_tune.minBrightness) {
			ms.brightness = g_tune.minBrightness;
		}
		tjc.setDim(ms.brightness);
		saveFlag = true;
		break;
	case 17:
			ms.mute =!ms.mute;
			break;
	case 18:
		ms.alarm=!ms.alarm;
		alarm?refresh_led_flag=1:refresh_led_flag=0;
		break;
	case 15:
		/* Reset machine usage. Clearing the base AND the accumulator is what
		 * makes this stick - the old version left a stale reset marker behind
		 * that later pushed the counter negative. */
		ms.usage_minutes = 0;
		g_usageBase      = 0;
		g_usageAccumMs   = 0;
		g_usageSaved     = 0;
		(void)EEPROM_SaveUsage(0);
		break;

	case 8:
		tjc.setPage(HOMESCREEN, ms);
		break;

	default:
		break;
	}
}
//------------------------------------HELP------------------------------------
void handleSupport(ScreenEvent event) {
	switch (event.objectID) {

	case 3:
		tjc.setPage(HOMESCREEN, ms);
		break;

	default:
		break;
	}
}
//------------------------------------FAN CONTROL------------------------------------

void handleFanControl(ScreenEvent event) {
	switch (event.objectID) {
	case 29:
		ms.isFanRegOn = event.valueA;
		break;
	case 30:
		// Flap button is a dual-state HMI button whose internal val desyncs from the firmware
		// (it gets forced back to match fanServoState on page refresh). Do NOT trust the
		// reported value - toggle the flap on every press and drive the button to match.
		ms.fanServoState = (ms.fanServoState == g_tune.servoOpenAngle) ? g_tune.servoCloseAngle : g_tune.servoOpenAngle;
		sw = (ms.fanServoState == g_tune.servoOpenAngle);
		tjc.setVal("bt1", sw);
		// applyServoControl() sees the changed fanServoState next pass and drives + detaches.
		saveFlag = true;
	break;
	case 3: //auto selected for filtering fan
		ms.autoFiltering = event.valueA;
		if (ms.autoFiltering) {
			ms.manFiltering = 0;
		}
		break;
	case 4: // auto selected for cooling fan
		ms.autoCooling = event.valueA;
		if (ms.autoCooling) {
			ms.manCooling = 0;
		}
		break;
	case 13: //Filtering slider, this also enables manual mode for filtering
		ms.manFiltering = event.valueA;
		if (ms.manFiltering) {
			ms.autoFiltering = 0;
		}
		ms.filterFanSetSpeed = event.valueA;
		break;

	case 8: //Cooling Slider, this also enables manual mode for cooling

		ms.manCooling = event.valueA;
		if (ms.manCooling) {
			ms.autoCooling = 0;
		}

		ms.coolingFanSetSpeed = event.valueA;
		break;
	case 7: //manual selected for cooling fan

		ms.manCooling = event.valueA;
		if (ms.manCooling) {
			ms.autoCooling = 0;
		}
		break;
	case 12: //manual selected for filter fan
		ms.manFiltering = event.valueA;
		if (ms.manFiltering) {
			ms.autoFiltering = 0;
		}
		break;

	case 19:
		tjc.setPage(HOMESCREEN, ms);
		break;
	default:
		break;
	}
}

//------------------------------------SENSORS------------------------------------

void handleSensors(ScreenEvent event) {

	switch (event.objectID) {
	case 2:
		if (csvDelete()) {
			tjc.setPage(FORMAT_SUCCESS, ms);
		}else {
			tjc.setPage(FORMAT_FAIL, ms);
		}
		break;
	case 3:
		ms.interval += 5;
		if (ms.interval > 15) {
			ms.interval = 5;
		}
		saveFlag = true;
		break;
	case 15:
		tjc.setPage(HOMESCREEN, ms);
		break;
	case 16:
		saveFlag = true;

		ms.logging = !ms.logging;
		break;
	default:
		break;
	}
}
//------------------------------------SPOOL HOLDERS------------------------------------

void handleSpoolHolders(ScreenEvent event) {
	forceUpdate = 1;
	// Handle spool holder page events if needed
	if ((event.objectID >= 6 && event.objectID <= 13)) { //Check boxes
		if (event.objectID % 2 == 0) { //left boxes
			for (size_t i = 0; i < numOfButtons; i++) {
				if (leftIDMaps[i] == event.objectID) {
					int tempIndex = (ms.filamentPage * 4) + i;
					// Bounds check: tempIndex must be within valid range
					if (tempIndex >= 0 && tempIndex <= ms.filamentIndexMax && tempIndex < NUM_OF_FILAMENTS) {
						ms.leftSpoolIndex = i; //for the check list
						ms.leftFilament = (ms.filamentPage * 4)
								+ ms.leftSpoolIndex;
					}

					break;
				}
			}
		} else {
			for (size_t i = 0; i < numOfButtons; i++) {
				if (rightIDMaps[i] == event.objectID) {

					int tempIndex = (ms.filamentPage * 4) + i;
					// Bounds check: tempIndex must be within valid range
					if (tempIndex >= 0 && tempIndex <= ms.filamentIndexMax && tempIndex < NUM_OF_FILAMENTS) {
						ms.rightSpoolIndex = i;
						ms.rightFilament = (ms.filamentPage * 4)
								+ ms.rightSpoolIndex;
					}

					break;
				}
			}
		}
		saveFlag = true;
	}
	switch (event.objectID) {
	case 2:
		tjc.setPage(CAL_SEQUENCE1, ms);
		break;
	case 3:
		tjc.setPage(CALIBRATE1, ms);
		break;
	case 4:
		tjc.setPage(HOMESCREEN, ms);
		break;

	case 21:

		if (ms.filamentPage < ms.filamentPageMax) {
			ms.filamentPage++;
		} else {
			ms.filamentPage = 0;

		}
		break;
	case 28:
		if (ms.filamentPage > 0) {
			ms.filamentPage--;
		} else {
			ms.filamentPage = ms.filamentPageMax;
		}
		break;

	default:
		break;
	}

}

// #TODO write the rpm counter for f140 fans in the timer callback
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM11) {
		const int fact = 12; //this is the factor based on the period
		__disable_irq();
		ms.filterFanASpeed = (filterFanACounter * fact) / osc_per_rev;
		ms.filterFanBSpeed = (filterFanBCounter * fact) / osc_per_rev;
		filterFanBCounter = 0;
		filterFanACounter = 0;
		__enable_irq();
	}
}

//#TODO Count the pulses per tachometer from the f140 fans
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == TACHO_A_Pin) {
		filterFanACounter +=1;
	} else if (GPIO_Pin == TACHO_B_Pin) {
		filterFanBCounter += 1;
	}
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		//  Change the pin mode from Alternate Function to GPIO Output
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		ws281x_done = true;
	}
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) { // Check if the interrupt is from UART2

		if (rxIndexScreen < SCREEN_BUFFER_SIZE) {
			rxBuffer[rxIndexScreen++] = rxDataScreen;
		} else {
			// Buffer full: reset index and discard overflow data
			// This prevents buffer overflow but may cause incomplete message handling
			rxIndexScreen = 0;
			dataReceivedScreen = 0; // Clear flag since buffer was reset
		}
		if (rxDataScreen != 0xFF) {         // Check for end of message

			endCount = 0;
		} else {
			endCount++;
		}

		if (endCount >= 3) {
//			rxBuffer[rxIndexScreen] = '\0'; // null-terminate the string
			if (rxIndexScreen > 5 && rxBuffer[0] == 0x65) {
//#TODO Chabge this back to dataReceivedScreen = 1; ti allow processing of screen data
				dataReceivedScreen = 1;   // set flag to show a complete message
			} else {
				dataReceivedScreen = 0;
			}

			rxIndexScreen = 0;              // reset index for the next message
		}
		HAL_UART_Receive_IT(&huart2, &rxDataScreen, 1);  // enable interrupt
	}

	if (huart->Instance == USART1) {

		if (rxDataPms == 0x42) { // Check for beginning of message
			recievingPms = true;
			rxIndexPms = 0;
			dataReceivedPms = 0;
		}
		if (recievingPms) {
			// Check bounds before writing to prevent buffer overflow
			if (rxIndexPms < PMS_FRAME_LENGTH) {
				pms_data[rxIndexPms] = (uint8_t) rxDataPms;
				rxIndexPms++;
				if (rxIndexPms >= PMS_FRAME_LENGTH) {
					rxIndexPms = 0; // prevent overflow of serial buffer
					recievingPms = false;
					dataReceivedPms = 1;
				}
			} else {
				// Buffer full, reset and stop receiving
				rxIndexPms = 0;
				recievingPms = false;
			}
		}
		HAL_UART_Receive_IT(&huart1, &rxDataPms, 1);  // enable interrupt
	}
}
bool readBme() {
	if (bmeIsFound) {
		if (bme68x_single_measure(&data) == 0) {
			ms.HumidityBME = data.humidity;
			ms.pressure = data.pressure;
			ms.temperature = data.temperature;
			bme68x_GetGasReference();
			bme68x_GetHumidityScore();
			bme68x_GetGasScore();
			data.iaq_score = bme68x_iaq();  // Calculate IAQ
			ms.voc = data.iaq_score;
			return true;
			//
		} else {
			return false;
		}
	} else {
		return false;
	}
}
extern "C" void hard_fault_handler_c(uint32_t *stack) {
	(void)stack;   /* R0-R3, R12, LR, PC, xPSR of the faulting context */

	/* D-04: restart instead of freezing.
	 *
	 * This is reached by a branch (not a call) from HardFault_Handler, so
	 * simply returning re-executes the faulting instruction and faults again -
	 * an endless loop with the machine dead on the bench. Resetting turns an
	 * unrecoverable freeze into a self-clearing reboot.
	 *
	 * Uncomment the breakpoint while debugging to inspect the stacked frame. */
//	__asm("BKPT #0");

	NVIC_SystemReset();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Reset of all peripherals, init the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  __enable_irq();
	HAL_UART_Receive_IT(&huart1, &rxDataPms, 1);
	HAL_UART_Receive_IT(&huart2, &rxDataScreen, 1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim11);

//	HAL_Delay(1000);

	/* ── EEPROM boot sequence ────────────────────────────────────────────────
	 *
	 * Step 1: Check that the EEPROM responds on the I²C bus.
	 *         If it doesn't reply at the default address (0x50), scan all
	 *         possible AT24C addresses (0x50–0x57) — open/floating address
	 *         pins can settle at VCC giving address 0x57.
	 *
	 * Step 2: If no device found at all, skip EEPROM and use compile-time
	 *         defaults. The system works normally but settings won't persist
	 *         across power cycles until the wiring is fixed.
	 *
	 * Step 3: Check the magic word. If missing (brand-new or erased chip),
	 *         write the magic + compile-time defaults (first-boot format).
	 *         Otherwise load the saved config and filament data.
	 *
	 * ─────────────────────────────────────────────────────────────────────── */
	if (!EEPROM_IsReady())
	{
		/* Not at 0x50 — scan the whole range */
		uint8_t found = EEPROM_ScanAddress();   /* also updates g_eeprom_addr */
		(void)found;   /* suppress unused-variable warning */
		/* If scan also fails, g_eeprom_addr is reset to 0x50 and
		 * all subsequent EEPROM calls will return HAL_ERROR gracefully. */
	}

	/* Runtime tuning: load the block from EEPROM (or defaults / migrate), then
	 * apply data.clu from the SD card if it carries a newer configRevision.
	 * Must run before anything reads g_tune. Safe with no EEPROM and/or no SD. */
	/* Boot session id for the CSV log. Read+incremented once per power-on. */
	g_bootId = EEPROM_NextBootId();

	Tuning_Init();

	if (EEPROM_IsReady())
	{
		if (!EEPROM_IsValid())
		{
			/* First boot or blank/erased chip — write magic and defaults */
			EEPROM_Format(&ms, filamentBrand, filamentWeight);
		}
		else
		{
			/* Normal boot — load persistent config and filament data */
			for (int var = 0; var < 3; ++var) {
				if (EEPROM_LoadConfig(&ms)) break;
			}
			/* Sanitize stale servo state from EEPROM (old firmware could have saved
			 * autoServo=true or an out-of-range fanServoState, which would disable the
			 * fan button or drive the servo into a hard stop at boot). */
			ms.autoServo = false;
			if (ms.fanServoState != g_tune.servoOpenAngle && ms.fanServoState != g_tune.servoCloseAngle) {
				ms.fanServoState = g_tune.servoCloseAngle;
			}
			for (int var = 0; var < 3; ++var) {
				if (EEPROM_LoadFilamentData(filamentBrand, filamentWeight)) break;
			}
			/* Reconcile with the SD card: create filament.clu if it is missing,
			 * or import it if a technician has raised its filamentRevision. */
			Filament_Init(filamentBrand, filamentWeight, &ms.filamentIndexMax);
		}
	}
	/* else: EEPROM not found — continue with compile-time defaults */

	/* Restore the gate/flap switch state from the persisted servo state */
	sw = (ms.fanServoState == g_tune.servoOpenAngle);

	/* Load-cell seeds for a board that has never been calibrated.
	 *
	 * ms.calibrationComplete is the authoritative flag: it starts false (see
	 * parameters.cpp), is set true ONLY at the end of the calibration wizard
	 * (handleDone), and is persisted in EEPROM. So "false" means this unit has
	 * never completed calibration and its cal factors are placeholders.
	 *
	 * In that state, seed from the tuning block so the values can be adjusted
	 * per production batch via data.clu instead of requiring a rebuild. Once a
	 * unit is calibrated the flag latches true and its measured factors are
	 * never overwritten - changing these in data.clu will not disturb it. */
	if (!ms.calibrationComplete) {
		ms.leftCalFactor  = g_tune.defaultLeftCal;
		ms.rightCalFactor = g_tune.defaultRightCal;
		ms.leftOffset     = g_tune.defaultLeftOffset;
		ms.rightOffset    = g_tune.defaultRightOffset;
	}

	loadCellLeft.init();
	loadCellLeft.setScale(ms.leftCalFactor);
	loadCellLeft.setOffset(ms.leftOffset);

	loadCellRight.init();
	loadCellRight.setScale(ms.rightCalFactor);
	loadCellRight.setOffset(ms.rightOffset);

	smokeSensor.begin();
	mqSensor.begin();
	memsSensor.begin();
	ssr1.begin();
	ssr2.begin();
	buzzer.begin();
	tjc.setDim(35);
	if (ms.calibrationComplete) {
		tjc.setPage(LNDSCREEN, ms); //TODO: CHANGE BACK TO  LNDSCREEN
		HAL_Delay(2000);
		tjc.setPage(HOMESCREEN, ms);//TODO: CHANGE BACK TO  HOMESCREEN
	} else {
		tjc.setPage(LNDSCREEN2, ms);//TODO: CHANGE BACK TO  LNDSCREEN2

	}
	tjc.setDim(ms.brightness);

//-----------------------------------------------------------------------------

	if (HAL_I2C_IsDeviceReady(&hi2c1, 0x76 << 1, 5, 1000) == HAL_OK) {
		bmeIsFound = true;
	} else if (HAL_I2C_IsDeviceReady(&hi2c1, 0x77 << 1, 5, 1000) == HAL_OK) {
		bmeIsFound = true;
	} else {
		bmeIsFound = false;
	}

	bme68x_start(&data, &hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	servo.start(); // servo left idle (no pulse) until the fan button commands it
 BME_Timestamp = smokeBeginTimestamp = dataLoggerTimestamp = HAL_GetTick();
 standbyTimestamp = HAL_GetTick(); // start the standby timer from boot (it inits to 0 → would dim early)

	/* Machine usage. The dedicated record is authoritative; if it is absent
	 * (first boot on this firmware) seed it from the value that was carried in
	 * the config block, so no accumulated history is lost. */
	{
		uint32_t stored = 0;
		if (!EEPROM_LoadUsage(&stored)) {
			stored = (ms.usage_minutes > 0) ? (uint32_t)ms.usage_minutes : 0u;
			(void)EEPROM_SaveUsage(stored);
		}
		g_usageBase      = stored;
		g_usageSaved     = stored;
		g_usageAccumMs   = 0;
		ms.usage_minutes = (int32_t)stored;
	}
	while (1)

	{
		runMachine();

		updateUsageCounter();

		if (ms.logging) {

			uint32_t logTime = 0;
			if (ms.interval == 5) {
				logTime = 5*60000;
			} else if (ms.interval == 10) {
				logTime = 10 * 60000;
			} else {
				logTime = 60 * 60000;
			}

			if (HAL_GetTick() - dataLoggerTimestamp >= logTime) {
				if (csvInit()) {
					int t = HAL_GetTick() / 60000;

					long lw =ms.useLoadCell?loadCellLeft.getWeight():NOT_FOUND;
					long rw =ms.useLoadCell?loadCellRight.getWeight():NOT_FOUND;

					/* Log the ACTUAL commanded fan speed, not ms.filterFanSetSpeed -
					 * only the manual slider writes that field, so in auto mode
					 * the log recorded 0% while the fan was running. Same root
					 * cause as the usage-counter gating. */
					csvLog(g_bootId, t, ms.temperature, ms.HumidityBME, ms.pressure,
							ms.memsSmokeValue,ms.pm03, ms.pm2, ms.pm10, ms.voc,
							ms.mqSmokeValue,filterFanDisplaySpeed,lw,rw);
					dataLoggerTimestamp = HAL_GetTick();
				}

			}
		}
		if (saveFlag) {
			EEPROM_SaveConfig(&ms);
			saveFlag = false;
		}
		if (dataReceivedScreen == 1) {
			char tempBuffer[SCREEN_BUFFER_SIZE];
			__disable_irq();
			memcpy(tempBuffer, (const uint8_t*) rxBuffer, SCREEN_BUFFER_SIZE);
			dataReceivedScreen = 0;
			__enable_irq();

			// Wake the screen if it was dimmed, but ALWAYS process the event. Previously a
			// press received while dimmed only woke the screen and discarded the action, so
			// the first touch (or every touch, if the screen re-dimmed) never reached its
			// handler - e.g. the flap button would refresh the page but never move the servo.
			if (tjc.dimmed) {
				tjc.setDim(ms.brightness);  // Wake the screen
			}
			event = tjc.readData(tempBuffer);
			/* D-08: commandCode is an unvalidated byte straight off the screen.
			 * Copying that many bytes into dataBuffer[20] from tempBuffer[32]
			 * let a noisy or malformed frame overwrite adjacent variables and
			 * read past the receive buffer. Clamp to both limits. */
			int copyLen = event.commandCode;
			if (copyLen < 0) copyLen = 0;
			if (copyLen > (int) sizeof(dataBuffer))        copyLen = (int) sizeof(dataBuffer);
			if (copyLen > SCREEN_BUFFER_SIZE - 9)          copyLen = SCREEN_BUFFER_SIZE - 9;
			for (int i = 0; i < copyLen; i++) {
				dataBuffer[i] = (uint8_t) tempBuffer[9 + i];
			}
			if (event.pageNumber >= 0 && event.pageNumber <= 7) {
				forceUpdate = true;
			}
			handlePageEvent(event);
			refreshFlag = 1;
			standbyTimestamp = HAL_GetTick(); // Reset the standby timeout after interaction

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//-----------------------------------------------------------------------
		if (!recievingPms && dataReceivedPms == 1) {
			if (isValidPmsData(pms_data, PMS_FRAME_LENGTH)) {
				process_pms_data(pms_data, &pms_info);
				debug_pms_data(&pms_info);
				for (uint8_t x = 0; x < PMS_FRAME_LENGTH; ++x) {
					pms_data[x] = 0;
				}
				dataReceivedPms = 0;
			}
			// Process sensor data as needed.
			ms.pm2 = pms_info.pm2_5_cf1;
			ms.pm10 = pms_info.pm10_cf1;
			ms.pm03 = pms_info.particles_0_3;
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 124;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  // TIM5 clock = 100 MHz. PSC 99 -> 1 MHz (1 us/count); ARR 19999 -> 20 ms frame (50 Hz).
  // This gives ~11 counts/degree instead of the previous ~1 count/12 degrees.
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 49999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HX1SCK_Pin|SSR2_Pin|SSR1_Pin|MUTE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|HX2SCK_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DET_Pin */
  GPIO_InitStruct.Pin = DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TACHO_A_Pin */
  GPIO_InitStruct.Pin = TACHO_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TACHO_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HX1SCK_Pin SSR2_Pin SSR1_Pin MUTE_Pin */
  GPIO_InitStruct.Pin = HX1SCK_Pin|SSR2_Pin|SSR1_Pin|MUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HX1DT_Pin */
  GPIO_InitStruct.Pin = HX1DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HX1DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin HX2SCK_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = CS_Pin|HX2SCK_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ALARM_Pin HX2DT_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin|HX2DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TACHO_B_Pin */
  GPIO_InitStruct.Pin = TACHO_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TACHO_B_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool isValidPmsData(volatile uint8_t *buffer, int size) {
	int length = 32;
//	debug_print("in validation func\n");
	/* D-09: was && - which only rejected a frame when BOTH magic bytes were
	 * wrong, so the header check was effectively bypassed and only the
	 * checksum was guarding the data. */
	if (buffer[0] != 0x42 || buffer[1] != 0x4D) {
//	debug_print("failed in first data \n");

		return false;
	}

	uint16_t checksum = 0;
	for (int i = 0; i < length - 2; i++) {
		checksum += buffer[i];
	}
	uint16_t received_checksum = (buffer[length - 2] << 8) | buffer[length - 1];
	return (checksum == received_checksum);
}

void process_pms_data(volatile uint8_t *buffer, PmsData *data) {

	data->pm1_0_cf1 = (buffer[4] << 8) | buffer[5];  // PM1.0
	data->pm2_5_cf1 = (buffer[6] << 8) | buffer[7];  // PM2.5
	data->pm10_cf1 = (buffer[8] << 8) | buffer[9];  // PM10

	data->pm1_0_atm = (buffer[10] << 8) | buffer[11];
	data->pm2_5_atm = (buffer[12] << 8) | buffer[13];
	data->pm10_atm = (buffer[14] << 8) | buffer[15];

	data->particles_0_3 = (buffer[16] << 8) | buffer[17];
	data->particles_0_5 = (buffer[18] << 8) | buffer[19];
	data->particles_1_0 = (buffer[20] << 8) | buffer[21];
	data->particles_2_5 = (buffer[22] << 8) | buffer[23];
	data->particles_5_0 = (buffer[24] << 8) | buffer[25];
	data->particles_10 = (buffer[26] << 8) | buffer[27];

}

void debug_pms_data(PmsData *data) {
	if (!data)
		return;

	char buffer[128];
	debug_print("\n");

	snprintf(buffer, sizeof(buffer),
			"PM1.0 CF1: %u, PM2.5 CF1: %u, PM10 CF1: %u\r\n", data->pm1_0_cf1,
			data->pm2_5_cf1, data->pm10_cf1);
	debug_print(buffer);

	snprintf(buffer, sizeof(buffer),
			"PM1.0 ATM: %u, PM2.5 ATM: %u, PM10 ATM: %u\r\n", data->pm1_0_atm,
			data->pm2_5_atm, data->pm10_atm);
	debug_print(buffer);

	snprintf(buffer, sizeof(buffer),
			"Particles > 0.3µm: %u, > 0.5µm: %u, > 1.0µm: %u\r\n",
			data->particles_0_3, data->particles_0_5, data->particles_1_0);
	debug_print(buffer);

	snprintf(buffer, sizeof(buffer),
			"Particles > 2.5µm: %u, > 5.0µm: %u, > 10µm: %u\r\n",
			data->particles_2_5, data->particles_5_0, data->particles_10);
	debug_print(buffer);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
