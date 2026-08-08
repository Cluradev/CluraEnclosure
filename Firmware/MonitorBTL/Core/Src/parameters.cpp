/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "parameters.h"

char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER] = {
    "Sunlu",         "Prusament",     "Bambu Lab",     "eSUN",
    "PolyMaker",     "ProtoPasta",    "Overture",      "Hatchbox",
    "Amazon Basics", "Colorfabb",     "Matter Hackers","Inland",
    "FormFutura",    "3D Jake",       "GST3D",         "Voxel"
};

int filamentWeight[24]={
    134, 198, 226, 224,
    194, 80,  200, 225,
    212, 202, 215, 178,
    184, 221, 206, 171
};

MachineState ms = {

		// settings page
		.mute = false,.alarm = true, .standbyTime = 5, .interval = 5, .logging = false,
		.brightness = 50, .leftSpoolWeight = 0, .rightSpoolWeight = 0,
		.temperatureBaseline = 0, .impuritiesBaseline = 0,.usage_minutes = 0,
		//Spool
		.leftSpoolIndex = 0,.rightSpoolIndex = 0,.leftSelectFilament=0,
		.rightSelectFilament=0,.filamentSelectPage=0,
		.filamentIndex=0,.filamentIndexMax=15,.filamentPage=0,.filamentPageMax=4,
		//spool holders
				.leftFilament = 0, .rightFilament = 0,
				.leftCalFactor = DEFAULT_LEFT_CAL, .rightCalFactor = DEFAULT_RIGHT_CAL,
				.leftOffset = DEFAULT_LEFT_OFFSET, .rightOffset = DEFAULT_RIGHT_OFFSET,

		// Homescreen
		.Impurities = 0,

		// Lighting
		.lightIndex = 0,.LedBrightness = 0.5, .red = 0, .blue = 0, .green = 0, .ledBar = false,

		//Fan control
		.filterFanASpeed = 0, .filterFanBSpeed = 0, .filterFanSetSpeed = 0,
		.coolingFanSpeed = 0, .coolingFanSetSpeed = 0, .autoCooling = false,
		.manCooling = false, .autoFiltering = false, .manFiltering = false,
		.isFanRegOn = false,



		//Sensors
		.pm03 = NOT_FOUND, .pm2 = NOT_FOUND, .pm10 = NOT_FOUND, .pressure = NOT_FOUND, .voc = NOT_FOUND, .temperature = NOT_FOUND,
		.HumidityBME = NOT_FOUND, .HumidityPMS = NOT_FOUND, .mqSmokeValue = NOT_FOUND, .memsSmokeValue =
				NOT_FOUND,

		//Output
		.servoAngle = 0, .autoServo = false, .servoPower = false, .fanServoState = CLOSE_ANGLE,
		.relay = false,
		//Extras
		.features = 0,  // 0:standard,1:Augmented,2:custom
		.usePms = false, .useBme = false, .useMq2 = false, .useOptical = false,
		.useLedStrip = false, .useLedBar = false, .useLoadCell = false,
		.useMems = false, .useFilterFan = false, .useCoolingFan = false,
		.qrUrl = { 0 }, .calibrationComplete = false, .currentPage = HOMESCREEN
//	.updateScreen {false};

		};
