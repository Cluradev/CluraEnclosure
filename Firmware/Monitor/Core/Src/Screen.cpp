/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "Screen.h"
#include <cstring>
// Constructor
Screen::Screen(UART_HandleTypeDef* huart) : _huart(huart) {}

// Destructor
Screen::~Screen() {}

// Initialize the screen
void Screen::init() {
dimmed = false;
}



ScreenEvent Screen::readData( const char* data) // Read data from the screen
{
	ScreenEvent _event;
	if((int) data[0]==0x65 ){
		_event.pageNumber   = ((int)data[1]);
		_event.objectID     = ((int)data[2]);
		_event.commandCode  = ((int)data[3]);
		_event.valueA   = ((int)data[4]);
		_event.valueB   = ((int)data[5]);
		_event.valueC   = ((int)data[6]);
		_event.valueD   = ((int)data[7]);
		_event.valueE   = ((int)data[8]);

	}

	    return _event;
}
// Send a command to the screen
void Screen::setDim(const int val){
	char command[10];
	val>0?this->dimmed=false:this->dimmed=true;
	    snprintf(command, sizeof(command), "dim=%d", val);
	    sendCommand(command);

}
void Screen::off(){
this->setDim(0);

}
void Screen::sendCommand(const char* command) {
    sendData(command);
    sendEnd();
    HAL_Delay(10);
}

void Screen::setPage(uint8_t newPage, MachineState &machine) {
	 machine.currentPage = newPage;
    char command[10];
    snprintf(command, sizeof(command), "page %d", newPage);
    sendCommand(command);
}
void Screen::setPic(const char* component, const int val) {
    char command[64];
    snprintf(command, sizeof(command), "%s.pic=%d", component, val);
    sendCommand(command);
}
// Update the text of a component
void Screen::setText(const char* component, const char* text) {
    char command[64];
    snprintf(command, sizeof(command), "%s.txt=\"%s\"", component, text);
    sendCommand(command);
}
void Screen::setText(const char* component, const int val) {
    char command[64];
    snprintf(command, sizeof(command), "%s.txt=\"%d\"", component, val);
    sendCommand(command);
}
void Screen::setVal(const char* component, const int val) {
    char command[64];
    snprintf(command, sizeof(command), "%s.val=%d", component, val);
    sendCommand(command);
}
void Screen::setAph(const char* component, const int val) {
    char command[64];
    snprintf(command, sizeof(command), "%s.aph=%d", component, val);
    sendCommand(command);
}
void Screen::setText(const char* component, const long val) {
    char command[64];
    snprintf(command, sizeof(command), "%s.txt=\"%ld\"", component, val);
    sendCommand(command);
}
// Change the state of a button
void Screen::setButtonState(const char* button, bool state) {
    char command[64];
    snprintf(command, sizeof(command), "%s.val=%d", button, state ? 1 : 0);
    sendCommand(command);
}
void Screen::setCheckboxState(int chkbxid, bool state) {
    char command[64];
    snprintf(command, sizeof(command), "c%d.val=%d", chkbxid, state ? 1 : 0);
    sendCommand(command);
}
void Screen::setButtonState(int buttonid, bool state) {
    char command[64];
    snprintf(command, sizeof(command), "bt%d.val=%d", buttonid, state ? 1 : 0);
    sendCommand(command);
}
// Send raw data to the screen
void Screen::sendData(const char* data) {
    HAL_UART_Transmit(_huart, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
}
void Screen::sendData(int data) {
	 char buffer[16]; // Buffer to hold the ASCII representation of the number
	    unsigned int length;
	    // Convert the integer to a string
	    length = snprintf(buffer, sizeof(buffer), "%d", data);
	    // Ensure the string fits into the buffer
	    if (length > 0 && length < sizeof(buffer)) {
	    // Transmit the string over UART
	        HAL_UART_Transmit(_huart, (uint8_t *)buffer, length, HAL_MAX_DELAY);
	    }
}

// Send TJC end command (0xFF 0xFF 0xFF)
void Screen::sendEnd() {
    uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
    HAL_UART_Transmit(_huart, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
}
