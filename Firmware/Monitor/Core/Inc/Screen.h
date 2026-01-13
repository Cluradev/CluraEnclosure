/*
 * Screen.h
 *
 *  Created on: Jan 18, 2025
 *      Author: Fishbone
 */

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_
//
//#ifndef SCREEN_H
#define DATA_LEN 6
#define BUFFER_SIZE 5
#include "stm32f4xx_hal.h" // Include STM32 HAL for UART communication
#include <cstdint>         // For uint8_t
#include "parameters.h"
#include <cstdio>
#include <vector>

struct ScreenEvent {
    int pageNumber  {-1};   // Current page number
    int objectID  {-1};   // Current page number
    int commandCode {-1};     // ID of the button pressed
    int valueA {-1};     // value of the object pressed
    int valueB {-1};     // value of the object pressed
    int valueC {-1};     // other value of the object
    int valueD {-1};     // other value of the object
    int valueE {-1};     // other value of the object
};

class Screen {
public:
    // Constructor: Accepts a UART handle for communication
    Screen(UART_HandleTypeDef* huart);

    // Destructor
    ~Screen();

    // Initialize the screen (if needed)
    void init();


    bool dimmed;
    void setPic(const char* component, const int val);
    void setText(const char* component, const char* text);
    void setText(const char* component, const int val);
    void setText(const char* component, const long val);
    void setPage(uint8_t newPage,  MachineState &machine);

    void setVal(const char* component, const int val);
    void setAph(const char* component, const int val);

    void setDim(const int val);
    void off();
    void setEEPROM(int address, uint8_t data);
    uint8_t getEEPROM(int address );
    // Change the state of a button (e.g., on/off)
    void setButtonState(const char* button, bool state);
    void setButtonState(int buttonid, bool state);
    void setCheckboxState(int chkbxid, bool state);

    void handleTouchEvent();

    ScreenEvent readData(const char* data); // Read data from the screen

    void sendCommand(const char* command);
    void sendData(int data);
    void sendData(const char* data);
    void updateScreenInfo();

private:
    // UART handle for communication
    UART_HandleTypeDef* _huart;

    uint8_t buffer[BUFFER_SIZE];





    // Send TJC's specific end command (0xFF 0xFF 0xFF)
    void sendEnd();
};

#endif // SCREEN_H
