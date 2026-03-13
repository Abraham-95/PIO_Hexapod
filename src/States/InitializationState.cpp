#include <Dynamixel2Arduino.h>
#include "State.h"
#include "LegManager.h"
#include "Utils.h"

void InitializationState::init() {
    Serial.println("Initialization State.");
    int8_t found_dynamixel = 0;

    for(int id = 0; id < DXL_BROADCAST_ID; id++) {
        if(dxl.ping(id)) {
            DEBUG_SERIAL.print("ID : "); DEBUG_SERIAL.print(id);
            DEBUG_SERIAL.print(", Model Number: "); DEBUG_SERIAL.println(dxl.getModelNumber(id));
            found_dynamixel++;
        }
    }
    DEBUG_SERIAL.print("Total "); DEBUG_SERIAL.print(found_dynamixel);
    DEBUG_SERIAL.println(" DYNAMIXEL(s) found!");

    for (int i = 0; i < 6; i++) {
        moveToPos(i, baseLegCalibrationPosition);
    }
    delay(100);
}

void InitializationState::exit() {
}

void InitializationState::loop() {
}
