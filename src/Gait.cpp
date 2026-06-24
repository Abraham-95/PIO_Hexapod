#include <Dynamixel2Arduino.h>
#include "Gait.h"

Gait triGait = Gait(
    TRI, {
        0.0f, // leg 1 offset
        0.5f, // leg 2 offset
        0.0f, // leg 3 offset
        0.5f, // leg 4 offset
        0.0f, // leg 5 offset
        0.5f  // leg 6 offset
    },
    0.55f,  // pushFraction
    1.5f,   // gaitSpeedMult
    0.9f,   // strideLengthMult
    100.0f, // liftHeight
    100.0f  // maxStrideLength
);

Gait rippleGait = Gait(
    RIPPLE, {
        0.0f,  // leg 1 offset
        0.67f, // leg 2 offset
        0.33f, // leg 3 offset
        0.83f, // leg 4 offset
        0.17f, // leg 5 offset
        0.5f   // leg 6 offset
    },
    0.6f,    // pushFraction
    1.1f,    // gaitSpeedMult
    0.9f,    // strideLengthMult
    100.0f,  // liftHeight
    100.0f   // maxStrideLength
);

Gait waveGait = Gait(
    WAVE, {
        0.0f,  // leg 1 offset
        0.17f, // leg 2 offset
        0.33f, // leg 3 offset
        0.83f, // leg 4 offset
        0.67f, // leg 5 offset
        0.5f   // leg 6 offset
    },
    0.82f,  // pushFraction
    0.50f,  // gaitSpeedMult
    0.9f,   // strideLengthMult
    130.0f, // liftHeight
    100.0f  // maxStrideLength
);

Gait quadGait = Gait(
    QUAD, {
        0.0f,  // leg 1 offset
        0.33f, // leg 2 offset
        0.66f, // leg 3 offset
        0.0f,  // leg 4 offset
        0.33f, // leg 5 offset
        0.66f  // leg 6 offset
    },
    0.68f,  // pushFraction
    1.0f,   // gaitSpeedMult
    0.9f,   // strideLengthMult
    100.0f, // liftHeight
    100.0f  // maxStrideLength
);

Gait biGait = Gait(
    BI, {
        0.0f,  // leg 1 offset
        0.33f, // leg 2 offset
        0.66f, // leg 3 offset
        0.0f,  // leg 4 offset
        0.33f, // leg 5 offset
        0.66f  // leg 6 offset
    },
    0.35f,  // pushFraction
    2.0f,   // gaitSpeedMult
    0.9f,   // strideLengthMult
    160.0f, // liftHeight
    100.0f  // maxStrideLength
);

Gait hopGait = Gait(
    HOP, {
        0.0f, // leg 1 offset
        0.0f, // leg 2 offset
        0.0f, // leg 3 offset
        0.0f, // leg 4 offset
        0.0f, // leg 5 offset
        0.0f  // leg 6 offset
    },
    0.55f,  // pushFraction (example value)
    1.1f,   // gaitSpeedMult
    1.4f,   // strideLengthMult
    80.0f,  // liftHeight
    125.0f  // maxStrideLength
);

const Gait& getGait(GaitType gaitType) {
    switch (gaitType) {
        case TRI:       return triGait;
        case RIPPLE:    return rippleGait;
        case WAVE:      return waveGait;
        case QUAD:      return quadGait;
        case BI:        return biGait;
        case HOP:       return hopGait;
        default:        return triGait; // Default to TRI if no match found
    }
}

Gait cGait = triGait;
GaitType currentGait = TRI;

const char* gaitToString(GaitType gait){
    switch(gait){
        case TRI:    return "TRIPOD";
        case RIPPLE: return "RIPPLE";
        case WAVE:   return "WAVE";
        case QUAD:   return "QUADRUPED";
        case BI:     return "BIPED";
        case HOP:    return "HOP";
        default:     return "TRIPOD";
    }
}

void setGait(GaitType newGait){
    if(newGait != currentGait){
        currentGait = newGait;

        Serial.print("Gait changed to: ");
        Serial.println(gaitToString(currentGait));
    }
}

GaitType nextGait(GaitType current) {
    return static_cast<GaitType>((current + 1) % 6);
}

GaitType prevGait(GaitType current) {
    return static_cast<GaitType>((current + 5) % 6);
}

uint8_t getGaitCode() {
    switch(currentGait) {
        case TRI:    return 0;
        case RIPPLE: return 1;
        case WAVE:   return 2;
        case QUAD:   return 3;
        case BI:     return 4;
        case HOP:    return 5;
        default: return 255;
    }
}
