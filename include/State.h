#pragma once

#include "Utils.h"
#include "Gait.h"
#include "LegManager.h"
#include "Gyro.h"

class State {
    public:
    virtual const char* getStateName() const = 0;
        virtual void init() = 0;
        virtual void loop() = 0;
        virtual void exit() = 0;
};

class InitializationState : public State {
    public:
        const char* getStateName() const override { return "Initialization"; }
        void loop() override;
        void init() override;
        void exit() override;
};

class JumpingState : public State {
    public:
        const char* getStateName() const override { return "Jumping"; }
        void loop() override;
        void init() override;
        void exit() override;
};

class SleepState : public State {
    public:
        const char* getStateName() const override { return "Sleep"; }
        void loop() override;
        void init() override;
        void exit() override;
    private:
        Vector3 target;
};

class StandingState : public State {
    public:
        const char* getStateName() const override { return "Standing"; }
        void loop() override;
        void init() override;
        void exit() override;
        void calculateLegAdjustOrder();
};

class WalkingState : public State {
    public:
        const char* getStateName() const override { return "Walking"; }
        void loop() override;
        void init() override;
        void exit() override;

        bool isIdle() const;
        Vector3 getGaitPoint(int leg, float pushFraction);
    private:
        float forwardAmount = 0.0f;
        float turnAmount = 0.0f;

        float tArray[6] = {0};
        int cycleProgress[6] = {0};

        LegState legStates[6];
        Vector3 cycleStartPoints[6];

        bool gaitChangeRequested = false;
        GaitType nextGait;
};

class GyroState : public State {
    public:
        const char* getStateName() const override { return "Gyro State"; }
        void loop() override;
        void init() override;
        void exit() override;
    private:
        Vector2 joy1TargetVector;
        Vector2 joy1CurrentVector;

        Vector2 joy2TargetVector;
        Vector2 joy2CurrentVector;

        Vector2 gyroTargetVector;
        Vector2 gyroCurrentVector;

        Vector3 basePoints[6];
};

extern State *currentState;
extern State *previousState;

extern InitializationState *initializationState;
extern JumpingState *jumpingState;
extern SleepState *sleepState;
extern StandingState *standingState;
extern WalkingState *walkingState;
extern GyroState *gyroState;

const char* stateToString(State* state);
void changeState(State* next);
extern uint8_t getStateCode();
