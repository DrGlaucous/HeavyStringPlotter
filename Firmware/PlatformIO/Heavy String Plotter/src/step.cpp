

#include <Arduino.h>
#include "main.h"
#include "configuration.h"



struct stepperInfo
{
    //holds the number of steps the motor is set to make
    int stepsInQueue;
    //direction of stepping
    bool stepDirection;
    //enabled
    bool enabled;
    //acceleration multiplier, what percentage of max speed will is the stepper currently at?
    int accelerationPercentage;

};

#define MOTOR_COUNT 2
stepperInfo steppers[MOTOR_COUNT];


void initSteppers()
{
    pinMode(DIRECTION_L, OUTPUT);
    pinMode(ENABLE_L, OUTPUT);
    pinMode(STEP_L, OUTPUT);
    pinMode(DIRECTION_R, OUTPUT);
    pinMode(ENABLE_R, OUTPUT);
    pinMode(STEP_R, OUTPUT);

    //init variables
    for(int i = 0; i < MOTOR_COUNT; ++i)
    {
        memset(&steppers[i], 0, sizeof(stepperInfo));
    }

}


//
void moveMM()
{

}








