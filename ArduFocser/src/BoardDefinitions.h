#include <Arduino.h>
#include <EEPROM.h>
#include "SerialComm.h"

#ifndef BoardDefinitions_h
#define BoardDefinitions_h

#define StepPin 10
#define DirPin 3
#define EnablePin 8
#define MS1Pin 7
#define MS2Pin 6
#define RXPin 4
#define TXPin 5
#define DiagPin 2
#define StallGuardThreshold 10 // Threshold for stall detection (0-255)
#define BaudRate 57600         // Baud rate for serial communication

#define StepperStepsPerRevolution 200 // Number of steps per revolution for the stepper motor
#define GearRatio 1                   // Gear ratio of the motor x:1 (eg 5:1 = 5 turns of motor for 1 turn of focuser)
#define StepperMaxSpeed 10000         // Maximum speed in steps per second
#define StepperAcceleration 1000      // Acceleration in steps per second squared
uint16_t StepperCurrent = 600;        // Motor current in mA
#define StepSize 0.08                 // Step size in um per step
#define MaxPosition 10000000          // Maximum position in steps

#define DHTPin 13
#define DHTType DHT11
#define DHTInterval 100 // Interval for DHT sensor readings in milliseconds

#define InLEDPin 11  // Pin for the indicator LED
#define OutLEDPin 12 // Pin for the output LED

uint16_t eepromAddress = 0; // EEPROM address to store settings and checksum
uint16_t checksum = 0;      // Checksum for the settings

//extern FastAccelStepper *focuser; // Pointer to the stepper motor object

struct Settings
{
    double stepSize;             // Steps per unit
    int32_t maxSteps;            // Maximum steps
    uint16_t current;            // Current in mA
    uint8_t microsteps;          // Microstepping
    uint32_t maxSpeed;           // Maximum speed in steps/s
    uint32_t acceleration;       // Acceleration in steps/s^2
    uint8_t stallGuardThreshold; // Stall guard threshold
    int32_t position;            // Current position
    bool reversed;               // Reverse direction
    uint8_t DHTPollInt;          // Interval for DHT sensor readings in milliseconds
};

Settings settings; // Global settings object to hold the current configuration



#endif // BoardDefinitions_h