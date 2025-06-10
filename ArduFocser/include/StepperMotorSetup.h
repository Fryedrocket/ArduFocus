// #include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include "BoardDefinitions.h"
#include "SerialComm.h"

#ifndef STEPPER_MOTOR_SETUP_H
#define STEPPER_MOTOR_SETUP_H

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *focuser = NULL;

SoftwareSerial TMC_serial(RXPin, TXPin);

TMC2209Stepper stepperDriver = TMC2209Stepper(&TMC_serial, 0.11f, 0b00); // Create a TMC2209 driver object

struct connectionState
{                                 // struct should only take one byte becuase it uses bitfields
    bool connected_to_driver : 1; // Flag to check if the driver is connected
    bool connected_flag : 1;      // Flag to check if the connection is established
    bool flash_connected_led : 1; // Flag to check if the flash connected LED is on
    bool connection_complete : 1; // Flag to check if the connection is complete
};

connectionState connection = {false, false, false}; // Initialize the connection state
// -----------------------
// Setup The Focuser Object
// -----------------------
// This sets up the focuser with the defualt settings, either first time or settings saved in EEPROM
int setupFocuser()
{
    settings.stepSize = StepSize;                       // Set the step size in um/step
    settings.maxSteps = MaxPosition;                    // Set the maximum position in steps
    settings.current = StepperCurrent;                  // Set the motor current in mA
    settings.microsteps = 16;                           // Set the microstepping to 1/16
    settings.maxSpeed = StepperMaxSpeed;                // Set the maximum speed in steps/s
    settings.acceleration = StepperAcceleration;        // Set the acceleration in steps/s^2
    settings.stallGuardThreshold = StallGuardThreshold; // Set the stall guard threshold
    settings.position = 0;                              // Set the current position to 0
    settings.reversed = false;                          // Set the direction to normal
    settings.DHTPollInt = DHTInterval;                  // Set the DHT sensor polling interval in milliseconds

    pinMode(StepPin, OUTPUT);   // Set the step pin as output
    pinMode(DirPin, OUTPUT);    // Set the direction pin as output
    pinMode(EnablePin, OUTPUT); // Set the enable pin as output
    engine.init();
    focuser = engine.stepperConnectToPin(StepPin);
    if (focuser == NULL)
    {
        return -1;
    }
    focuser->setDirectionPin(DirPin);
    focuser->setEnablePin(EnablePin);
    focuser->setAutoEnable(true);
    focuser->setSpeedInHz(StepperMaxSpeed);        // Set speed in steps/s
    focuser->setAcceleration(StepperAcceleration); // Set acceleration in steps/s^2
    focuser->setAutoEnable(true); // Enable auto enable/disable of the stepper motor
    return 0;
}

// -----------------------
// Setup The Stepper Motor Driver
// -----------------------
// Sets pins as outputs and creates serial communication for the tmc driver
void setupDriver()
{
    connection.connected_flag = false; // Reset the connection flag
    pinMode(MS1Pin, OUTPUT);           // Set the microstepping pin 1 as output
    pinMode(MS2Pin, OUTPUT);           // Set the microstepping pin 2 as output
    pinMode(DiagPin, INPUT);           // Set the diagnostic pin as input

    TMC_serial.begin(BaudRate);   // Initialize the serial communication with the driver
    TMC_serial.listen();          // Start listening to the serial port
    digitalWrite(EnablePin, LOW); // Enable the driver by setting the enable pin low
    pinMode(InLEDPin, OUTPUT);    // Set the input LED pin as output
    pinMode(OutLEDPin, OUTPUT);   // Set the output LED pin as output
    digitalWrite(InLEDPin, LOW);
    digitalWrite(OutLEDPin, LOW); // Turn off the LEDs

    stepperDriver.begin(); // initialize the driver
}

bool ledState = LOW; // Initialize the LED state

// -----------------------
// Connect to the driver
// -----------------------
// Starts communication with the stepper driver and sets the defualt values
void connectToDriver()
{
    if (stepperDriver.test_connection() == 0)
    {
        digitalWrite(InLEDPin, LOW); // Turn off the InLEDPin LED

        stepperDriver.defaults(); // Set default values for the driver

        stepperDriver.toff(5);                     // Set the off time to 5
        stepperDriver.rms_current(StepperCurrent); // Set the motor current in mA
        stepperDriver.microsteps(16);              // Set microstepping to 1/16

        stepperDriver.pwm_autoscale(true);   // Enable automatic scaling of PWM
        stepperDriver.en_spreadCycle(false); // Disable spreadCycle mode

        stepperDriver.semin(5);           // Set the minimum current for the driver
        stepperDriver.semax(2);           // Set the maximum current for the driver
        stepperDriver.TCOOLTHRS(0xFFFFF); // Set the cool threshold to maximum
        stepperDriver.COOLCONF(0x00);     // Set the cool configuration to default

        stepperDriver.SGTHRS(StallGuardThreshold); // Set the stall guard threshold

        connection.connected_to_driver = true; // Set the driver connection flag to true
        digitalWrite(OutLEDPin, HIGH);         // Turn on the OutLEDPin LED to indicate successful connection
        return;
    }
    else
    {
        TMC_serial.end(); // End the serial communication

        digitalWrite(InLEDPin, ledState);    // Blink the InLEDPin to indecate connection failure
        TMC_serial.begin(BaudRate);          // Reinitialize the serial communication
        TMC_serial.listen();                 // Start listening to the serial port again
        //stepperDriver.beginSerial(BaudRate); // Reinitialize the driver
        ledState = !ledState;                // Toggle the LED state
    }
}

#endif // STEPPER_MOTOR_SETUP_H
