#include "StepperMotorSetup.h"
#include "BoardDefinitions.h"
#include "TemperatureSetup.h"


#ifndef FOCUSER_COMMANDS_H
#define FOCUSER_COMMANDS_H


enum MICROSTEPPING {
  FULL_STEP = 0,
  MICRO_2 = 2,
  MICRO_4 = 4,
  MICRO_8 = 8,
  MICRO_16 = 16,
  MICRO_32 = 32,
  MICRO_64 = 64,
  MICRO_128 = 128,
  MICRO_256 = 256
};

#pragma region Positional Commands

inline void move(int32_t steps) {
  focuser->move(steps);  // Move a specific number of steps
}

inline void moveTo(uint32_t position) {
  focuser->moveTo(position);  // Move to a specific position in steps
}

inline int32_t getPosition() {
  settings.position = focuser->getCurrentPosition();  // Get current position in steps
  return (focuser->getCurrentPosition());                    // Get current position in steps
}

inline int32_t getMaxPosition() {
  return settings.maxSteps;  // Get maximum position in steps
}

inline void halt() {
  focuser->forceStop();  // Stop the motor
}

inline void stop() {
  focuser->stopMove();  // Stop the motor
}

inline void setMaxPosition(int32_t maxPosition) {
  settings.maxSteps = maxPosition;  // Set maximum position in steps
}

#pragma endregion

#pragma region Driver Commands

inline void setMicrosteps(int microsteps) {
  settings.microsteps = microsteps;  // Set microstepping
  stepperDriver.microsteps(microsteps);     // Set microstepping
}

inline uint16_t getMicrosteps() {
  return stepperDriver.microsteps();  // Get current microstepping
}

inline void setCurrent(uint16_t current) {
  settings.current = current;   // Set motor current in mA
  stepperDriver.rms_current(current);  // Set motor current in mA
}

inline uint16_t getCurrent() {
  return settings.current;  // Get current motor current in mA
}

inline void setStallGuardThreshold(uint8_t threshold) {
  settings.stallGuardThreshold = threshold;  // Set stall guard threshold
  stepperDriver.SGTHRS(threshold);                  // Set stall guard threshold
}

inline uint8_t getStallGuardThreshold() {
  return stepperDriver.SGTHRS();  // Get current stall guard threshold
}

#pragma endregion

#pragma region LED Commands
inline void setInLED(bool state) {
  digitalWrite(InLEDPin, state ? HIGH : LOW);  // Set the input LED state
}

inline void setOutLED(bool state) {
  digitalWrite(OutLEDPin, state ? HIGH : LOW);  // Set the output LED state
}

inline bool getInLED() {
  return digitalRead(InLEDPin);  // Get the input LED state
}

inline bool getOutLED() {
  return digitalRead(OutLEDPin);  // Get the output LED state
}

#pragma endregion

#pragma region Speed and Acceleration Commands

inline void setMaxSpeed(uint32_t speed) {
  settings.maxSpeed = speed;   // Set maximum speed in steps/s
  focuser->setSpeedInHz(speed);       // Set speed in steps/s
  focuser->applySpeedAcceleration();  // Apply the speed and acceleration settings
}

inline int32_t getCurrentSpeed() {
  return (focuser->getCurrentSpeedInMilliHz() / 1000);  // Convert to Hz
}

inline int32_t getMaxSpeed() {
  return (focuser->getCurrentSpeedInMilliHz(false) / 1000);  // Convert to Hz
}

inline void setAcceleration(int32_t acceleration) {
  settings.acceleration = acceleration;  // Set acceleration in steps/s^2
  focuser->setAcceleration(acceleration);       // Set acceleration in steps/s^2
  focuser->applySpeedAcceleration();
}

inline int32_t getAcceleration() {
  return focuser->getCurrentAcceleration();  // Get current acceleration in steps/s^2
}

inline bool isMoving() {
  return focuser->isRunning();  // Check if the motor is moving
}


#pragma endregion

#pragma region Temperature Commands

inline double getTemperature() {
  return cashedTemperature;  // Get the temperature from the sensor
}


#pragma endregion

inline void reverseDirection() {
  settings.reversed = !settings.reversed;        // Reverse the direction of the motor
  focuser->setDirectionPin(DirPin, settings.reversed);  // Reverse the direction of the motor
}

inline bool getReversed() {
  return settings.reversed;  // Check if the motor direction is reversed
}

#pragma region Miscellaneous Commands

inline float getStepSize() {
  return settings.stepSize;
}

inline void updateSettings() {
    settings.stepSize = getStepSize();             // Update the step size in um/step
    settings.maxSteps = getMaxPosition();              // Update the maximum position in steps
    settings.current = getCurrent();              // Update the motor current in mA
    settings.microsteps = getMicrosteps();        // Update the microstepping
    settings.maxSpeed = getMaxSpeed();            // Update the maximum speed in steps/s
    settings.acceleration = getAcceleration();    // Update the acceleration in steps/s^2
    settings.stallGuardThreshold = getStallGuardThreshold(); // Update the stall guard threshold
    settings.position = getPosition();            // Update the current position in steps
    settings.reversed = getReversed();            // Update the direction of the motor
    settings.DHTPollInt = DHTInterval;            // Update the DHT sensor polling interval in milliseconds
}

inline void applySettings(){
    focuser->setSpeedInHz(settings.maxSpeed);        // Apply the maximum speed in steps/s
    focuser->setAcceleration(settings.acceleration); // Apply the acceleration in steps/s^2
    focuser->setDirectionPin(DirPin, settings.reversed);  // Set the direction pin based on the reversed setting
    focuser->setCurrentPosition(settings.position); // Set the current position in steps
    stepperDriver.microsteps(settings.microsteps); // Set the microstepping
    stepperDriver.rms_current(settings.current);   // Set the motor current in mA
    stepperDriver.SGTHRS(settings.stallGuardThreshold); // Set the stall guard threshold
    focuser->applySpeedAcceleration(); // Apply the speed and acceleration settings
}

/**
 * @brief Computes the CRC-16 checksum for a given data block.
 *
 * This function calculates a 16-bit cyclic redundancy check (CRC-16)
 * using the polynomial 0x8005. It processes the data byte-by-byte,
 * applying XOR and bitwise shifts to generate the final checksum.
 *
 * @tparam T The data type of the input.
 * @param data Pointer to the input data block.
 * @param length The number of elements to process (default is 1).
 *               The function automatically scales based on the data type size.
 *
 * @return uint16_t The computed CRC-16 checksum.
 *
 * @note The checksum is stored in the settings object for future integrity checks.
 */
template <typename T>
uint16_t CRC16(const T *data, size_t length = 1)
{
    const uint8_t *byteData = reinterpret_cast<const uint8_t *>(data);
    uint16_t crc = 0xFFFF; // Initial CRC value

    for (size_t i = 0; i < length * sizeof(T); i++)
    {
        crc ^= byteData[i]; // XOR with each byte
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0x8005; // Polynomial for CRC-16
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void loadSettings()
{
    uint16_t EEchecksum;
    EEPROM.get(eepromAddress + sizeof(Settings), EEchecksum); // Load settings from EEPROM
    checksum = CRC16(&settings); // Calculate the current checksum

    // sendString("Loading settings from EEPROM..."); // Debug message
    // sendString("Checksum: 0x");
    // sendString(String(EEchecksum, HEX).c_str()); // Debug message
    // sendString("\n");
    // sendString("Old Checksum: 0x");
    // sendString(String(checksum, HEX).c_str()); // Debug message
    // sendString("\n");


    if (EEchecksum != checksum) // Check if the loaded settings are different from default
    {
        Settings newSettings;
        EEPROM.get(eepromAddress, newSettings); // Read settings from EEPROM
        settings = newSettings; // Update the settings object
        checksum = EEchecksum; // Update the checksum
        // sendString("Current Checksum: 0x");
        // sendString(String(checksum, HEX).c_str()); // Debug message
        // sendString("\n");

        // sendString("Step Size: ");
        // sendString(String(newSettings.stepSize).c_str());
        // sendString("\nMax Pos: ");
        // sendString(String(newSettings.maxSteps).c_str());
        // sendString("\nCurrent: ");
        // sendString(String(newSettings.current).c_str());
        // sendString("\nMicrosteps: ");
        // sendString(String(newSettings.microsteps).c_str());
        // sendString("\nAcceleration: ");
        // sendString(String(newSettings.acceleration).c_str());
        // sendString("\nSG Thresh: ");
        // sendString(String(newSettings.stallGuardThreshold).c_str());
        // sendString("\nPos: ");
        // sendString(String(newSettings.position).c_str());
        // sendString("\nReversed: ");
        // sendString(String(newSettings.reversed).c_str());
        // sendString("\nPoll Int: ");
        // sendString(String(newSettings.DHTPollInt).c_str());
        // tx('\n'); // New line for better readability
        applySettings(); // Apply the loaded settings
    }
}

void checkEEPROM(){
    uint16_t EEChecksum;
    EEPROM.get(eepromAddress + sizeof(Settings), EEChecksum); // Read checksum from EEPROM
    checksum = CRC16(&settings); // Calculate the current checksum
    // sendString("EEPROM Checksum: 0x");
    // sendString(String(EEChecksum, HEX).c_str()); // Debug message
    // sendString("\n");
    // sendString("Current Checksum: 0x");
    // sendString(String(checksum, HEX).c_str()); // Debug message
    // sendString("\n");

    if (EEChecksum != checksum) // Check if the checksum is different
    {
        EEPROM.put(eepromAddress, settings); // Write settings to EEPROM
        EEPROM.put(eepromAddress + sizeof(Settings), checksum); // Write checksum to EEPROM
        // sendString("Settings saved to EEPROM.\n"); // Debug message
    }
    else
    {
        // sendString("Settings already saved in EEPROM.\n"); // Debug message
    }

}
#endif  // FOCUSER_COMMANDS_H