#include "BoardDefinitions.h"
#include "FocuserCommands.h"
#include "SerialComm.h"

#include <TimerEvent.h>

int32_t TargetPosition = 0;                // Target position for the focuser
volatile bool diag_triggered_flag = false; // Flag to indicate if the diagnostic pin was triggered (motor stall)

char Version[] = "V0.1";                            // Version of the firmware
char Build[] = "2023-10-01";                        // Build date of the firmware
char Author[] = "Connor Hutcheson";                 // Author of the firmware
char FirmwareName[] = "ArduFocus-TMC2209";          // Name of the firmware
void processCommand(String command, String params); // function to process commands from the queue                              // function to handle serial events

void processData(String input)
{
  String command = "";     // Initialize the command string
  String params = "";      // Initialize the parameters string
  String parseBuffer = ""; // Initialize the parse buffer

  for (char c : input)
  {
    switch (c)
    {
    case '{':
      command = parseBuffer; // Extract the command from the buffer
      parseBuffer = "";      // Clear the parse buffer
      break;
    case '}':
      params = parseBuffer; // Extract the parameters from the buffer
      parseBuffer = "";     // Clear the parse buffer
      break;
    case '\0':
      // Ignore null characters
      break;
    case '/':
      // Ignore command beginning
      break;
    case ';':
      // Ignore command end
      break;
    default:
      parseBuffer += c; // Append the character to the parse buffer
      break;
    }
  }
  if (parseBuffer.length() > 0)
  {
    command = parseBuffer; // Extract the command from the buffer
    parseBuffer = "";      // Clear the parse buffer
  }
  processCommand(command, params); // Process the command from the queue
}

void processCommand(String command, String params)
{

  char buffer[32]; // Buffer to hold the response string

  if (command[0] == 'S')
  {
    if (command == "SM") // Move the focuser
    {
      if (!connection.connected_to_driver)
        return;
      int32_t steps = strtol(params.c_str(), NULL, 10); // Convert the parameters to an integer
      TargetPosition += steps;                          // Update the target position
      if (TargetPosition > settings.maxSteps)
      {
        TargetPosition = settings.maxSteps; // If the target position is greater than the maximum position, set the target position to the maximum position
        moveTo(TargetPosition);             // Move the focuser to the target position
        return;
      }
      else if (TargetPosition < 0)
      {
        TargetPosition = 0;     // If the target position is less than 0, set the target position to 0
        moveTo(TargetPosition); // Move the focuser to the target position
        return;
      }else{
        move(steps); // Move the focuser by the specified number of steps
        return;
      }
    }
    else if (command == "SMT") // move to a specific position
    {
      if (!connection.connected_to_driver)
        return;                                             // Check if the driver is connected
      int32_t position = strtol(params.c_str(), NULL, 10); // Convert the parameters to an integer
      if (position > settings.maxSteps)
      {
        TargetPosition = settings.maxSteps; // If the position is greater than the maximum position, set the target position to the maximum position
        moveTo(settings.maxSteps);          // If the position is greater than the maximum position, move to the maximum position
        return;
      }
      else if (position < 0)
      {
        TargetPosition = 0; // If the position is less than 0, set the target position to 0
        moveTo(0);
        return;
      }
      else
      {
        TargetPosition = position; // Update the target position
        moveTo(position);          // Move the focuser to the specified position
        return;
      }
    }
    else if (command == "SH")
    {
      if (!connection.connected_to_driver)
        return; // Check if the driver is connected
      halt();   // Stop the focuser
      return;
    }
    else if (command == "S")
    {
      if (!connection.connected_to_driver)
        return; // Check if the driver is connected
      stop();   // Stop the focuser
      return;
    }
    else if (command == "SR")
    {
      if (!connection.connected_to_driver)
        return;           // Check if the driver is connected
      reverseDirection(); // Reverse the direction of the focuser
      return;
    }
    else if (command == "SS")
    {
      if (!connection.connected_to_driver)
        return;                                          // Check if the driver is connected
      uint32_t speed = strtol(params.c_str(), NULL, 10); // Convert the parameters to an integer
      setMaxSpeed(speed);                                // Set the maximum speed of the focuser
      return;
    }
    else if (command == "SA")
    {
      if (!connection.connected_to_driver)
        return;                                                 // Check if the driver is connected
      uint32_t acceleration = strtol(params.c_str(), NULL, 10); // Convert the parameters to an integer
      setAcceleration(acceleration);                            // Set the acceleration of the focuser
      return;
    }
    else if (command == "SC")
    {
      if (!connection.connected_to_driver)
        return;                                // Check if the driver is connected
      uint16_t current = atoi(params.c_str()); // Convert the parameters to an integer
      setCurrent(current);                     // Set the motor current in mA
      return;
    }
    else if (command == "SSG")
    {
      if (!connection.connected_to_driver)
        return;                                 // Check if the driver is connected
      uint8_t threshold = atoi(params.c_str()); // Convert the parameters to an integer
      setStallGuardThreshold(threshold);        // Set the stall guard threshold
      return;
    }
    else if (command == "SMS")
    {
      if (!connection.connected_to_driver)
        return;                              // Check if the driver is connected
      int microsteps = atoi(params.c_str()); // Convert the parameters to an enum value
      setMicrosteps(microsteps);             // Set the microstepping
      return;
    }
    else if (command == "SMP")
    {
      uint32_t maxPosition = strtol(params.c_str(), NULL, 10); // Convert the parameters to an integer
      setMaxPosition(maxPosition);                             // Set the maximum position of the focuser
      return;
    }
    else if (command == "SIL")
    {
      uint8_t state = atoi(params.c_str()); // Convert the parameters to an integer
      setInLED(state);                      // Set the input LED state
      return;
    }
    else if (command == "SOL")
    {
      uint8_t state = atoi(params.c_str()); // Convert the parameters to an integer
      setOutLED(state);                     // Set the output LED state
      return;
    }
  }
  else if (command[0] == 'G')
  {
    if (command == "GP")
    {
      ultoa(getPosition(), buffer, 10); // Get the current position of the focuser
      strcat(buffer, ";");              // Append a semicolon to the position
      sendString(buffer);
      return;
    }
    else if (command == "GIM")
    {
      sendString(isMoving() ? "1;" : "0;");
      return;
    }
    else if (command == "GSS")
    {
      dtostrf(getStepSize(), 1, 3, buffer); // Get the step size of the focuser
      strcat(buffer, ";");                  // Append a semicolon to the position
      sendString(buffer);                   // Debug message
      return;
    }
    else if (command == "GMP")
    {
      ultoa(settings.maxSteps, buffer, 10); // Get the maximum position of the focuser
      strcat(buffer, ";");                  // Append a semicolon to the position
      sendString(buffer);                   // Debug message
      return;
    }
    else if (command == "GT")
    {
      dtostrf(getTemperature(), 1, 2, buffer); // Get the temperature of the focuser
      strcat(buffer, ";");                     // Append a semicolon to the temperature
      sendString(buffer);                      // Debug message
      return;
    }
    else if (command == "GS")
    {
      ultoa(getCurrentSpeed(), buffer, 10); // Get the current speed of the focuser
      strcat(buffer, ";");                  // Append a semicolon to the speed
      sendString(buffer);                   // Debug message
      return;
    }
    else if (command == "GR")
    {
      itoa(getReversed(), buffer, 10); // Get the current direction of the focuser
      strcat(buffer, ";");             // Append a semicolon to the direction
      sendString(buffer);              // Debug message
    }
    else if (command == "GMS")
    {
      ultoa(getMaxSpeed(), buffer, 10); // Get the maximum speed of the focuser
      strcat(buffer, ";");              // Append a semicolon to the speed
      sendString(buffer);               // Debug message
      return;
    }
    else if (command == "GA")
    {
      ultoa(getAcceleration(), buffer, 10); // Get the current acceleration of the focuser
      strcat(buffer, ";");                  // Append a semicolon to the acceleration
      sendString(buffer);                   // Debug message
      return;
    }
    else if (command == "GC")
    {
      if (!connection.connected_to_driver)
        return;                        // Check if the driver is connected
      uint16_t current = getCurrent(); // Get the current motor current in mA
      sprintf(buffer, "%d;", current); // Convert the current to a string                   // Debug message
      sendString(buffer);              // Debug message
      return;
    }
    else if (command == "GSG")
    {
      if (!connection.connected_to_driver)
        return;                                     // Check if the driver is connected
      uint8_t threshold = getStallGuardThreshold(); // Get the current stall guard threshold
      sprintf(buffer, "%d;", threshold);            // Convert the threshold to a string
      sendString(buffer);                           // Debug message
      return;
    }
    else if (command == "GMS")
    {
      if (!connection.connected_to_driver)
        return;                              // check if the driver is connected
      itoa(diag_triggered_flag, buffer, 10); // Get the diagnostic pin state
      strcat(buffer, ";");                   // Append a semicolon to the state
      sendString(buffer);                    // Debug message
    }
    else if (command == "GM")
    {
      if (!connection.connected_to_driver)
        return;                              // Check if the driver is connected
      uint16_t microsteps = getMicrosteps(); // Get the current microstepping
      sprintf(buffer, "%d;", microsteps);    // Convert the microsteps to a string
      sendString(buffer);                    // Debug message
      return;
    }
    else if (command == "GIL")
    {
      uint8_t state = getInLED();      // Read the input LED state
      sprintf(buffer, "%d;", state);   // Convert the state to a string
      sendString("Input LED State: "); // Debug message
      sendString(buffer);              // Debug message
      return;
    }
    else if (command == "GOL")
    {
      uint8_t state = getOutLED();      // Read the output LED state
      sprintf(buffer, "%d;", state);    // Convert the state to a string
      sendString("Output LED State: "); // Debug message
      sendString(buffer);               // Debug message
      return;
    }
    else if (command == "GV")
    {
      sendString("Version: "); // Debug message
      sendString(Version);     // Debug message
      return;
    }
    else if (command == "GB")
    {
      sendString("Build: "); // Debug message
      sendString(Build);     // Debug message
      return;
    }
    else if (command == "GAU")
    {
      sendString("Author: "); // Debug message
      sendString(Author);     // Debug message
      return;
    }
    else if (command == "GF")
    {
      sendString("Firmware: "); // Debug message
      sendString(FirmwareName); // Debug message
      return;
    }
  }
}

TimerEvent connectionTimer;  // Timer for driver connection
TimerEvent temperatureTimer; // Timer for temperature sensor readings
TimerEvent EEPROMTimer;      // Timer for EEPROM operations

// -----------------------
// Arduino Setup and Loop
// -----------------------
void setup()
{
  // pinMode(LED_BUILTIN, OUTPUT);
  setupUART(115200); // Initialize UART at 115200 baud.
  if (setupFocuser() == -1)
    sendString("ERROR");
  setupDriver();
  connectionTimer.set(250, connectToDriver);
  connectToDriver();
  temperatureTimer.set(DHTInterval, []()
                       { if(!isMoving()) readTemperature(); }); // Set the temperature sensor reading interval.

  loadSettings(); // Check and load settings from EEPROM.

  EEPROMTimer.set(30000, []() { // Set the EEPROM operation interval.
    if (connection.connected_to_driver && !isMoving())
    {
      updateSettings();
      checkEEPROM(); // Save the focuser settings to EEPROM.;
    }
  });

  sei(); // Enable global interrupts.
}

void loop()
{
  // logic that blinks the red LED if the driver is not connected and flashes the green LED once if the driver is connected
  if (connection.connected_to_driver && !connection.connected_flag && !connection.connection_complete)
  {
    connection.connected_flag = true;        // Set the connection flag to true.
    digitalWrite(InLEDPin, LOW);             // Turn off the input LED to indicate successful connection.
    digitalWrite(OutLEDPin, HIGH);           // Turn on the output LED to indicate successful connection.
    connectionTimer.reset();                 // Reset the connection timer.
    connectionTimer.set(1000, []() {         // Set a timer to turn off the output LED after 1 second.
      digitalWrite(OutLEDPin, LOW);          // Turn off the output LED.
      connection.flash_connected_led = true; // Reset the flash connected LED flag.
      connectionTimer.set(500, []() {        // Set a timer to turn off the flash connected LED after 500 ms.
        if (isMoving())
          return; // If the focuser is moving, do not test connection
        if (stepperDriver.test_connection())
        {
          connection.connected_to_driver = false;    // Reset the connected to driver flag if the connection is lost.
          connection.connected_flag = false;         // Reset the connected flag.
          connection.connection_complete = false;    // Reset the connection complete flag.
          connection.flash_connected_led = false;    // Reset the flash connected LED flag.
          connectionTimer.set(250, connectToDriver); // Set a timer to try to reconnect to the driver after 250 ms.
        }
      });
    });
  }
  else if (connection.connected_to_driver && connection.connected_flag && !connection.connection_complete)
  {
    if (!connection.connection_complete)
    {
      connection.connection_complete = true; // Set the connection complete flag to true.
    }
  }

  if (connection.connected_to_driver)
  {                                                   // Check if the focuser is running.
    if (TargetPosition > getPosition() && isMoving()) // Check if the target position is greater than the current position and the focuser is moving.
    {                                                 // Mask the direction to see which direction the focuser is moving.
      digitalWrite(InLEDPin, HIGH);                   // Turn on the LED if the focuser is moving in the positive direction.
      digitalWrite(OutLEDPin, LOW);                   // Turn off the output LED.
    }
    else if (TargetPosition < getPosition() && isMoving())
    {
      digitalWrite(OutLEDPin, HIGH); // Turn off the LED if the focuser is moving in the negative direction.
      digitalWrite(InLEDPin, LOW);   // Turn off the input LED.
    }
    else if (!isMoving() && connection.flash_connected_led)
    {
      digitalWrite(InLEDPin, LOW);  // Turn off the input LED if the focuser is not moving.
      digitalWrite(OutLEDPin, LOW); // Turn off the output LED if the focuser is not moving.
    }
  }

  temperatureTimer.update();                  // Update the temperature sensor reading timer.
  connectionTimer.update();                   // Update the connection timer.
  EEPROMTimer.update();                       // Update the EEPROM operation timer.
  processInputWithCallback(';', processData); // Process input with a callback function that handles the ';' terminator.
}
