ArduFocser
ArduFocser is an open-source firmware for Arduino Nano-based motorized focusers, designed for telescope automation. It provides precise control of a stepper motor via serial commands and supports integration with ASCOM drivers for remote operation. The firmware is highly configurable and supports features such as EEPROM settings storage, temperature monitoring, and LED status indication.

Features
Stepper Motor Control: Move to absolute or relative positions, set speed, acceleration, and microstepping.
EEPROM Storage: Persistent storage of configuration and position.
Temperature Monitoring: Periodic temperature readings for compensation.
ASCOM Integration: Serial protocol compatible with ASCOM drivers.
Status LEDs: Visual feedback for connection and movement status.
Robust Serial Command Interface: For both manual and automated control.
Hardware Connections
Arduino Nano Pin	Function	Description
D2	DiagPin	Diagnostic input
D3	DirPin	Stepper direction
D4	RXPin	Serial RX (optional)
D5	TXPin	Serial TX (optional)
D6	MS2Pin	Microstep select
D7	MS1Pin	Microstep select
D8	EnablePin	Stepper enable
D10	StepPin	Stepper step
...	...	...
Note: Connect the stepper driver (e.g., TMC2209) to the corresponding pins. LEDs for status indication should be connected to InLEDPin and OutLEDPin as defined in BoardDefinitions.h.

Serial Communication
Overview
Communication is performed over the Arduino's serial port (default 115200 baud). Commands are sent as ASCII strings and terminated with a semicolon ;. Each command can include parameters, typically separated by curly braces {}.

Example:
SM{100}; — Move focuser by 100 steps.

Command Structure
Command: 2-3 letter code (e.g., SM, GP)
Parameters: Enclosed in {} (optional)
Terminator: ;
Available Commands
Set (S-prefixed) Commands
Command	Description	Example
SM{n}	Move focuser by n steps (relative)	SM{100};
SMT{n}	Move focuser to absolute position n	SMT{5000};
SH	Halt focuser immediately	SH;
S	Stop focuser	S;
SR	Reverse focuser direction	SR;
SS{n}	Set max speed to n	SS{2000};
SA{n}	Set acceleration to n	SA{500};
SC{n}	Set motor current (mA)	SC{600};
SSG{n}	Set stall guard threshold	SSG{10};
SMS{n}	Set microstepping mode	SMS{16};
SMP{n}	Set max position	SMP{10000};
SIL{n}	Set input LED state	SIL{1};
SOL{n}	Set output LED state	SOL{0};
Get (G-prefixed) Commands
Command	Description	Example	Response Example
GP	Get current position	GP;	1234;
GIM	Is focuser moving?	GIM;	1; or 0;
GSS	Get step size	GSS;	0.005;
GMP	Get max position	GMP;	10000;
GT	Get temperature	GT;	22.5;
GS	Get current speed	GS;	1500;
GR	Get reversed direction state	GR;	1; or 0;
GMS	Get max speed	GMS;	2000;
GA	Get acceleration	GA;	500;
GC	Get motor current (mA)	GC;	600;
GSG	Get stall guard threshold	GSG;	10;
GM	Get microstepping mode	GM;	16;
GIL	Get input LED state	GIL;	Input LED State: 1;
GOL	Get output LED state	GOL;	Output LED State: 0;
GV	Get firmware version	GV;	Version: V0.1
GB	Get build date	GB;	Build: 2023-10-01
GAU	Get author	GAU;	Author: Connor Hutcheson
GF	Get firmware name	GF;	Firmware: ArduFocus-TMC2209
ASCOM Driver Integration
Connect Arduino Nano to your PC via USB.
Install ASCOM Platform and the appropriate ASCOM driver for ArduFocser.
Configure the COM port in the ASCOM driver settings to match your Arduino Nano.
Use your astronomy software (e.g., NINA, SGP, APT) to connect to the focuser via ASCOM.
The ASCOM driver communicates with the firmware using the serial protocol described above, sending commands and parsing responses to control and monitor the focuser.

Methods and Architecture
Command Parsing:
The firmware parses incoming serial data, extracting commands and parameters, and dispatches them to handler functions.
Timers:
Uses timer events for periodic tasks (temperature reading, EEPROM updates, connection checks).
EEPROM Management:
Settings and position are periodically saved to EEPROM for persistence.
Status LEDs:
Indicate connection and movement status for user feedback.
Interrupts:
Global interrupts are enabled for responsive operation.
Building and Uploading
Install PlatformIO (https://platformio.org/) in VS Code.
Connect your Arduino Nano via USB.
Build and upload the firmware using PlatformIO's interface or CLI: pio run --target upload
License
This project is open-source and provided under the MIT License.

Credits
Firmware by Connor Hutcheson
Based on open-source community contributions
For more details, see the source code and comments in src/main.cpp.
