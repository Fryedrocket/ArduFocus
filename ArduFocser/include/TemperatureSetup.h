#include <DHT.h>
#include "BoardDefinitions.h"

#ifndef TEMPERATURE_SETUP_H
#define TEMPERATURE_SETUP_H

double cashedTemperature; // Variable to store the last read temperature
DHT probe(DHTPin, DHTType); // Create a DHT sensor object
// DHT_Unified probe(13, DHTType); // Create a DHT sensor object

void setupTemperatureSensor() {
    probe.begin(); // Initialize the DHT sensor
}

double readTemperature() {
    // Read the temperature from the DHT sensor
    double temperature = probe.readTemperature(); // Read temperature in Celsius
    if (isnan(temperature)) {
        return -1; // Return -1 if reading fails
    }
    cashedTemperature = temperature; // Cache the temperature value
    return temperature; // Return the temperature in Celsius
}

#endif // TEMPERATURE_SETUP_H