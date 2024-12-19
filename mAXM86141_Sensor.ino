
#include <Wire.h> // Include Wire library for I2C communication

// Define MAXM86141 I2C Address
#define MAXM86141_I2C_ADDR 0x57

// Define register addresses for MAXM86141
#define MAXM86141_REG_HR_DATA 0x0F    // Register address for heart rate data
#define MAXM86141_REG_SPO2_DATA 0x10 // Register address for SPO2 data

// Function to initialize I2C communication with the sensor
void initSensor() {
    Wire.begin(); // Initialize I2C as master
    Serial.begin(115200); // Start serial communication for debugging
    Serial.println("Sensor initialization complete.");
}

// Function to read 16-bit data from a specific register
uint16_t readRegister16(uint8_t reg) {
    Wire.beginTransmission(MAXM86141_I2C_ADDR); // Begin transmission to sensor
    Wire.write(reg); // Write the register address
    Wire.endTransmission(false); // End transmission but keep connection open
    Wire.requestFrom(MAXM86141_I2C_ADDR, 2); // Request 2 bytes of data

    uint16_t value = 0;
    if (Wire.available() == 2) {
        value = (Wire.read() << 8) | Wire.read(); // Combine two bytes into 16-bit value
    }
    return value; // Return the read value
}

// Function to read HR data from the sensor
uint16_t readHRData() {
    return readRegister16(MAXM86141_REG_HR_DATA); // Read data from HR register
}

// Function to read SPO2 data from the sensor
uint16_t readSPO2Data() {
    return readRegister16(MAXM86141_REG_SPO2_DATA); // Read data from SPO2 register
}

// Function to calculate HRV using HR data
float calculateHRV(uint16_t hrData) {
    if (hrData == 0) return 0; // Avoid division by zero
    return 60.0 / (hrData / 100.0); // Example: Calculate HRV as the time between beats (ms)
}

// Function to calculate RR using HR and SPO2 data
float calculateRR(uint16_t hrData, uint16_t spo2Data) {
    // Example: Basic RR formula (adjust based on application)
    return hrData / 400.0 + (spo2Data / 10000.0); // Placeholder formula
}

// Main setup function
void setup() {
    initSensor(); // Initialize the sensor communication
}

// Main loop function
void loop() {
    // Continuously read HR and SPO2 data
    uint16_t hrData = readHRData(); // Read HR data
    uint16_t spo2Data = readSPO2Data(); // Read SPO2 data

    // Calculate HRV and RR
    float hrv = calculateHRV(hrData); // Calculate HRV
    float rr = calculateRR(hrData, spo2Data); // Calculate RR

    // Output the values to the serial monitor
    Serial.print("HR: ");
    Serial.print(hrData / 100.0); // Convert and print HR data in bpm
    Serial.print(" bpm, SPO2: ");
    Serial.print(spo2Data / 100.0); // Convert and print SPO2 data as percentage
    Serial.print(" %, HRV: ");
    Serial.print(hrv); // Print HRV in ms
    Serial.print(" ms, RR: ");
    Serial.print(rr); // Print RR in breaths/min
    Serial.println(" breaths/min");

    delay(1000); // Wait for 1 second before the next reading
}
