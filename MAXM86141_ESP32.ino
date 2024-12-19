// Device Driver for MAXM86141 Optical Biomedical Sensor
// Controller: ESP32
// Communication Protocol: I2C

#include <Wire.h> // Include the Wire library for I2C communication

#define MAXM86141_I2C_ADDRESS 0x57 // Default I2C address for MAXM86141

// Register addresses (refer to MAXM86141 datasheet)
#define REG_PART_ID 0xFF // Register for Part ID
#define REG_INT_STATUS 0x00 // Register for Interrupt Status
#define REG_FIFO_DATA 0x07 // Register to access FIFO data
#define REG_MODE_CONFIG 0x09 // Register for Mode Configuration
#define REG_LED_CONFIG 0x0C // Register for LED Configuration

// Configuration parameters
#define EXPECTED_PART_ID 0x36 // Expected Part ID for MAXM86141

// Global variables
uint32_t rawHR; // Variable to store raw heart rate data
uint32_t rawSPO2; // Variable to store raw SPO2 data
float HR, SPO2, HRV, RR; // Variables to store processed HR, SPO2, HRV, and RR

// Function to initialize I2C communication
void initI2C() {
    Wire.begin(); // Initialize I2C as master
}

// Function to write to a MAXM86141 register
void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MAXM86141_I2C_ADDRESS); // Begin transmission to the sensor
    Wire.write(reg); // Write the register address
    Wire.write(value); // Write the value to the register
    Wire.endTransmission(); // End transmission
}

// Function to read from a MAXM86141 register
uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(MAXM86141_I2C_ADDRESS); // Begin transmission to the sensor
    Wire.write(reg); // Write the register address
    Wire.endTransmission(false); // End transmission but keep the connection open
    Wire.requestFrom(MAXM86141_I2C_ADDRESS, 1); // Request one byte from the sensor
    return Wire.read(); // Read and return the data
}

// Function to read multiple bytes from a MAXM86141 register
void readFIFOData(uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(MAXM86141_I2C_ADDRESS); // Begin transmission to the sensor
    Wire.write(REG_FIFO_DATA); // Write the FIFO data register address
    Wire.endTransmission(false); // End transmission but keep the connection open
    Wire.requestFrom(MAXM86141_I2C_ADDRESS, length); // Request 'length' bytes from the sensor
    for (int i = 0; i < length; i++) {
        buffer[i] = Wire.read(); // Read each byte and store it in the buffer
    }
}

// Function to initialize the MAXM86141 sensor
bool initSensor() {
    uint8_t partId = readRegister(REG_PART_ID); // Read the Part ID register
    if (partId != EXPECTED_PART_ID) { // Check if the Part ID matches the expected value
        return false; // Sensor not detected, return failure
    }

    // Configure sensor (refer to datasheet for recommended settings)
    writeRegister(REG_MODE_CONFIG, 0x03); // Enable HR and SPO2 mode
    writeRegister(REG_LED_CONFIG, 0x33); // Set LED current levels
    return true; // Sensor initialized successfully
}

// Function to process raw HR and SPO2 data
void processSensorData(uint8_t *data) {
    rawHR = (data[0] << 16) | (data[1] << 8) | data[2]; // Combine 3 bytes to form raw HR data
    rawSPO2 = (data[3] << 16) | (data[4] << 8) | data[5]; // Combine 3 bytes to form raw SPO2 data

    // Example formulas to calculate HR and SPO2
    HR = rawHR / 100.0; // Convert raw HR data to bpm (example scaling factor)
    SPO2 = rawSPO2 / 100.0; // Convert raw SPO2 data to % (example scaling factor)
}

// Function to calculate HRV and RR
void calculateHRVandRR() {
    // HRV and RR estimation formulas (placeholders)
    HRV = 60.0 / HR; // Example: Time between heartbeats (ms)
    RR = HR / 4.0; // Example: Respiration rate (breaths per minute)
}

// Function to filter motion artifacts
float filterNoise(float signal, float prevSignal, float alpha = 0.1) {
    return alpha * signal + (1 - alpha) * prevSignal; // Simple low-pass filter to suppress noise
}

// Main setup function
void setup() {
    Serial.begin(115200); // Initialize serial communication for debugging
    initI2C(); // Initialize I2C communication

    if (!initSensor()) { // Initialize the sensor and check if successful
        Serial.println("Sensor initialization failed!"); // Print error message
        while (1); // Halt execution if sensor initialization fails
    }
    Serial.println("Sensor initialized successfully."); // Print success message
}

// Main loop function
void loop() {
    uint8_t fifoData[6]; // Buffer to store FIFO data

    // Read FIFO data
    readFIFOData(fifoData, 6); // Read 6 bytes of FIFO data from the sensor

    // Process the sensor data
    processSensorData(fifoData); // Extract and process HR and SPO2 data

    // Apply noise filtering
    static float prevHR = 0, prevSPO2 = 0; // Static variables to store previous values
    HR = filterNoise(HR, prevHR); // Apply low-pass filter to HR data
    SPO2 = filterNoise(SPO2, prevSPO2); // Apply low-pass filter to SPO2 data

    prevHR = HR; // Update previous HR value
    prevSPO2 = SPO2; // Update previous SPO2 value

    // Calculate HRV and RR
    calculateHRVandRR(); // Compute HRV and RR based on HR data

    // Output the data
    Serial.print("HR: ");
    Serial.print(HR); // Print heart rate
    Serial.print(" bpm, SPO2: ");
    Serial.print(SPO2); // Print oxygen saturation
    Serial.print(" %, HRV: ");
    Serial.print(HRV); // Print heart rate variability
    Serial.print(" ms, RR: ");
    Serial.print(RR); // Print respiration rate
    Serial.println(" breaths/min");

    delay(1000); // Wait for 1 second before next reading
}