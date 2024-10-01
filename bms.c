#include <stdio.h>

#define MIN_VOLTAGE 3.0   // Minimum safe voltage (in volts)
#define MAX_VOLTAGE 4.2   // Maximum safe voltage (in volts)
#define MAX_TEMPERATURE 45.0  // Maximum safe temperature (in degrees Celsius)
#define MIN_CHARGE_LEVEL 20   // Minimum safe charge level (in percentage)

// Function to monitor voltage
void monitor_voltage(float voltage) {
    if (voltage < MIN_VOLTAGE) {
        printf("Warning: Low voltage (%.2fV)\n", voltage);
    } else if (voltage > MAX_VOLTAGE) {
        printf("Warning: High voltage (%.2fV)\n", voltage);
    } else {
        printf("Voltage is normal (%.2fV)\n", voltage);
    }
}

// Function to monitor temperature
void monitor_temperature(float temperature) {
    if (temperature > MAX_TEMPERATURE) {
        printf("Warning: High temperature (%.2fC)\n", temperature);
    } else {
        printf("Temperature is normal (%.2fC)\n", temperature);
    }
}

// Function to monitor charge level
void monitor_charge_level(int charge_level) {
    if (charge_level < MIN_CHARGE_LEVEL) {
        printf("Warning: Low charge level (%d%%)\n", charge_level);
    } else {
        printf("Charge level is normal (%d%%)\n", charge_level);
    }
}

int main() {
    // Simulate input values for the battery parameters
    float voltage, temperature;
    int charge_level;
    
    // Input the parameters (in a real system, these would come from sensors)
    printf("Enter the battery voltage (in volts): ");
    scanf("%f", &voltage);
    
    printf("Enter the battery temperature (in Celsius): ");
    scanf("%f", &temperature);
    
    printf("Enter the battery charge level (in percentage): ");
    scanf("%d", &charge_level);
    
    // Monitor the parameters
    monitor_voltage(voltage);
    monitor_temperature(temperature);
    monitor_charge_level(charge_level);

    return 0;
}
