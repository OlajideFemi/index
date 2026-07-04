#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

// ---- Configurable safety thresholds ----
#define MIN_VOLTAGE       3.0   // Minimum safe voltage (V)
#define MAX_VOLTAGE       4.2   // Maximum safe voltage (V)
#define MIN_TEMPERATURE   0.0   // Minimum safe temperature (C) - avoids lithium plating on charge
#define MAX_TEMPERATURE  45.0   // Maximum safe temperature (C)
#define MIN_CHARGE_LEVEL   20   // Minimum safe charge level (%)
#define MAX_CHARGE_LEVEL  100   // Maximum valid charge level (%)
#define VOLTAGE_HYSTERESIS 0.05 // 50mV deadband to prevent chattering
#define TEMP_HYSTERESIS   0.5   // 0.5C deadband
#define MAX_VOLTAGE_SLOPE 1.0   // Maximum voltage change per second (V/s)
#define MAX_TEMP_SLOPE    5.0   // Maximum temperature change per second (C/s)
#define MAX_INPUT_LEN     100   // Maximum input buffer size

// ---- Enhanced status codes ----
typedef enum {
    STATUS_OK = 0,
    STATUS_WARNING_LOW = 1,
    STATUS_WARNING_HIGH = 2,
    STATUS_CRITICAL = 3,
    STATUS_INVALID = 4
} status_t;

// ---- Battery reading structure with history ----
typedef struct {
    float voltage;
    float temperature;
    int   charge_level;
    float prev_voltage;      // Previous reading for rate detection
    float prev_temperature;
    time_t last_reading_time;
    float health_score;      // Calculated health metric
} battery_reading_t;

// ---- Function prototypes ----
status_t monitor_voltage(const battery_reading_t *reading);
status_t monitor_temperature(const battery_reading_t *reading);
status_t monitor_charge_level(int charge_level);
int read_float(const char *prompt, float *out);
int read_float_with_range(const char *prompt, float *out, float min, float max);
int read_int(const char *prompt, int *out);
void print_status_summary(const battery_reading_t *reading, 
                         status_t v, status_t t, status_t c);
void log_reading(const battery_reading_t *reading, status_t overall);
float calculate_battery_health(const battery_reading_t *reading);
const char* status_to_string(status_t status);

// ---- Individual monitors with hysteresis and rate checking ----
status_t monitor_voltage(const battery_reading_t *reading) {
    float voltage = reading->voltage;
    float prev_voltage = reading->prev_voltage;
    
    // Check for rate of change (abnormal sudden changes)
    if (reading->last_reading_time > 0) {
        time_t now = time(NULL);
        double delta_t = difftime(now, reading->last_reading_time);
        if (delta_t > 0) {
            float rate = (voltage - prev_voltage) / delta_t;
            if (fabs(rate) > MAX_VOLTAGE_SLOPE) {
                printf("CRITICAL: Abrupt voltage change (%.2fV/s)\n", rate);
                return STATUS_CRITICAL;
            }
        }
    }
    
    // Check with hysteresis to prevent chattering
    if (voltage < MIN_VOLTAGE - VOLTAGE_HYSTERESIS) {
        printf("WARNING: Low voltage (%.2fV) - below minimum threshold\n", voltage);
        return STATUS_WARNING_LOW;
    } else if (voltage > MAX_VOLTAGE + VOLTAGE_HYSTERESIS) {
        printf("WARNING: High voltage (%.2fV) - above maximum threshold\n", voltage);
        return STATUS_WARNING_HIGH;
    } else if (voltage < MIN_VOLTAGE) {
        printf("NOTICE: Voltage approaching minimum (%.2fV)\n", voltage);
        return STATUS_WARNING_LOW;
    } else if (voltage > MAX_VOLTAGE) {
        printf("NOTICE: Voltage approaching maximum (%.2fV)\n", voltage);
        return STATUS_WARNING_HIGH;
    }
    
    printf("Voltage is normal (%.2fV)\n", voltage);
    return STATUS_OK;
}

status_t monitor_temperature(const battery_reading_t *reading) {
    float temp = reading->temperature;
    float prev_temp = reading->prev_temperature;
    
    // Check for rate of change
    if (reading->last_reading_time > 0) {
        time_t now = time(NULL);
        double delta_t = difftime(now, reading->last_reading_time);
        if (delta_t > 0) {
            float rate = (temp - prev_temp) / delta_t;
            if (fabs(rate) > MAX_TEMP_SLOPE) {
                printf("CRITICAL: Abrupt temperature change (%.2fC/s)\n", rate);
                return STATUS_CRITICAL;
            }
        }
    }
    
    // Check with hysteresis
    if (temp > MAX_TEMPERATURE + TEMP_HYSTERESIS) {
        printf("WARNING: High temperature (%.2fC) - above maximum threshold\n", temp);
        return STATUS_WARNING_HIGH;
    } else if (temp < MIN_TEMPERATURE - TEMP_HYSTERESIS) {
        printf("WARNING: Low temperature (%.2fC) - below minimum threshold\n", temp);
        return STATUS_WARNING_LOW;
    } else if (temp > MAX_TEMPERATURE) {
        printf("NOTICE: Temperature approaching maximum (%.2fC)\n", temp);
        return STATUS_WARNING_HIGH;
    } else if (temp < MIN_TEMPERATURE) {
        printf("NOTICE: Temperature approaching minimum (%.2fC)\n", temp);
        return STATUS_WARNING_LOW;
    }
    
    printf("Temperature is normal (%.2fC)\n", temp);
    return STATUS_OK;
}

status_t monitor_charge_level(int charge_level) {
    if (charge_level < 0 || charge_level > MAX_CHARGE_LEVEL) {
        printf("ERROR: Invalid charge level reading (%d%%)\n", charge_level);
        return STATUS_INVALID;
    } else if (charge_level < MIN_CHARGE_LEVEL) {
        if (charge_level < 10) {
            printf("CRITICAL: Charge level critically low (%d%%)\n", charge_level);
            return STATUS_CRITICAL;
        }
        printf("WARNING: Low charge level (%d%%)\n", charge_level);
        return STATUS_WARNING_LOW;
    } else if (charge_level < 25) {
        printf("NOTICE: Charge level approaching minimum (%d%%)\n", charge_level);
        return STATUS_WARNING_LOW;
    }
    
    printf("Charge level is normal (%d%%)\n", charge_level);
    return STATUS_OK;
}

// ---- Input helpers with validation ----
int read_float(const char *prompt, float *out) {
    char buffer[MAX_INPUT_LEN];
    printf("%s", prompt);
    if (fgets(buffer, sizeof(buffer), stdin) == NULL) {
        printf("Error reading input.\n");
        return 0;
    }
    if (sscanf(buffer, "%f", out) != 1) {
        printf("Invalid input: please enter a numeric value.\n");
        return 0;
    }
    return 1;
}

int read_float_with_range(const char *prompt, float *out, float min, float max) {
    if (!read_float(prompt, out))
        return 0;
    if (*out < min || *out > max) {
        printf("Value out of range (%.2f - %.2f)\n", min, max);
        return 0;
    }
    return 1;
}

int read_int(const char *prompt, int *out) {
    char buffer[MAX_INPUT_LEN];
    printf("%s", prompt);
    if (fgets(buffer, sizeof(buffer), stdin) == NULL) {
        printf("Error reading input.\n");
        return 0;
    }
    if (sscanf(buffer, "%d", out) != 1) {
        printf("Invalid input: please enter an integer value.\n");
        return 0;
    }
    return 1;
}

// ---- Health calculation ----
float calculate_battery_health(const battery_reading_t *reading) {
    float health = 100.0;
    
    // Voltage health (30% weight)
    if (reading->voltage < MIN_VOLTAGE || reading->voltage > MAX_VOLTAGE)
        health -= 30;
    else if (reading->voltage < MIN_VOLTAGE + 0.2 || reading->voltage > MAX_VOLTAGE - 0.2)
        health -= 10;
    
    // Temperature health (30% weight)
    if (reading->temperature > MAX_TEMPERATURE || reading->temperature < MIN_TEMPERATURE)
        health -= 30;
    else if (reading->temperature > 35.0 || reading->temperature < 5.0)
        health -= 15;
    
    // Charge level health (20% weight)
    if (reading->charge_level < 10)
        health -= 20;
    else if (reading->charge_level < MIN_CHARGE_LEVEL)
        health -= 10;
    
    // Rate of change penalty (20% weight)
    if (reading->last_reading_time > 0) {
        time_t now = time(NULL);
        double delta_t = difftime(now, reading->last_reading_time);
        if (delta_t > 0) {
            float v_rate = (reading->voltage - reading->prev_voltage) / delta_t;
            float t_rate = (reading->temperature - reading->prev_temperature) / delta_t;
            if (fabs(v_rate) > MAX_VOLTAGE_SLOPE * 0.5)
                health -= 10;
            if (fabs(t_rate) > MAX_TEMP_SLOPE * 0.5)
                health -= 10;
        }
    }
    
    return health > 0 ? health : 0;
}

// ---- Status to string conversion ----
const char* status_to_string(status_t status) {
    switch(status) {
        case STATUS_OK: return "NORMAL";
        case STATUS_WARNING_LOW: return "WARNING (LOW)";
        case STATUS_WARNING_HIGH: return "WARNING (HIGH)";
        case STATUS_CRITICAL: return "CRITICAL";
        case STATUS_INVALID: return "INVALID";
        default: return "UNKNOWN";
    }
}

// ---- Print detailed status summary ----
void print_status_summary(const battery_reading_t *reading, 
                         status_t v, status_t t, status_t c) {
    printf("\n╔═══════════════════════════════════════════════════════╗\n");
    printf("║            BATTERY STATUS REPORT                      ║\n");
    printf("╠═══════════════════════════════════════════════════════╣\n");
    printf("║ Voltage:     %6.2fV     %-20s ║\n", 
           reading->voltage, status_to_string(v));
    printf("║ Temperature: %6.1f°C    %-20s ║\n", 
           reading->temperature, status_to_string(t));
    printf("║ Charge:      %6d%%      %-20s ║\n", 
           reading->charge_level, status_to_string(c));
    printf("║ Health:      %6.1f%%                               ║\n", 
           reading->health_score);
    printf("╠═══════════════════════════════════════════════════════╣\n");
    
    // Determine overall status
    bool has_critical = (v == STATUS_CRITICAL || t == STATUS_CRITICAL || c == STATUS_CRITICAL);
    bool has_warning = (v != STATUS_OK || t != STATUS_OK || c != STATUS_OK);
    bool has_invalid = (c == STATUS_INVALID);
    
    if (has_invalid) {
        printf("║ OVERALL:     INVALID READING                       ║\n");
    } else if (has_critical) {
        printf("║ OVERALL:     ⚠ CRITICAL CONDITION!                ║\n");
    } else if (has_warning) {
        printf("║ OVERALL:     ⚠ ATTENTION NEEDED                   ║\n");
    } else {
        printf("║ OVERALL:     ✓ ALL SYSTEMS NORMAL                 ║\n");
    }
    printf("╚═══════════════════════════════════════════════════════╝\n");
}

// ---- Logging to CSV file ----
void log_reading(const battery_reading_t *reading, status_t overall) {
    FILE *log = fopen("battery_log.csv", "a");
    if (log == NULL) {
        printf("Warning: Unable to open log file\n");
        return;
    }
    
    // Write header if file is new
    fseek(log, 0, SEEK_END);
    if (ftell(log) == 0) {
        fprintf(log, "Timestamp,Voltage(V),Temperature(C),Charge(%%),Health(%%),Status\n");
    }
    
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char timestamp[30];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    fprintf(log, "%s,%.2f,%.1f,%d,%.1f,%d\n", 
            timestamp, 
            reading->voltage, 
            reading->temperature, 
            reading->charge_level,
            reading->health_score,
            overall);
    fclose(log);
}

int main(void) {
    battery_reading_t reading = {0};
    reading.last_reading_time = time(NULL);
    
    printf("═══════════════════════════════════════════════════════\n");
    printf("          BATTERY MONITORING SYSTEM v2.0             \n");
    printf("═══════════════════════════════════════════════════════\n\n");
    
    // Input with range validation
    printf("Enter battery readings (within safe ranges):\n");
    printf("  Voltage: %.1f - %.1f V\n", MIN_VOLTAGE, MAX_VOLTAGE);
    printf("  Temperature: %.1f - %.1f °C\n", MIN_TEMPERATURE, MAX_TEMPERATURE);
    printf("  Charge: %d - %d %%\n\n", 0, MAX_CHARGE_LEVEL);
    
    if (!read_float_with_range("Voltage (V): ", &reading.voltage, 0, 5.0))
        return 1;
    if (!read_float_with_range("Temperature (°C): ", &reading.temperature, -10, 60))
        return 1;
    if (!read_int("Charge level (%): ", &reading.charge_level))
        return 1;
    
    // Store previous values (simulate first reading)
    reading.prev_voltage = reading.voltage;
    reading.prev_temperature = reading.temperature;
    
    // Calculate health score
    reading.health_score = calculate_battery_health(&reading);
    
    // Monitor each parameter
    status_t v_status = monitor_voltage(&reading);
    status_t t_status = monitor_temperature(&reading);
    status_t c_status = monitor_charge_level(reading.charge_level);
    
    // Determine overall status
    status_t overall = STATUS_OK;
    if (c_status == STATUS_INVALID) {
        overall = STATUS_INVALID;
    } else if (v_status == STATUS_CRITICAL || t_status == STATUS_CRITICAL || c_status == STATUS_CRITICAL) {
        overall = STATUS_CRITICAL;
    } else if (v_status != STATUS_OK || t_status != STATUS_OK || c_status != STATUS_OK) {
        overall = STATUS_WARNING_LOW;
    }
    
    // Print detailed report
    print_status_summary(&reading, v_status, t_status, c_status);
    
    // Log the reading
    log_reading(&reading, overall);
    printf("\nReading logged to battery_log.csv\n");
    
    // Provide recommendations
    printf("\n--- RECOMMENDATIONS ---\n");
    if (overall == STATUS_OK) {
        printf("✓ Battery is in excellent condition. Continue normal operation.\n");
    } else if (overall == STATUS_WARNING_LOW || overall == STATUS_WARNING_HIGH) {
        if (v_status != STATUS_OK)
            printf("⚠ Check charging system and connections.\n");
        if (t_status != STATUS_OK)
            printf("⚠ Battery temperature is outside optimal range. Consider cooling/warming.\n");
        if (c_status != STATUS_OK)
            printf("⚠ Charge level is low. Recharge battery soon.\n");
    } else if (overall == STATUS_CRITICAL) {
        printf("⚠ IMMEDIATE ACTION REQUIRED!\n");
        if (v_status == STATUS_CRITICAL)
            printf("  - Disconnect battery immediately! Voltage instability detected.\n");
        if (t_status == STATUS_CRITICAL)
            printf("  - Thermal runaway risk! Move to safe location.\n");
        if (c_status == STATUS_CRITICAL)
            printf("  - Critical charge level. Recharge immediately.\n");
    } else if (overall == STATUS_INVALID) {
        printf("✗ Invalid readings detected. Please re-check sensors.\n");
    }
    
    // Return appropriate exit code
    int exit_code = (overall == STATUS_OK) ? 0 : 1;
    if (overall == STATUS_CRITICAL) exit_code = 2;
    if (overall == STATUS_INVALID) exit_code = 3;
    
    return exit_code;
}