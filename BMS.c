#include <stdio.h>

// ---- Configurable safety thresholds ----
#define MIN_VOLTAGE       3.0   // Minimum safe voltage (V)
#define MAX_VOLTAGE       4.2   // Maximum safe voltage (V)
#define MIN_TEMPERATURE   0.0   // Minimum safe temperature (C) - avoids lithium plating on charge
#define MAX_TEMPERATURE  45.0   // Maximum safe temperature (C)
#define MIN_CHARGE_LEVEL   20   // Minimum safe charge level (%)
#define MAX_CHARGE_LEVEL  100   // Maximum valid charge level (%)

typedef enum {
    STATUS_OK = 0,
    STATUS_WARNING = 1,
    STATUS_INVALID = 2
} status_t;

typedef struct {
    float voltage;
    float temperature;
    int   charge_level;
} battery_reading_t;

// ---- Individual monitors ----
status_t monitor_voltage(float voltage) {
    if (voltage < MIN_VOLTAGE) {
        printf("Warning: Low voltage (%.2fV)\n", voltage);
        return STATUS_WARNING;
    } else if (voltage > MAX_VOLTAGE) {
        printf("Warning: High voltage (%.2fV)\n", voltage);
        return STATUS_WARNING;
    }
    printf("Voltage is normal (%.2fV)\n", voltage);
    return STATUS_OK;
}

status_t monitor_temperature(float temperature) {
    if (temperature > MAX_TEMPERATURE) {
        printf("Warning: High temperature (%.2fC)\n", temperature);
        return STATUS_WARNING;
    } else if (temperature < MIN_TEMPERATURE) {
        printf("Warning: Low temperature (%.2fC)\n", temperature);
        return STATUS_WARNING;
    }
    printf("Temperature is normal (%.2fC)\n", temperature);
    return STATUS_OK;
}

status_t monitor_charge_level(int charge_level) {
    if (charge_level < 0 || charge_level > MAX_CHARGE_LEVEL) {
        printf("Invalid charge level reading (%d%%)\n", charge_level);
        return STATUS_INVALID;
    } else if (charge_level < MIN_CHARGE_LEVEL) {
        printf("Warning: Low charge level (%d%%)\n", charge_level);
        return STATUS_WARNING;
    }
    printf("Charge level is normal (%d%%)\n", charge_level);
    return STATUS_OK;
}

// ---- Input helpers with validation ----
int read_float(const char *prompt, float *out) {
    printf("%s", prompt);
    if (scanf("%f", out) != 1) {
        printf("Invalid input: please enter a numeric value.\n");
        while (getchar() != '\n' && !feof(stdin)); // clear bad input
        return 0;
    }
    return 1;
}

int read_int(const char *prompt, int *out) {
    printf("%s", prompt);
    if (scanf("%d", out) != 1) {
        printf("Invalid input: please enter an integer value.\n");
        while (getchar() != '\n' && !feof(stdin));
        return 0;
    }
    return 1;
}

int main(void) {
    battery_reading_t reading;

    if (!read_float("Enter the battery voltage (in volts): ", &reading.voltage))
        return 1;
    if (!read_float("Enter the battery temperature (in Celsius): ", &reading.temperature))
        return 1;
    if (!read_int("Enter the battery charge level (in percentage): ", &reading.charge_level))
        return 1;

    printf("\n--- Battery Status Report ---\n");
    status_t v_status = monitor_voltage(reading.voltage);
    status_t t_status = monitor_temperature(reading.temperature);
    status_t c_status = monitor_charge_level(reading.charge_level);

    int overall_warning = (v_status != STATUS_OK) ||
                           (t_status != STATUS_OK) ||
                           (c_status != STATUS_OK);

    printf("\nOverall status: %s\n",
           overall_warning ? "ATTENTION NEEDED" : "ALL SYSTEMS NORMAL");

    return overall_warning ? 1 : 0;
}