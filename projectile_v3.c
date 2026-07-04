/*
 * Projectile Motion Simulator
 * Features: optional air resistance, validated input, CSV export,
 * multi-scenario comparison, and edge-case safeguards.
 * Compile: gcc -Wall -Wextra -pedantic -std=c11 projectile_submission_ready.c -lm -o projectile
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <float.h>

#define G 9.81
#define RHO 1.225
#define PI 3.14159265358979323846
#define MAX_STEPS 1000000
#define DEFAULT_OUTPUT_FILE "projectile_data.csv"

#define MIN_VELOCITY 0.1
#define MAX_VELOCITY 1000.0
#define MIN_ANGLE 0.1
#define MAX_ANGLE 89.9
#define MIN_DT 0.0001
#define MAX_DT 10.0

typedef struct {
    bool enable_drag;
    double drag_coefficient;
    double cross_section;
    double mass;
    bool show_animation;
    bool export_csv;
    bool verbose;
    char csv_filename[128];
} sim_config_t;

typedef struct {
    double x, y;
    double vx, vy;
    double ax, ay;
    double time;
} projectile_state_t;

static bool is_valid_number(double value) {
    return isfinite(value);
}

static void clear_remaining_input(void) {
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF) {
        ;
    }
}

static int read_double_retry(const char *prompt, double *out, double min, double max) {
    char buffer[128];
    char *endptr = NULL;
    double value;

    for (;;) {
        printf("%s", prompt);
        if (fgets(buffer, sizeof(buffer), stdin) == NULL) {
            printf("Error: input could not be read.\n");
            return 0;
        }

        if (strchr(buffer, '\n') == NULL) {
            clear_remaining_input();
        }

        errno = 0;
        value = strtod(buffer, &endptr);

        while (*endptr == ' ' || *endptr == '\t') {
            endptr++;
        }

        if (errno != 0 || endptr == buffer || (*endptr != '\n' && *endptr != '\0') || !is_valid_number(value)) {
            printf("Invalid input. Please enter a valid number.\n");
            continue;
        }

        if (value < min || value > max) {
            printf("Value out of range. Enter a value between %.4f and %.4f.\n", min, max);
            continue;
        }

        *out = value;
        return 1;
    }
}

static int read_yes_no_retry(const char *prompt) {
    char response[32];

    for (;;) {
        printf("%s (y/n): ", prompt);
        if (fgets(response, sizeof(response), stdin) == NULL) {
            return 0;
        }

        if (response[0] == 'y' || response[0] == 'Y') {
            return 1;
        }
        if (response[0] == 'n' || response[0] == 'N') {
            return 0;
        }

        printf("Please enter y or n.\n");
    }
}

static void update_with_drag(projectile_state_t *state, double dt, const sim_config_t *config) {
    const double speed = hypot(state->vx, state->vy);
    const double drag_factor = 0.5 * RHO * config->drag_coefficient *
                               config->cross_section / config->mass;

    if (speed > 1e-9) {
        /* Drag acceleration vector: a_drag = -k * |v| * v */
        state->ax = -drag_factor * speed * state->vx;
        state->ay = -G - drag_factor * speed * state->vy;
    } else {
        state->ax = 0.0;
        state->ay = -G;
    }

    state->vx += state->ax * dt;
    state->vy += state->ay * dt;
    state->x += state->vx * dt;
    state->y += state->vy * dt;
    state->time += dt;
}

static void update_no_drag(projectile_state_t *state, double dt) {
    state->ax = 0.0;
    state->ay = -G;
    state->vy += state->ay * dt;
    state->x += state->vx * dt;
    state->y += state->vy * dt;
    state->time += dt;
}

static int validate_simulation_inputs(double v0, double angle, double dt, const sim_config_t *config) {
    if (config == NULL) {
        printf("Error: simulation configuration is missing.\n");
        return 0;
    }
    if (!is_valid_number(v0) || v0 < MIN_VELOCITY || v0 > MAX_VELOCITY) {
        printf("Error: initial velocity must be between %.1f and %.1f m/s.\n", MIN_VELOCITY, MAX_VELOCITY);
        return 0;
    }
    if (!is_valid_number(angle) || angle < MIN_ANGLE || angle > MAX_ANGLE) {
        printf("Error: launch angle must be between %.1f and %.1f degrees.\n", MIN_ANGLE, MAX_ANGLE);
        return 0;
    }
    if (!is_valid_number(dt) || dt < MIN_DT || dt > MAX_DT) {
        printf("Error: time step must be between %.4f and %.1f seconds.\n", MIN_DT, MAX_DT);
        return 0;
    }
    if (config->enable_drag) {
        if (!is_valid_number(config->drag_coefficient) || config->drag_coefficient < 0.0) {
            printf("Error: drag coefficient must be non-negative.\n");
            return 0;
        }
        if (!is_valid_number(config->cross_section) || config->cross_section <= 0.0) {
            printf("Error: cross-sectional area must be positive.\n");
            return 0;
        }
        if (!is_valid_number(config->mass) || config->mass <= 0.0) {
            printf("Error: mass must be positive.\n");
            return 0;
        }
    }
    return 1;
}

static int simulate_projectile(double v0, double angle, double dt, const sim_config_t *config) {
    if (!validate_simulation_inputs(v0, angle, dt, config)) {
        return 1;
    }

    const double theta = angle * PI / 180.0;
    projectile_state_t state = {
        .x = 0.0,
        .y = 0.0,
        .vx = v0 * cos(theta),
        .vy = v0 * sin(theta),
        .ax = 0.0,
        .ay = -G,
        .time = 0.0
    };

    const double analytic_time = 2.0 * state.vy / G;
    const double analytic_range = v0 * cos(theta) * analytic_time;
    const double analytic_max_height = (state.vy * state.vy) / (2.0 * G);

    double max_y = state.y;
    double previous_x = state.x;
    double previous_y = state.y;
    double previous_time = state.time;
    double previous_vx = state.vx;
    double previous_vy = state.vy;
    int step_count = 0;
    bool landed = false;

    FILE *csv_file = NULL;
    if (config->export_csv) {
        const char *filename = strlen(config->csv_filename) > 0 ? config->csv_filename : DEFAULT_OUTPUT_FILE;
        csv_file = fopen(filename, "w");
        if (csv_file == NULL) {
            perror("CSV export failed");
            return 1;
        }
        fprintf(csv_file, "Time(s),X(m),Y(m),Vx(m/s),Vy(m/s),Ax(m/s^2),Ay(m/s^2)\n");
    }

    if (config->verbose) {
        printf("\nProjectile Motion Simulation\n");
        printf("Initial velocity: %.2f m/s\n", v0);
        printf("Launch angle:     %.2f degrees\n", angle);
        printf("Time step:        %.4f s\n", dt);
        printf("Air resistance:   %s\n", config->enable_drag ? "enabled" : "disabled");
        if (config->enable_drag) {
            printf("Cd: %.3f, area: %.5f m^2, mass: %.3f kg\n",
                   config->drag_coefficient, config->cross_section, config->mass);
        }
        printf("\n%-10s %-12s %-12s %-12s %-12s %-12s\n",
               "Time(s)", "X(m)", "Y(m)", "Vx(m/s)", "Vy(m/s)", "Status");
        printf("----------------------------------------------------------------------------\n");
    }

    while (!landed && step_count < MAX_STEPS) {
        previous_x = state.x;
        previous_y = state.y;
        previous_time = state.time;
        previous_vx = state.vx;
        previous_vy = state.vy;

        if (config->enable_drag) {
            update_with_drag(&state, dt, config);
        } else {
            update_no_drag(&state, dt);
        }

        step_count++;

        if (!is_valid_number(state.x) || !is_valid_number(state.y) ||
            !is_valid_number(state.vx) || !is_valid_number(state.vy)) {
            printf("Error: numerical overflow occurred. Try a smaller time step or lower velocity.\n");
            if (csv_file != NULL) {
                fclose(csv_file);
            }
            return 1;
        }

        if (state.y > max_y) {
            max_y = state.y;
        }

        if (state.y <= 0.0 && state.time > 0.0) {
            const double denominator = previous_y - state.y;
            const double fraction = fabs(denominator) > DBL_EPSILON ? previous_y / denominator : 1.0;

            state.x = previous_x + fraction * (state.x - previous_x);
            state.y = 0.0;
            state.time = previous_time + fraction * (state.time - previous_time);
            state.vx = previous_vx + fraction * (state.vx - previous_vx);
            state.vy = previous_vy + fraction * (state.vy - previous_vy);
            landed = true;
        }

        if (csv_file != NULL) {
            fprintf(csv_file, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    state.time, state.x, state.y, state.vx, state.vy, state.ax, state.ay);
        }

        if (config->verbose || step_count % 10 == 0 || landed) {
            printf("%-10.4f %-12.3f %-12.3f %-12.3f %-12.3f",
                   state.time, state.x, state.y, state.vx, state.vy);
            if (landed) {
                printf(" %-12s", "LANDED");
            }
            printf("\n");
        }

        if (config->show_animation && !landed) {
            int pos = (int)fmin(59.0, fmax(0.0, state.x / fmax(1.0, analytic_range) * 59.0));
            printf("|");
            for (int i = 0; i < 60; i++) {
                putchar(i == pos ? '*' : '-');
            }
            printf("|\n");
        }
    }

    if (!landed) {
        printf("Error: projectile did not land within the maximum allowed steps. Try a larger time step.\n");
        if (csv_file != NULL) {
            fclose(csv_file);
        }
        return 1;
    }

    if (csv_file != NULL) {
        fclose(csv_file);
    }

    printf("\n==================== SIMULATION SUMMARY ====================\n");
    printf("Time of flight:     %.4f s\n", state.time);
    printf("Range:              %.4f m\n", state.x);
    printf("Maximum height:     %.4f m\n", max_y);
    printf("Impact velocity:    %.4f m/s\n", hypot(state.vx, state.vy));
    printf("Impact angle:       %.4f degrees\n", atan2(fabs(state.vy), fabs(state.vx)) * 180.0 / PI);
    printf("Steps:              %d\n", step_count);

    if (!config->enable_drag) {
        printf("Analytical time:    %.4f s\n", analytic_time);
        printf("Analytical range:   %.4f m\n", analytic_range);
        printf("Analytical max h:   %.4f m\n", analytic_max_height);
    } else if (analytic_range > 0.0) {
        double range_reduction = (analytic_range - state.x) / analytic_range * 100.0;
        printf("Range reduction:    %.2f%% compared with no-drag theory\n", range_reduction);
    }
    printf("============================================================\n");

    if (config->export_csv) {
        printf("CSV data exported to: %s\n", strlen(config->csv_filename) > 0 ? config->csv_filename : DEFAULT_OUTPUT_FILE);
    }

    return 0;
}

static void run_multiple_simulations(void) {
    const double velocities[] = {10.0, 20.0, 30.0, 40.0, 50.0};
    const double angles[] = {30.0, 45.0, 60.0};
    const size_t velocity_count = sizeof(velocities) / sizeof(velocities[0]);
    const size_t angle_count = sizeof(angles) / sizeof(angles[0]);

    printf("\nMulti-Scenario Comparison - No Drag Analytical Results\n");
    printf("%-10s %-10s %-12s %-12s %-12s\n", "V0(m/s)", "Angle", "Range(m)", "MaxH(m)", "Time(s)");
    printf("--------------------------------------------------------------\n");

    for (size_t i = 0; i < velocity_count; i++) {
        for (size_t j = 0; j < angle_count; j++) {
            const double v0 = velocities[i];
            const double angle = angles[j];
            const double theta = angle * PI / 180.0;
            const double time = 2.0 * v0 * sin(theta) / G;
            const double range = v0 * cos(theta) * time;
            const double max_height = pow(v0 * sin(theta), 2.0) / (2.0 * G);

            printf("%-10.0f %-10.0f %-12.3f %-12.3f %-12.3f\n",
                   v0, angle, range, max_height, time);
        }
    }
}

int main(void) {
    double velocity = 0.0;
    double angle = 0.0;
    double time_step = 0.0;

    sim_config_t config = {
        .enable_drag = false,
        .drag_coefficient = 0.47,
        .cross_section = 0.01,
        .mass = 1.0,
        .show_animation = false,
        .export_csv = false,
        .verbose = true,
        .csv_filename = DEFAULT_OUTPUT_FILE
    };

    printf("============================================================\n");
    printf("              PROJECTILE MOTION SIMULATOR                   \n");
    printf("============================================================\n\n");

    if (read_yes_no_retry("Enable advanced simulation options?")) {
        config.enable_drag = read_yes_no_retry("Include air resistance?");
        if (config.enable_drag) {
            if (!read_double_retry("Drag coefficient, e.g. 0.47 for sphere: ", &config.drag_coefficient, 0.0, 2.0)) {
                return EXIT_FAILURE;
            }
            if (!read_double_retry("Cross-sectional area in m^2: ", &config.cross_section, 0.0001, 10.0)) {
                return EXIT_FAILURE;
            }
            if (!read_double_retry("Mass in kg: ", &config.mass, 0.001, 1000.0)) {
                return EXIT_FAILURE;
            }
        }
        config.show_animation = read_yes_no_retry("Show simple text trajectory?");
        config.export_csv = read_yes_no_retry("Export all time steps to CSV?");
        config.verbose = read_yes_no_retry("Show detailed output?");
    }

    printf("\nSimulation Parameters\n");
    if (!read_double_retry("Initial velocity in m/s: ", &velocity, MIN_VELOCITY, MAX_VELOCITY)) {
        return EXIT_FAILURE;
    }
    if (!read_double_retry("Launch angle in degrees: ", &angle, MIN_ANGLE, MAX_ANGLE)) {
        return EXIT_FAILURE;
    }
    if (!read_double_retry("Time step in seconds: ", &time_step, MIN_DT, MAX_DT)) {
        return EXIT_FAILURE;
    }

    if (simulate_projectile(velocity, angle, time_step, &config) != 0) {
        return EXIT_FAILURE;
    }

    if (read_yes_no_retry("\nRun multi-scenario comparison?")) {
        run_multiple_simulations();
    }

    printf("\nSimulation complete.\n");
    return EXIT_SUCCESS;
}
