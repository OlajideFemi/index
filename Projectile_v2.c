#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define G 9.81          // Acceleration due to gravity (m/s^2)
#define RHO 1.225       // Air density at sea level (kg/m^3)
#define PI 3.14159265359

// ---- Configuration ----
typedef struct {
    bool enable_drag;           // Include air resistance
    double drag_coefficient;    // Drag coefficient (0.47 for sphere)
    double cross_section;       // Cross-sectional area (m^2)
    double mass;               // Mass (kg)
    bool show_animation;       // Show simple text animation
    bool export_csv;          // Export to CSV file
    bool verbose;             // Show detailed output
} sim_config_t;

// ---- Projectile state ----
typedef struct {
    double x, y;        // Position (m)
    double vx, vy;      // Velocity (m/s)
    double ax, ay;      // Acceleration (m/s^2)
    double time;        // Current time (s)
} projectile_state_t;

// ---- Input helpers ----
int read_double(const char *prompt, double *out, double min, double max) {
    char buffer[100];
    printf("%s", prompt);
    if (fgets(buffer, sizeof(buffer), stdin) == NULL) {
        printf("Error reading input.\n");
        return 0;
    }
    if (sscanf(buffer, "%lf", out) != 1) {
        printf("Invalid input: please enter a numeric value.\n");
        return 0;
    }
    if ((min != max) && (*out < min || *out > max)) {
        printf("Value out of range (%.2f - %.2f)\n", min, max);
        return 0;
    }
    return 1;
}

int read_string(const char *prompt, char *out, size_t size) {
    printf("%s", prompt);
    if (fgets(out, size, stdin) == NULL) {
        printf("Error reading input.\n");
        return 0;
    }
    // Remove newline
    size_t len = strlen(out);
    if (len > 0 && out[len-1] == '\n')
        out[len-1] = '\0';
    return 1;
}

int read_yes_no(const char *prompt) {
    char response[10];
    printf("%s (y/n): ", prompt);
    if (fgets(response, sizeof(response), stdin) == NULL)
        return 0;
    return (response[0] == 'y' || response[0] == 'Y');
}

// ---- Physics engine ----
void update_with_drag(projectile_state_t *state, double dt, const sim_config_t *config) {
    // Calculate drag force: Fd = 0.5 * rho * v^2 * Cd * A
    double v = sqrt(state->vx * state->vx + state->vy * state->vy);
    double drag_factor = 0.5 * RHO * config->drag_coefficient * config->cross_section / config->mass;
    
    if (v > 0.001) {
        double drag_accel = drag_factor * v;
        state->ax = -drag_accel * (state->vx / v);
        state->ay = -G - drag_accel * (state->vy / v);
    } else {
        state->ax = 0;
        state->ay = -G;
    }
    
    // Update velocity (semi-implicit Euler)
    state->vx += state->ax * dt;
    state->vy += state->ay * dt;
    
    // Update position
    state->x += state->vx * dt;
    state->y += state->vy * dt;
    state->time += dt;
}

void update_no_drag(projectile_state_t *state, double dt) {
    state->vx = state->vx;  // Constant
    state->vy -= G * dt;
    state->x += state->vx * dt;
    state->y += state->vy * dt;
    state->time += dt;
}

// ---- Simulation ----
int simulateProjectile(double v0, double angle, double dt, const sim_config_t *config) {
    // Validate inputs
    if (v0 <= 0.0) {
        printf("Error: initial velocity must be positive.\n");
        return 1;
    }
    if (angle <= 0.0 || angle >= 90.0) {
        printf("Error: launch angle must be between 0 and 90 degrees (exclusive).\n");
        return 1;
    }
    if (dt <= 0.0) {
        printf("Error: time step must be positive.\n");
        return 1;
    }
    
    double theta = angle * PI / 180.0;
    projectile_state_t state = {
        .x = 0,
        .y = 0,
        .vx = v0 * cos(theta),
        .vy = v0 * sin(theta),
        .time = 0
    };
    
    // Analytical landing time (for reference)
    double t_land_analytic = 2.0 * state.vy / G;
    double t_land = t_land_analytic;
    double max_y = 0;
    double max_x = 0;
    int step_count = 0;
    
    // Open CSV file if requested
    FILE *csv_file = NULL;
    if (config->export_csv) {
        csv_file = fopen("projectile_data.csv", "w");
        if (csv_file) {
            fprintf(csv_file, "Time(s),X(m),Y(m),Vx(m/s),Vy(m/s)\n");
        }
    }
    
    // Print header
    printf("\n");
    if (config->verbose) {
        printf("┌─────────────── Projectile Motion Simulation ───────────────┐\n");
        printf("│ Initial Velocity: %.2f m/s                                │\n", v0);
        printf("│ Launch Angle:     %.2f degrees                           │\n", angle);
        printf("│ Time Step:        %.3f s                                 │\n", dt);
        if (config->enable_drag) {
            printf("│ Drag Coefficient: %.2f                                 │\n", config->drag_coefficient);
            printf("│ Cross Section:    %.4f m²                              │\n", config->cross_section);
            printf("│ Mass:             %.2f kg                              │\n", config->mass);
        }
        printf("└────────────────────────────────────────────────────────────┘\n\n");
    }
    
    if (config->verbose) {
        printf("%-8s %-10s %-10s %-10s %-10s %-10s\n", 
               "Time(s)", "X(m)", "Y(m)", "Vx(m/s)", "Vy(m/s)", "Status");
        printf("----------------------------------------------------------------------------\n");
    }
    
    // Simulation loop
    bool landed = false;
    double prev_y = state.y;
    
    while (!landed && state.time <= t_land_analytic * 2.0) {
        // Update physics
        if (config->enable_drag) {
            update_with_drag(&state, dt, config);
        } else {
            update_no_drag(&state, dt);
        }
        
        // Check if projectile has landed
        if (state.y < 0) {
            // Interpolate to find exact landing point
            double fraction = prev_y / (prev_y - state.y);
            state.x -= state.vx * dt * (1 - fraction);
            state.y = 0;
            state.time -= dt * (1 - fraction);
            landed = true;
        }
        
        // Update max values
        if (state.y > max_y) max_y = state.y;
        if (state.x > max_x) max_x = state.x;
        
        // Print step
        if (config->verbose || step_count % 10 == 0 || landed) {
            printf("%-8.3f %-10.2f %-10.2f %-10.2f %-10.2f", 
                   state.time, state.x, state.y, state.vx, state.vy);
            if (landed) {
                printf(" %-10s", "← LANDED");
            }
            printf("\n");
            
            // Write to CSV
            if (csv_file) {
                fprintf(csv_file, "%.3f,%.2f,%.2f,%.2f,%.2f\n", 
                        state.time, state.x, state.y, state.vx, state.vy);
            }
            
            // Simple text animation
            if (config->show_animation && !landed) {
                int pos = (int)(state.x / 2);
                int height = (int)(state.y / 2);
                printf("\r");
                for (int i = 0; i < 60; i++) {
                    if (i == pos && height == 0) printf("*");
                    else if (i == pos) printf(" ");
                    else printf("-");
                }
                fflush(stdout);
            }
        }
        
        if (landed) break;
        prev_y = state.y;
        step_count++;
    }
    
    // Calculate max height and time
    double t_apex = state.vy / G;
    if (config->enable_drag) {
        // Numerical approximation for apex with drag
        t_apex = (state.vy + sqrt(state.vy * state.vy + 2 * G * max_y)) / G;
    }
    
    // Print summary
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("                    SIMULATION SUMMARY                        \n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  Time of Flight:    %.3f s\n", state.time);
    printf("  Range:             %.2f m\n", state.x);
    printf("  Maximum Height:    %.2f m\n", max_y);
    printf("  Impact Velocity:   %.2f m/s\n", sqrt(state.vx * state.vx + state.vy * state.vy));
    printf("  Impact Angle:      %.2f degrees\n", 
           atan2(fabs(state.vy), state.vx) * 180.0 / PI);
    printf("  Steps:             %d\n", step_count);
    
    if (config->enable_drag) {
        double range_reduction = (t_land_analytic * v0 * cos(theta) - state.x) / 
                                 (t_land_analytic * v0 * cos(theta)) * 100;
        printf("  Range Reduction:   %.1f%% (due to air resistance)\n", range_reduction);
    }
    printf("═══════════════════════════════════════════════════════════════\n");
    
    // Close CSV file
    if (csv_file) {
        fclose(csv_file);
        printf("\n✓ Data exported to 'projectile_data.csv'\n");
    }
    
    return 0;
}

// ---- Multiple simulations ----
void run_multiple_simulations(const sim_config_t *config) {
    double velocities[] = {10, 20, 30, 40, 50};
    double angles[] = {30, 45, 60};
    
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║           MULTI-SCENARIO COMPARISON                          ║\n");
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    printf("║ V0(m/s)  Angle(°)  Range(m)  MaxH(m)  Time(s)              ║\n");
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    
    for (int i = 0; i < sizeof(velocities)/sizeof(velocities[0]); i++) {
        for (int j = 0; j < sizeof(angles)/sizeof(angles[0]); j++) {
            double v0 = velocities[i];
            double angle = angles[j];
            double theta = angle * PI / 180.0;
            double t_land = 2.0 * v0 * sin(theta) / G;
            double range = v0 * cos(theta) * t_land;
            double max_h = (v0 * sin(theta) * v0 * sin(theta)) / (2 * G);
            
            printf("║ %-6.0f   %-7.0f   %-8.2f %-8.2f %-8.2f ║\n", 
                   v0, angle, range, max_h, t_land);
        }
    }
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
}

// ---- Main program ----
int main(void) {
    double velocity, angle, timeStep;
    sim_config_t config = {
        .enable_drag = false,
        .drag_coefficient = 0.47,
        .cross_section = 0.01,  // 0.01 m²
        .mass = 1.0,
        .show_animation = false,
        .export_csv = false,
        .verbose = true
    };
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("          PROJECTILE MOTION SIMULATOR v3.0                    \n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    // Check if user wants advanced options
    bool advanced = read_yes_no("Enable advanced simulation options?");
    
    if (advanced) {
        config.enable_drag = read_yes_no("Include air resistance?");
        if (config.enable_drag) {
            read_double("Drag coefficient (0.47 for sphere): ", 
                       &config.drag_coefficient, 0, 2);
            read_double("Cross-sectional area (m²): ", 
                       &config.cross_section, 0.0001, 10);
            read_double("Mass (kg): ", &config.mass, 0.001, 1000);
        }
        config.show_animation = read_yes_no("Show animated trajectory?");
        config.export_csv = read_yes_no("Export data to CSV?");
        config.verbose = read_yes_no("Show detailed output?");
    }
    
    // Get simulation parameters
    printf("\n--- Simulation Parameters ---\n");
    if (!read_double("Initial velocity (m/s): ", &velocity, 0.1, 1000))
        return 1;
    if (!read_double("Launch angle (degrees, 0-90): ", &angle, 0.1, 89.9))
        return 1;
    if (!read_double("Time step (seconds): ", &timeStep, 0.001, 10))
        return 1;
    
    // Run simulation
    if (simulateProjectile(velocity, angle, timeStep, &config) != 0)
        return 1;
    
    // Multi-scenario comparison
    if (read_yes_no("\nRun multi-scenario comparison?")) {
        run_multiple_simulations(&config);
    }
    
    printf("\n✓ Simulation complete.\n");
    return 0;
}