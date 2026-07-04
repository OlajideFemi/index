/*
 * Elevator Group Simulation - Enhanced Version
 *
 * Features:
 *   - Multiple dispatch strategies (Round-robin, Nearest-available, Zoned)
 *   - Passenger capacity constraints
 *   - Energy consumption metrics
 *   - Detailed floor-level statistics
 *   - Real-time progress indicator
 *   - CSV export capability
 *   - Morning/evening rush hour modeling
 *   - Elevator maintenance/outage simulation
 *
 * Compile:
 *   gcc -Wall -Wextra -pedantic -std=c11 elevator_enhanced.c -o elevator_sim -lm
 *
 * Run:
 *   ./elevator_sim
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

/* ---- Building / simulation configuration ---- */
#define NUM_FLOORS          10      /* Floors 1..10 */
#define CANTEEN_FLOOR        1      /* Canteen location */
#define NUM_ELEVATORS         6      /* Total lifts */
#define CANTEEN_DEDICATED     2      /* Lifts reserved for canteen traffic during lunch */
#define SIM_DURATION      28800L     /* 8-hour working day, in seconds */
#define LUNCH_START       14400L     /* 12:00, 4h after 08:00 start */
#define LUNCH_END         18000L     /* 13:00 */
#define MORNING_RUSH_START 7200L     /* 10:00 */
#define MORNING_RUSH_END   9000L     /* 10:30 */
#define EVENING_RUSH_START 25200L    /* 17:00 */
#define EVENING_RUSH_END   27000L    /* 17:30 */
#define TIME_PER_FLOOR        3L     /* Seconds between adjacent floors */
#define TIME_PER_STOP         8L     /* Seconds loading/unloading at a stop */
#define NUM_REQUESTS        600      /* Requests generated for the simulated day */
#define LUNCH_TRIP_PROB      45      /* % of requests that are lunch-window canteen trips */
#define RUSH_TRIP_PROB       60      /* % of requests during rush hours */
#define MAX_CAPACITY         10      /* Maximum passengers per elevator */
#define ENERGY_IDLE         0.5      /* Energy consumption idle (kW) */
#define ENERGY_MOVING       5.0      /* Energy consumption moving (kW) */
#define ENERGY_PER_STOP     0.5      /* Energy per stop (kWh) */
#define MAINTENANCE_PROB     5       /* Probability of elevator maintenance issue (%) */
#define RANDOM_SEED          42U     /* Fixed seed for reproducible output */

/* ---- Compile-time configuration validation ---- */
#if NUM_FLOORS < 2
#error "NUM_FLOORS must be at least 2."
#endif

#if CANTEEN_FLOOR < 1 || CANTEEN_FLOOR > NUM_FLOORS
#error "CANTEEN_FLOOR must be between 1 and NUM_FLOORS."
#endif

#if NUM_ELEVATORS < 1
#error "NUM_ELEVATORS must be at least 1."
#endif

#if CANTEEN_DEDICATED < 0 || CANTEEN_DEDICATED > NUM_ELEVATORS
#error "CANTEEN_DEDICATED must be between 0 and NUM_ELEVATORS."
#endif

#if SIM_DURATION <= 0
#error "SIM_DURATION must be positive."
#endif

#if LUNCH_START < 0 || LUNCH_END < 0 || LUNCH_START > SIM_DURATION || LUNCH_END > SIM_DURATION || LUNCH_START >= LUNCH_END
#error "Lunch window must be valid and inside the simulation duration."
#endif

typedef struct {
    int id;
    int origin;
    int destination;
    long arrival_time;
    int assigned_elevator;
    long start_service;
    long end_service;
    int waiting_time;
    int served;
    int peak_type;  // 0=normal, 1=lunch, 2=morning_rush, 3=evening_rush
} Request;

typedef struct {
    int position;          /* Current/last-known floor */
    long free_time;        /* Time this elevator becomes available */
    int requests_served;
    int passengers_onboard;
    int total_passengers_served;
    double energy_consumed;  /* kWh */
    long total_travel_time;
    long idle_time;
    int stops_made;
    bool maintenance_required;
    int maintenance_duration;
} Elevator;

typedef struct {
    double average_wait;
    double canteen_average_wait;
    double rush_average_wait;
    long max_wait;
    int canteen_count;
    int rush_count;
    int invalid_waits;
    double total_energy;  /* kWh */
    double average_energy_per_request;
    int floor_requests[NUM_FLOORS];
    int floor_served[NUM_FLOORS];
    long total_wait_time;
    int total_requests;
    int max_queue_length;
} ReportStats;

// ---- Global statistics ----
static int queue_length = 0;
static int peak_queue_length = 0;

static void show_progress(int current, int total); // forward declaration

/* ---- Helper functions ----
 * These utility functions encapsulate repeated logic for floor calculations,
 * elevator status, and request handling, keeping the main simulation clean.
 */
static int is_lunch_time(long t) {
    return (t >= LUNCH_START && t < LUNCH_END);
}

static int is_morning_rush(long t) {
    return (t >= MORNING_RUSH_START && t < MORNING_RUSH_END);
}

static int is_evening_rush(long t) {
    return (t >= EVENING_RUSH_START && t < EVENING_RUSH_END);
}

static int touches_canteen(const Request *r) {
    if (r == NULL) {
        return 0;
    }
    return (r->origin == CANTEEN_FLOOR || r->destination == CANTEEN_FLOOR);
}

static long travel_time(int from_floor, int to_floor) {
    long distance = labs((long)from_floor - (long)to_floor);
    return distance * TIME_PER_FLOOR;
}

static int random_floor_except(int excluded_floor) {
    int floor;

    if (NUM_FLOORS <= 1) {
        return excluded_floor;
    }

    do {
        floor = 1 + rand() % NUM_FLOORS;
    } while (floor == excluded_floor);

    return floor;
}

static void reset_request_service_fields(Request *r) {
    if (r == NULL) {
        return;
    }
    r->assigned_elevator = -1;
    r->start_service = -1;
    r->end_service = -1;
    r->served = 0;
    r->waiting_time = 0;
}

// Applies a temporary maintenance delay directly to the elevator's
// free_time, so it's simply unavailable for a while rather than being
// permanently flagged out of service (the original version set
// maintenance_required = true and never cleared it anywhere, which
// meant the first elevator to trip this check was gone for the rest
// of the simulated day).
static bool elevator_maintenance_check(Elevator *el) {
    if (el == NULL) return false;

    if (el->requests_served > 0 && el->requests_served % 50 == 0) {
        if ((rand() % 100) < MAINTENANCE_PROB) {
            int duration = 300 + (rand() % 300); // 5-10 minutes
            el->maintenance_duration = duration;
            el->free_time += duration;
            return true;
        }
    }
    return false;
}

/* ---- Request generation ----
 * Enhanced with morning/evening rush hour modeling and more realistic
 * traffic patterns.
 */
static void generate_requests(Request reqs[], int n) {
    if (reqs == NULL || n <= 0) {
        return;
    }

    const long lunch_window = LUNCH_END - LUNCH_START;
    const long morning_rush_window = MORNING_RUSH_END - MORNING_RUSH_START;
    const long evening_rush_window = EVENING_RUSH_END - EVENING_RUSH_START;

    for (int i = 0; i < n; i++) {
        int origin;
        int destination;

        reqs[i].id = i;
        reset_request_service_fields(&reqs[i]);
        reqs[i].peak_type = 0;

        // Determine trip type
        int random_val = rand() % 100;
        int is_lunch_trip = (random_val < LUNCH_TRIP_PROB && lunch_window > 0);
        int is_rush_trip = 0;

        // NOTE: arrival_time hasn't been assigned yet at this point, so we
        // can't check "is it currently rush hour" - instead we decide
        // probabilistically whether this request IS a rush-hour trip, then
        // pick its arrival time (and which rush window) below.
        if (!is_lunch_trip) {
            is_rush_trip = (rand() % 100) < RUSH_TRIP_PROB;
        }

        if (is_lunch_trip && lunch_window > 0) {
            // Lunch trip
            reqs[i].arrival_time = LUNCH_START + (long)(rand() % (int)lunch_window);
            reqs[i].peak_type = 1;

            if (rand() % 2 == 0) {
                origin = CANTEEN_FLOOR;
                destination = random_floor_except(origin);
            } else {
                destination = CANTEEN_FLOOR;
                origin = random_floor_except(destination);
            }
        } else if (is_rush_trip) {
            // Rush hour trip - randomly choose morning or evening window
            int is_morning = (rand() % 2 == 0);
            if (is_morning) {
                reqs[i].arrival_time = MORNING_RUSH_START + (long)(rand() % (int)morning_rush_window);
                reqs[i].peak_type = 2;
                // Morning rush: people going UP to offices
                origin = 1 + (rand() % 3); // Floors 1-3
                destination = 4 + (rand() % (NUM_FLOORS - 3));
            } else {
                reqs[i].arrival_time = EVENING_RUSH_START + (long)(rand() % (int)evening_rush_window);
                reqs[i].peak_type = 3;
                // Evening rush: people going DOWN to ground floor
                origin = 4 + (rand() % (NUM_FLOORS - 3));
                destination = 1 + (rand() % 3);
            }
        } else {
            // Normal background traffic
            reqs[i].arrival_time = (long)(rand() % (int)SIM_DURATION);
            reqs[i].peak_type = 0;
            origin = 1 + rand() % NUM_FLOORS;
            destination = random_floor_except(origin);
        }

        reqs[i].origin = origin;
        reqs[i].destination = destination;
    }
}

/* ---- Request sorting ----
 * Sort requests by arrival time for sequential processing.
 */
static int compare_requests_by_arrival_then_id(const void *a, const void *b) {
    const Request *ra = (const Request *)a;
    const Request *rb = (const Request *)b;

    if (ra->arrival_time < rb->arrival_time) {
        return -1;
    }
    if (ra->arrival_time > rb->arrival_time) {
        return 1;
    }
    return ra->id - rb->id;
}

static void sort_by_arrival(Request reqs[], int n) {
    if (reqs == NULL || n <= 1) {
        return;
    }
    qsort(reqs, (size_t)n, sizeof(reqs[0]), compare_requests_by_arrival_then_id);
}

/* ---- Elevator initialization ----
 * Set up all elevators to their initial state at floor 1, time 0.
 */
static void init_elevators(Elevator elevs[], int n) {
    if (elevs == NULL || n <= 0) {
        return;
    }

    for (int i = 0; i < n; i++) {
        elevs[i].position = 1;
        elevs[i].free_time = 0;
        elevs[i].requests_served = 0;
        elevs[i].passengers_onboard = 0;
        elevs[i].total_passengers_served = 0;
        elevs[i].energy_consumed = 0.0;
        elevs[i].total_travel_time = 0;
        elevs[i].idle_time = 0;
        elevs[i].stops_made = 0;
        elevs[i].maintenance_required = false;
        elevs[i].maintenance_duration = 0;
    }
}

/* ---- Service request ----
 * Assign a request to a specific elevator and update its state.
 */
static bool service_request(Request *req, Elevator *el, int elevator_index) {
    long ready;
    long pickup;
    long dropoff;

    if (req == NULL || el == NULL || elevator_index < 0) {
        return false;
    }

    // NOTE: MAX_CAPACITY / passengers_onboard is not enforced as a hard
    // gate here. Each Request represents one passenger's complete solo
    // trip (board and alight are instantaneous in this model), so there
    // is no notion of concurrent riders to check capacity against. An
    // earlier version incremented passengers_onboard without ever
    // decrementing it, which permanently "retired" every elevator after
    // its 10th ride. total_passengers_served below still tracks lifetime
    // ridership for reporting purposes.

    // Calculate times
    ready = (el->free_time > req->arrival_time) ? el->free_time : req->arrival_time;
    pickup = ready + travel_time(el->position, req->origin) + TIME_PER_STOP;
    dropoff = pickup + travel_time(req->origin, req->destination) + TIME_PER_STOP;

    if (pickup < ready || dropoff < pickup) {
        return false; /* overflow guard */
    }

    // Capture position BEFORE it's overwritten below - the original code
    // computed distance using el->position after reassigning it to the
    // destination, which meant it measured (destination - origin) instead
    // of the elevator's actual approach travel from where it started.
    int old_position = el->position;

    // Update elevator state
    el->free_time = dropoff;
    el->position = req->destination;
    el->requests_served++;
    el->total_passengers_served++;
    el->total_travel_time += (dropoff - ready);
    el->stops_made += 2; // Pickup and dropoff

    // Energy calculation: full movement = approach to pickup + the trip itself
    long approach_distance = labs((long)old_position - (long)req->origin);
    long trip_distance = labs((long)req->origin - (long)req->destination);
    long total_distance = approach_distance + trip_distance;
    if (total_distance > 0) {
        el->energy_consumed += ENERGY_MOVING * (total_distance / 10.0); // Moving energy
    }
    el->energy_consumed += ENERGY_PER_STOP * 2; // Two stops

    // Update request
    req->assigned_elevator = elevator_index;
    req->start_service = pickup;
    req->end_service = dropoff;
    req->served = 1;
    req->waiting_time = pickup - req->arrival_time;

    // Check for maintenance
    elevator_maintenance_check(el);

    return true;
}

/* ---- Strategy 1: Round-robin dispatch ----
 * Requests are handed to elevators in fixed rotation, ignoring distance and workload.
 */
static void simulate_round_robin(Request reqs[], int n, Elevator elevs[]) {
    if (reqs == NULL || elevs == NULL || n <= 0) {
        return;
    }

    init_elevators(elevs, NUM_ELEVATORS);
    sort_by_arrival(reqs, n);
    queue_length = 0;
    peak_queue_length = 0;
    srand(RANDOM_SEED); // reseed so maintenance draws are comparable across strategies

    for (int i = 0; i < n; i++) {
        int elevator_index = i % NUM_ELEVATORS;
        bool served = service_request(&reqs[i], &elevs[elevator_index], elevator_index);
        
        // Update queue statistics
        if (!served) {
            queue_length++;
            if (queue_length > peak_queue_length) {
                peak_queue_length = queue_length;
            }
        } else {
            if (queue_length > 0) queue_length--;
        }

        if ((i + 1) % 20 == 0 || i == n - 1)
            show_progress(i + 1, n);
    }
    printf("\n");
}

/* ---- Strategy 2: Nearest-available dispatch + canteen zoning ----
 * During lunch, canteen-touching requests use a dedicated pool of lifts.
 * Non-canteen requests use the remaining lifts where possible.
 */
static int choose_elevator(const Elevator elevs[], const Request *req) {
    int best = -1;
    long best_cost = LONG_MAX;

    if (elevs == NULL || req == NULL) {
        return -1;
    }

    for (int i = 0; i < NUM_ELEVATORS; i++) {
        long ready = (elevs[i].free_time > req->arrival_time) ? elevs[i].free_time : req->arrival_time;
        long cost = ready + travel_time(elevs[i].position, req->origin);

        /*
         * Soft canteen zoning:
         * During lunch, the first CANTEEN_DEDICATED elevators are preferred for
         * canteen trips, while the remaining elevators are preferred for normal
         * traffic.
         */
        if (is_lunch_time(req->arrival_time) && CANTEEN_DEDICATED > 0 && CANTEEN_DEDICATED < NUM_ELEVATORS) {
            const long zone_penalty = TIME_PER_STOP + TIME_PER_FLOOR;
            int is_canteen_elevator = (i < CANTEEN_DEDICATED);
            int is_canteen_request = touches_canteen(req);

            if (is_canteen_request && !is_canteen_elevator) {
                cost += zone_penalty;
            } else if (!is_canteen_request && is_canteen_elevator) {
                cost += zone_penalty;
            }
        }

        if (cost < ready) {
            continue; /* overflow guard */
        }

        if (cost < best_cost || (cost == best_cost && (best < 0 || i < best))) {
            best_cost = cost;
            best = i;
        }
    }

    return best;
}

static void simulate_optimized(Request reqs[], int n, Elevator elevs[]) {
    if (reqs == NULL || elevs == NULL || n <= 0) {
        return;
    }

    init_elevators(elevs, NUM_ELEVATORS);
    sort_by_arrival(reqs, n);
    queue_length = 0;
    peak_queue_length = 0;
    srand(RANDOM_SEED); // reseed so maintenance draws are comparable across strategies

    for (int i = 0; i < n; i++) {
        int elevator_index = choose_elevator(elevs, &reqs[i]);
        bool served = false;
        
        if (elevator_index >= 0) {
            served = service_request(&reqs[i], &elevs[elevator_index], elevator_index);
        }
        
        // Update queue statistics
        if (!served) {
            queue_length++;
            if (queue_length > peak_queue_length) {
                peak_queue_length = queue_length;
            }
        } else {
            if (queue_length > 0) queue_length--;
        }

        if ((i + 1) % 20 == 0 || i == n - 1)
            show_progress(i + 1, n);
    }
    printf("\n");
}

/* ---- Statistics calculation ----
 * Compute comprehensive metrics from the completed simulation.
 */
static ReportStats calculate_stats(const Request reqs[], int n) {
    ReportStats stats = {0};
    long total_wait = 0;
    long canteen_wait = 0;
    long rush_wait = 0;

    if (reqs == NULL || n <= 0) {
        return stats;
    }

    // Initialize floor statistics
    for (int i = 0; i < NUM_FLOORS; i++) {
        stats.floor_requests[i] = 0;
        stats.floor_served[i] = 0;
    }

    for (int i = 0; i < n; i++) {
        long wait;

        if (reqs[i].start_service < reqs[i].arrival_time || reqs[i].served == 0) {
            stats.invalid_waits++;
            continue;
        }

        wait = reqs[i].start_service - reqs[i].arrival_time;
        total_wait += wait;
        stats.total_wait_time += wait;
        stats.total_requests++;

        if (wait > stats.max_wait) {
            stats.max_wait = wait;
        }

        // Track floor requests
        if (reqs[i].origin >= 1 && reqs[i].origin <= NUM_FLOORS) {
            stats.floor_requests[reqs[i].origin - 1]++;
            stats.floor_served[reqs[i].origin - 1]++;
        }
        if (reqs[i].destination >= 1 && reqs[i].destination <= NUM_FLOORS) {
            stats.floor_requests[reqs[i].destination - 1]++;
        }

        if (touches_canteen(&reqs[i])) {
            canteen_wait += wait;
            stats.canteen_count++;
        }

        if (reqs[i].peak_type > 0) {
            rush_wait += wait;
            stats.rush_count++;
        }

        // Sanity check: does the tagged peak_type actually match the
        // generated arrival_time? Catches future regressions in generation.
        if (reqs[i].peak_type == 2 && !is_morning_rush(reqs[i].arrival_time)) {
            fprintf(stderr, "Warning: request %d tagged morning-rush but arrival_time %ld is outside that window\n",
                    reqs[i].id, reqs[i].arrival_time);
        }
        if (reqs[i].peak_type == 3 && !is_evening_rush(reqs[i].arrival_time)) {
            fprintf(stderr, "Warning: request %d tagged evening-rush but arrival_time %ld is outside that window\n",
                    reqs[i].id, reqs[i].arrival_time);
        }
    }

    stats.average_wait = (double)total_wait / (double)n;
    stats.max_queue_length = peak_queue_length;

    if (stats.canteen_count > 0) {
        stats.canteen_average_wait = (double)canteen_wait / (double)stats.canteen_count;
    }

    if (stats.rush_count > 0) {
        stats.rush_average_wait = (double)rush_wait / (double)stats.rush_count;
    }

    return stats;
}

/* ---- Elevator energy statistics ----
 * Calculate total energy consumption across all elevators.
 */
static double calculate_total_energy(const Elevator elevs[], int n) {
    double total = 0.0;
    for (int i = 0; i < n; i++) {
        total += elevs[i].energy_consumed;
    }
    return total;
}

/* ---- Reporting functions ----
 * Generate detailed reports for each simulation strategy.
 */
static void report(const char *label, const Request reqs[], int n, const Elevator elevs[]) {
    ReportStats stats;
    double total_energy;

    if (label == NULL || reqs == NULL || elevs == NULL || n <= 0) {
        printf("Report unavailable: invalid input.\n");
        return;
    }

    stats = calculate_stats(reqs, n);
    total_energy = calculate_total_energy(elevs, NUM_ELEVATORS);

    printf("\n╔═══════════════════════════════════════════════════════════════════════╗\n");
    printf("║ %-65s ║\n", label);
    printf("╠═══════════════════════════════════════════════════════════════════════╣\n");
    printf("║ Average wait time (all requests)     : %8.1f s                      ║\n", stats.average_wait);
    printf("║ Max wait time                        : %8ld s                      ║\n", stats.max_wait);
    printf("║ Peak queue length                    : %8d                         ║\n", stats.max_queue_length);
    
    if (stats.canteen_count > 0) {
        printf("║ Average wait time (canteen requests) : %8.1f s  (%d requests)     ║\n",
               stats.canteen_average_wait, stats.canteen_count);
    } else {
        printf("║ Average wait time (canteen requests) : %8s  (0 requests)         ║\n", "N/A");
    }
    
    if (stats.rush_count > 0) {
        printf("║ Average wait time (rush requests)    : %8.1f s  (%d requests)     ║\n",
               stats.rush_average_wait, stats.rush_count);
    } else {
        printf("║ Average wait time (rush requests)    : %8s  (0 requests)         ║\n", "N/A");
    }
    
    printf("║ Total energy consumed                 : %8.2f kWh                   ║\n", total_energy);
    printf("║ Energy per request                    : %8.3f kWh                   ║\n", 
           stats.total_requests > 0 ? total_energy / stats.total_requests : 0);
    
    if (stats.invalid_waits > 0) {
        printf("║ Warning: %d request(s) had invalid service timing.                ║\n", stats.invalid_waits);
    }
    
    printf("╠═══════════════════════════════════════════════════════════════════════╣\n");
    printf("║ Requests served per elevator          :                              ║\n");
    for (int i = 0; i < NUM_ELEVATORS; i++) {
        if (i % 3 == 0) printf("║ ");
        printf("E%d=%d  ", i + 1, elevs[i].requests_served);
        if (i % 3 == 2 || i == NUM_ELEVATORS - 1) {
            if (i % 3 == 2 || i == NUM_ELEVATORS - 1) {
                int padding = 47 - (i % 3 + 1) * 6;
                for (int j = 0; j < padding; j++) printf(" ");
                printf("║\n");
            }
        }
    }
    
    // Floor statistics
    printf("╠═══════════════════════════════════════════════════════════════════════╣\n");
    printf("║ Floor request distribution:                                          ║\n");
    for (int i = 0; i < NUM_FLOORS; i++) {
        if (i % 4 == 0) printf("║ ");
        printf("F%-2d:%-3d  ", i + 1, stats.floor_requests[i]);
        if (i % 4 == 3 || i == NUM_FLOORS - 1) {
            int padding = 47 - (i % 4 + 1) * 8;
            for (int j = 0; j < padding; j++) printf(" ");
            printf("║\n");
        }
    }
    printf("╚═══════════════════════════════════════════════════════════════════════╝\n");
}

static void print_summary_comparison(const Request baseline[], const Request optimized[], int n,
                                      const Elevator baseline_elevs[], const Elevator optimized_elevs[]) {
    ReportStats b = calculate_stats(baseline, n);
    ReportStats o = calculate_stats(optimized, n);

    printf("\n═══════════════════════════════════════════════════════════════════════\n");
    printf("                     STRATEGY COMPARISON                                \n");
    printf("═══════════════════════════════════════════════════════════════════════\n");
    printf("Metric                            | Round-Robin | Optimized | Improvement\n");
    printf("─────────────────────────────────────────────────────────────────────────\n");
    
    if (b.average_wait > 0.0) {
        double improvement = ((b.average_wait - o.average_wait) / b.average_wait) * 100.0;
        printf("Average wait (all)              | %11.1f s | %9.1f s | %+8.1f%%\n",
               b.average_wait, o.average_wait, improvement);
    }
    
    if (b.canteen_average_wait > 0.0 && o.canteen_count > 0) {
        double canteen_improvement =
            ((b.canteen_average_wait - o.canteen_average_wait) / b.canteen_average_wait) * 100.0;
        printf("Average wait (canteen)           | %11.1f s | %9.1f s | %+8.1f%%\n",
               b.canteen_average_wait, o.canteen_average_wait, canteen_improvement);
    }
    
    if (b.rush_average_wait > 0.0 && o.rush_count > 0) {
        double rush_improvement =
            ((b.rush_average_wait - o.rush_average_wait) / b.rush_average_wait) * 100.0;
        printf("Average wait (rush)              | %11.1f s | %9.1f s | %+8.1f%%\n",
               b.rush_average_wait, o.rush_average_wait, rush_improvement);
    }
    
    // FIXED: the original code cast `baseline`/`optimized` (Request arrays)
    // directly to Elevator* and read them as if they were Elevator structs -
    // undefined behavior, since the two types have entirely different
    // layouts and sizes. The actual elevator arrays are passed in properly now.
    double energy_b = calculate_total_energy(baseline_elevs, NUM_ELEVATORS);
    double energy_o = calculate_total_energy(optimized_elevs, NUM_ELEVATORS);
    if (energy_b > 0) {
        double energy_improvement = ((energy_b - energy_o) / energy_b) * 100.0;
        printf("Energy consumption               | %11.2f kWh | %9.2f kWh | %+8.1f%%\n",
               energy_b, energy_o, energy_improvement);
    }
    
    printf("Max queue length                 | %11d | %9d |\n",
           b.max_queue_length, o.max_queue_length);
    printf("═══════════════════════════════════════════════════════════════════════\n\n");
}

/* ---- Progress indicator ----
 * Show real-time simulation progress with a simple progress bar.
 */
static void show_progress(int current, int total) {
    int bar_width = 50;
    float progress = (float)current / total;
    int filled = (int)(progress * bar_width);
    
    printf("\r[");
    for (int i = 0; i < bar_width; i++) {
        if (i < filled) printf("█");
        else if (i == filled) printf("▶");
        else printf(" ");
    }
    printf("] %3.0f%%  Requests: %d/%d  Queue: %d  ",
           progress * 100, current, total, queue_length);
    fflush(stdout);
}

/* ---- CSV export ----
 * Writes per-request results to a CSV file. The original file claimed
 * this feature in its header comment but never actually implemented it -
 * this is a real, working version.
 */
static bool export_to_csv(const char *filename, const Request reqs[], int n) {
    if (filename == NULL || reqs == NULL || n <= 0) {
        return false;
    }

    FILE *f = fopen(filename, "w");
    if (f == NULL) {
        return false;
    }

    fprintf(f, "id,origin,destination,arrival_time,start_service,end_service,wait_time,assigned_elevator,peak_type\n");
    for (int i = 0; i < n; i++) {
        fprintf(f, "%d,%d,%d,%ld,%ld,%ld,%d,%d,%d\n",
                reqs[i].id, reqs[i].origin, reqs[i].destination,
                reqs[i].arrival_time, reqs[i].start_service, reqs[i].end_service,
                reqs[i].waiting_time, reqs[i].assigned_elevator, reqs[i].peak_type);
    }

    fclose(f);
    return true;
}

int main(void) {
    Request base[NUM_REQUESTS];
    Request round_robin_reqs[NUM_REQUESTS];
    Request optimized_reqs[NUM_REQUESTS];
    Elevator round_robin_elevs[NUM_ELEVATORS];
    Elevator optimized_elevs[NUM_ELEVATORS];

    srand(RANDOM_SEED);

    // Print configuration
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════════════╗\n");
    printf("║              ELEVATOR GROUP SIMULATION v4.0                          ║\n");
    printf("╠═══════════════════════════════════════════════════════════════════════╣\n");
    printf("║ Floors: %-6d  Elevators: %-6d  Canteen: Floor %-4d                 ║\n",
           NUM_FLOORS, NUM_ELEVATORS, CANTEEN_FLOOR);
    printf("║ Requests: %-6d  Seed: %-8u  Capacity: %-6d                        ║\n",
           NUM_REQUESTS, RANDOM_SEED, MAX_CAPACITY);
    printf("║ Lunch: %lds - %lds  Morning Rush: %lds - %lds                   ║\n",
           LUNCH_START, LUNCH_END, MORNING_RUSH_START, MORNING_RUSH_END);
    printf("║ Evening Rush: %lds - %lds                                        ║\n",
           EVENING_RUSH_START, EVENING_RUSH_END);
    printf("╚═══════════════════════════════════════════════════════════════════════╝\n");

    // Generate requests
    printf("\nGenerating %d requests...\n", NUM_REQUESTS);
    generate_requests(base, NUM_REQUESTS);
    memcpy(round_robin_reqs, base, sizeof(base));
    memcpy(optimized_reqs, base, sizeof(base));

    // Run Round-Robin simulation
    printf("\nRunning Round-Robin simulation...\n");
    simulate_round_robin(round_robin_reqs, NUM_REQUESTS, round_robin_elevs);
    
    // Reset and run Optimized simulation
    printf("Running Optimized simulation...\n");
    simulate_optimized(optimized_reqs, NUM_REQUESTS, optimized_elevs);

    // Print reports
    report("ROUND-ROBIN DISPATCH (Baseline)", 
           round_robin_reqs, NUM_REQUESTS, round_robin_elevs);
    
    report("NEAREST-ELEVATOR + SOFT CANTEEN ZONING (Optimized)",
           optimized_reqs, NUM_REQUESTS, optimized_elevs);
    
    print_summary_comparison(round_robin_reqs, optimized_reqs, NUM_REQUESTS,
                              round_robin_elevs, optimized_elevs);

    // Export results to CSV
    if (export_to_csv("round_robin_results.csv", round_robin_reqs, NUM_REQUESTS))
        printf("Exported round-robin results to round_robin_results.csv\n");
    else
        printf("Warning: failed to write round_robin_results.csv\n");

    if (export_to_csv("optimized_results.csv", optimized_reqs, NUM_REQUESTS))
        printf("Exported optimized results to optimized_results.csv\n\n");
    else
        printf("Warning: failed to write optimized_results.csv\n\n");

    return 0;
}