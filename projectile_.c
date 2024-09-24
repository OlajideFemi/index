#include <stdio.h>
#include <math.h>


#define G 9.81 

void simulateProjectile(double v0, double angle, double delta_t) {

    double theta = angle * M_PI / 180.0;
    
    double vx0 = v0 * cos(theta);
    double vy0 = v0 * sin(theta);

    double t = 0.0;
    double x = 0.0, y = 0.0;
    
    printf("Time(s)\tX(m)\tY(m)\tVx(m/s)\tVy(m/s)\n");

    while (y >= 0) { 
        x = vx0 * t;
        y = vy0 * t - 0.5 * G * t * t;
        double vx = vx0;
        double vy = vy0 - G * t;

        printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", t, x, y, vx, vy);

        t += delta_t; 
    }
}

int main() {
    double velocity, angle, timeStep;

    printf("Enter initial velocity (m/s): ");
    scanf("%lf", &velocity);
    printf("Enter launch angle (degrees): ");
    scanf("%lf", &angle);
    printf("Enter time step (seconds): ");
    scanf("%lf", &timeStep);

    simulateProjectile(velocity, angle, timeStep);

    return 0;
}
