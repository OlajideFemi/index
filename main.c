#include <stdio.h>
#include <stdlib.h>

double metersToFeet(double meters) {
         return meters * 3.28084;
     }

double kilogramsToPounds(double kg) {
         return kg * 2.20462;
     }
int main()
{
         int choice;
         double value, result;

         printf("Unit Converter\n");
         printf("1. Meters to Feet\n");
         printf("2. Kilograms to Pounds\n");
         printf("Enter your choice: ");
         scanf("%d", &choice);

         printf("Enter the value to convert: ");
         scanf("%lf", &value);

         switch(choice) {
             case 1:
                 result = metersToFeet(value);
                 printf("%.2f meters is %.2f feet\n", value, result);
                 break;
             case 2:
                 result = kilogramsToPounds(value);
                 printf("%.2f kilograms is %.2f pounds\n", value, result);
                 break;
             default:
                 printf("Invalid choice\n");
         }

         return 0;
     }
