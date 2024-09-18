
#include <stdio.h>
#include <wiringPi.h>
#include <stdlib.h>


#define SENSOR1 7  // Physical pin 7
#define SENSOR2 0  // Physical pin 11
#define SENSOR3 3  // Physical pin 15
#define SENSOR4 2  // Physical pin 13



void setup() 
{
    // Initialize wiringPi
    if (wiringPiSetup() == -1) 
    {
        printf("wiringPi Setup error!\n");
        exit(1);
    }
    
    // Set sensor pins as input
    pinMode(SENSOR1, INPUT);
    pinMode(SENSOR2, INPUT);
    pinMode(SENSOR3, INPUT);
    pinMode(SENSOR4, INPUT);
}

void loop() 
{
    int sensor1Value = digitalRead(SENSOR1);
    int sensor2Value = digitalRead(SENSOR2);
    int sensor3Value = digitalRead(SENSOR3);
    int sensor4Value = digitalRead(SENSOR4);

    printf("Sensor Values: %d %d %d %d\n", sensor1Value, sensor2Value, sensor3Value, sensor4Value);

    delay(1000); // Wait for 1 second
}

int main(void) 
{
    setup();
    while (1) 
    {
        loop();
    }
    return 0;
}
