#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>

#define GPIO0 0  // Physical pin 11
#define PWM_RANGE 200  // PWM range for softPwm

int servo_angle = 90;
pthread_t pthread_A;

// Function prototypes
void* RC_Servo_Control_thread(void *data);
void sig_Handler(int sig);

int GPIO_control_setup(void)
{
    if (wiringPiSetup() == -1)
    {
        printf("wiringPi Setup error!\n");
        return -1;
    }
    
    pinMode(GPIO0, OUTPUT);
    digitalWrite(GPIO0, LOW);
    softPwmCreate(GPIO0, 0, PWM_RANGE); // Setup software PWM for GPIO0
    
    return 0;
}

void* RC_Servo_Control_thread(void *data)
{
    while (1)
    {
        softPwmWrite(GPIO0, servo_angle);
        delay(20);  // Adjust delay as needed for servo control
    }
    return NULL;
}

void sig_Handler(int sig)
{
    printf("\nProgram and Motor Stopped!\n");
    digitalWrite(GPIO0, LOW); // Ensure GPIO is off when program stops
    pthread_cancel(pthread_A); // Cancel thread to exit cleanly
    exit(0);
}

int main(void)
{
    if (GPIO_control_setup() == -1)
    {
        return -1;
    }
    
    signal(SIGINT, sig_Handler); // Register signal handler for Ctrl+C
    
    printf("Creating Thread A\n");
    pthread_create(&pthread_A, NULL, RC_Servo_Control_thread, NULL);
    
    // Main loop for servo control
    while (1)
    {
        for (servo_angle = 0; servo_angle <= 180; servo_angle++)
        {
            delay(20);  // Adjust delay as needed for servo movement speed
        }
        for (servo_angle = 180; servo_angle >= 0; servo_angle--)
        {
            delay(20);  // Adjust delay as needed for servo movement speed
        }
    }
    
    return 0;
}
