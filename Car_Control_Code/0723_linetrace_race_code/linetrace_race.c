#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#define SENSOR1 2  // Physical pin 13 (rightmost sensor)
#define SENSOR2 3  // Physical pin 15
#define SENSOR3 0  // Physical pin 11
#define SENSOR4 7  // Physical pin 7 (leftmost sensor)

#define ADDRESS 0x16    // I2C address
static const char *deviceName = "/dev/i2c-1";  // I2C bus

int file_I2C = -1;  // I2C file descriptor

void setup() 
{
    // Initialize wiringPi
    if (wiringPiSetup() == -1) 
    {
        fprintf(stderr, "wiringPi Setup error: %s\n", strerror(errno));
        // Handle the error gracefully, for example, print an error message and return
        return;
    }
    
    // Set sensor pins as input
    pinMode(SENSOR1, INPUT);
    pinMode(SENSOR2, INPUT);
    pinMode(SENSOR3, INPUT);
    pinMode(SENSOR4, INPUT);
}

int open_I2C(void) 
{
    int file;

    if ((file = open(deviceName, O_RDWR)) < 0) 
    {
        fprintf(stderr, "I2C: Failed to access %s: %s\n", deviceName, strerror(errno));
        return -1;
    }

    printf("I2C: Connected\n");

    printf("I2C: acquiring bus to 0x%x\n", ADDRESS);

    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) 
    {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x: %s\n", ADDRESS, strerror(errno));
        close(file);
        return -1;
    }

    return file;
}

void close_I2C(int fd) 
{
    if (fd != -1) 
    {
        close(fd);
        printf("I2C: Closed\n");
    }
}

void car_control(int l_dir, int l_speed, int r_dir, int r_speed) 
{
    unsigned char data[5] = {0x01, l_dir, l_speed, r_dir, r_speed};

    printf("Sending control command to I2C bus...\n");
    if (write(file_I2C, data, 5) != 5) 
    {
        fprintf(stderr, "Failed to write to the i2c bus: %s\n", strerror(errno));
    } 
    else 
    {
        printf("Control command sent successfully.\n");
    }
}

void car_stop() 
{
    unsigned char data[2] = {0x02, 0x00};

    printf("Sending stop command to I2C bus...\n");
    if (write(file_I2C, data, 2) != 2) 
    {
        fprintf(stderr, "Failed to write to the i2c bus: %s\n", strerror(errno));
    } 
    else 
    {
        printf("Stop command sent successfully.\n");
    }
}

void line_trace(int sensor1, int sensor2, int sensor3, int sensor4) 
{
    int l_dir, l_speed, r_dir, r_speed;

    // Default direction and speed
    //l_dir = 1;  // Forward
    //r_dir = 1;  // Forward
    //l_speed = 100;  // Speed settings (adjust as needed)
    //r_speed = 100;

    // Adjust direction and speed based on sensor readings
    if (sensor2 == 0 && sensor3 == 1) 
    {
        // Line is towards right, adjust direction and speed for a right turn
        l_dir = 1;  // Forward
        r_dir = 1;  // Right turn
        l_speed = 50;  // Reduce left motor speed for sharper turn
        r_speed = 200; // Right motor full speed
    } 
    else if (sensor2 == 1 && sensor3 == 0) 
    {
        // Line is towards left, adjust direction and speed for a left turn
        l_dir = 1;  // Left turn
        r_dir = 1;  // Forward
        l_speed = 200; // Left motor full speed
        r_speed = 80;  // Reduce right motor speed for sharper turn
    } 
    else if (sensor2 == 1 && sensor3 == 1) 
    {
        // Line is directly in front, go straight
        l_dir = 1;  // Forward
        r_dir = 1;  // Forward
        l_speed = 100;  // Adjust as needed
        r_speed = 100;  // Adjust as needed
    } 
    else if (sensor2 == 0 && sensor3 == 0)
    {
        l_dir = 1;
        r_dir = 1;
        l_speed = l_speed;
        r_speed = r_speed;
    }

    // Send control command to the I2C bus
    car_control(l_dir, l_speed, r_dir, r_speed);
}

void loop() 
{
    int sensor1Value = digitalRead(SENSOR1);
    int sensor2Value = digitalRead(SENSOR2);
    int sensor3Value = digitalRead(SENSOR3);
    int sensor4Value = digitalRead(SENSOR4);

    printf("Sensor Values: %d %d %d %d\n", sensor1Value, sensor2Value, sensor3Value, sensor4Value);

    // Perform line tracing based on sensor readings
    line_trace(sensor1Value, sensor2Value, sensor3Value, sensor4Value);
}

int main(void) 
{
    setup();

    file_I2C = open_I2C();

    if (file_I2C < 0) 
    {
        printf("Unable to open I2C\n");
        return -1;
    } 
    else 
    {
        printf("I2C is Connected\n");
    }

    while (1) 
    {
        loop();
    }

    close_I2C(file_I2C);

    return 0;
}
