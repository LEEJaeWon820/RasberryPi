#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#define SENSOR1 7  // Physical pin 7
#define SENSOR2 0  // Physical pin 11
#define SENSOR3 3  // Physical pin 15
#define SENSOR4 2  // Physical pin 13

#define ADDRESS 0x16    // I2C address
static const char *deviceName = "/dev/i2c-1";  // I2C bus

int file_I2C = -1;  // I2C file descriptor

// Previous sensor values to detect changes
int prevSensor1 = -1;
int prevSensor2 = -1;
int prevSensor3 = -1;
int prevSensor4 = -1;

// Current motor speeds
int currentLSpeed = 0;
int currentRSpeed = 0;

void setup() {
    // Initialize wiringPi
    if (wiringPiSetup() == -1) {
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

int open_I2C(void) {
    int file;

    if ((file = open(deviceName, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %s: %s\n", deviceName, strerror(errno));
        return -1;
    }

    printf("I2C: Connected\n");

    printf("I2C: acquiring bus to 0x%x\n", ADDRESS);

    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x: %s\n", ADDRESS, strerror(errno));
        close(file);
        return -1;
    }

    return file;
}

void close_I2C(int fd) {
    if (fd != -1) {
        close(fd);
        printf("I2C: Closed\n");
    }
}

void car_control(int l_dir, int l_speed, int r_dir, int r_speed) {
    unsigned char data[5] = {0x01, l_dir, l_speed, r_dir, r_speed};

    printf("Sending control command to I2C bus...\n");
    if (write(file_I2C, data, 5) != 5) {
        fprintf(stderr, "Failed to write to the i2c bus: %s\n", strerror(errno));
    } else {
        printf("Control command sent successfully.\n");
    }
}

void car_stop() {
    unsigned char data[2] = {0x02, 0x00};

    printf("Sending stop command to I2C bus...\n");
    if (write(file_I2C, data, 2) != 2) {
        fprintf(stderr, "Failed to write to the i2c bus: %s\n", strerror(errno));
    } else {
        printf("Stop command sent successfully.\n");
    }
}

void line_trace(int sensor1, int sensor2, int sensor3, int sensor4) {
    int l_dir, l_speed, r_dir, r_speed;

    // Default direction and speed
    l_dir = 1;  // Forward
    r_dir = 1;  // Forward
    l_speed = 100;  // Speed settings (adjust as needed)
    r_speed = 100;

    // Check for all black (0000) or all white (1111) conditions
    if (sensor1 == 1 && sensor2 == 1 && sensor3 == 1 && sensor4 == 1) {
        // All sensors detect black line, stop the car
        car_stop();
        return;
    }

    // Detect changes in sensor states
    if (sensor3 == 0 && sensor2 == 1 && (prevSensor3 == 1 || prevSensor2 == 0)) {
        // Line is towards right, adjust direction to keep following the line
        l_dir = 1;  // Forward
        r_dir = 1;
        r_speed = 75;
        l_speed = 255;  // Slight left turn
    } else if (sensor3 == 1 && sensor2 == 0 && (prevSensor3 == 0 || prevSensor2 == 1)) {
        // Line is towards left, adjust direction to keep following the line
        l_dir = 1;  // Slight right turn
        r_dir = 1;  // Forward
        r_speed = 250;
        r_speed = 30;
    } else if (sensor3 == 1 && sensor2 == 1 && (prevSensor3 == 0 || prevSensor2 == 0)) {
        // Line is directly in front, go straight
        l_dir = 1;  // Forward
        r_dir = 1;  // Forward
    } else if (sensor3 == 1 && sensor2 == 1 && sensor1 == 1 && sensor4 == 1) {
        // Line is directly in front, go straight
        r_speed = 0;  // Forward
        r_speed = 0;  // Forward
    } else {
        // Maintain current direction and speed
        l_speed = currentLSpeed;
        r_speed = currentRSpeed;
    }

    // Save current sensor values for the next iteration
    prevSensor1 = sensor1;
    prevSensor2 = sensor2;
    prevSensor3 = sensor3;
    prevSensor4 = sensor4;

    // Send control command to the I2C bus
    car_control(l_dir, l_speed, r_dir, r_speed);

    // Update current motor speeds
    currentLSpeed = l_speed;
    currentRSpeed = r_speed;
}

void loop() {
    int sensor1Value = digitalRead(SENSOR1);
    int sensor2Value = digitalRead(SENSOR2);
    int sensor3Value = digitalRead(SENSOR3);
    int sensor4Value = digitalRead(SENSOR4);

    printf("Sensor Values: %d %d %d %d\n", sensor1Value, sensor2Value, sensor3Value, sensor4Value);

    // Perform line tracing based on sensor readings
    line_trace(sensor1Value, sensor2Value, sensor3Value, sensor4Value);

    delay(100); // Adjust delay as needed
}

int main(void) {
    setup();

    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Unable to initialize WiringPi: %s\n", strerror(errno));
        return 1; // Return non-zero value to indicate an error
    }

    file_I2C = open_I2C();

    if (file_I2C < 0) {
        printf("Unable to open I2C\n");
        return -1;
    } else {
        printf("I2C is Connected\n");
    }

    while (1) {
        loop();
    }

    close_I2C(file_I2C);

    return 0;
}
