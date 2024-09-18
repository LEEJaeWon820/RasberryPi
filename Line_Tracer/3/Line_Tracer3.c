#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termio.h>
#include <softPwm.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// I2C address
#define ADDRESS 0x16

// I2C bus
static const char *deviceName = "/dev/i2c-1";

#define IMG_Width     640
#define IMG_Height    480

#define ASSIST_BASE_LINE 320
#define ASSIST_BASE_WIDTH 40

int guide_width1 = 50;
int guide_height1 = 20;
int guide_center = IMG_Width / 2;
int line_center = -1;

#define MAX_PWM_DUTY 100
#define PWM_BASE 20
#define P_gain 0.05

#define TRIG 21
#define ECHO 22

#define baud_rate 115200

int file_I2C;

using namespace cv;
using namespace std;

int getch(void) {
    int ch;
    struct termios buf;
    struct termios save;

    tcgetattr(0, &save);
    buf = save;
    buf.c_lflag &= ~(ICANON | ECHO);
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    return ch;
}

int GPIO_control_setup(void) {
    if (wiringPiSetup() == -1) {
        printf("wiringPi Setup error!\n");
        return -1;
    }

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    return 0;
}

float ultrasonic_sensor(void)
 {
    long start_time, end_time;
    long temp_time1, temp_time2;
    int duration;
    float distance;

    digitalWrite(TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    delayMicroseconds(200); // wait for burst signal. 40kHz x 8 = 8x25us = 200

    temp_time1 = micros();

    while (digitalRead(ECHO) == LOW) 
    {
        temp_time2 = micros();
        duration = temp_time2 - temp_time1;
        if (duration > 1000) return -1;
    }

    start_time = micros();

    while (digitalRead(ECHO) == HIGH) 
    { // wait until ECHO pin is LOW
        temp_time2 = micros();
        duration = temp_time2 - temp_time1;
        if (duration > 2000) return -1;
    }
    end_time = micros();

    duration = end_time - start_time;
    distance = duration / 58;

    return distance;
}

Mat region_of_interest(Mat img, Point *points) 
{
    Mat img_mask = Mat::zeros(img.rows, img.cols, CV_8UC1);
    Scalar mask_color = Scalar(255, 255, 255);
    const Point* pt[1] = { points };
    int npt[] = { 4 };

    fillPoly(img_mask, pt, npt, 1, Scalar(255,255,255), LINE_8);

    Mat masked_img;
    bitwise_and(img, img_mask, masked_img);

    return masked_img;
}

Mat Canny_Edge_Detection(Mat img)
 {
    Mat mat_blur_img, mat_canny_img;
    blur(img, mat_blur_img, Size(3, 3));
    Canny(mat_blur_img, mat_canny_img, 50, 165, 3);

    return mat_canny_img;
}

Mat Draw_Guide_Line(Mat img) 
{
    Mat result_img;
    img.copyTo(result_img);

    rectangle(result_img, Point(50, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH),
              Point(IMG_Width - 50, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH),
              Scalar(152, 160, 237), 1, LINE_AA);

    line(result_img, Point(guide_center - guide_width1, ASSIST_BASE_LINE),
         Point(guide_center, ASSIST_BASE_LINE), Scalar(0, 255, 255), 1, 0);
    line(result_img, Point(guide_center, ASSIST_BASE_LINE),
         Point(guide_center + guide_width1, ASSIST_BASE_LINE), Scalar(0, 255, 255), 1, 0);

    line(result_img, Point(guide_center - guide_width1, ASSIST_BASE_LINE - guide_height1),
         Point(guide_center - guide_width1, ASSIST_BASE_LINE + guide_height1), Scalar(0, 255, 255), 1, 0);
    line(result_img, Point(guide_center + guide_width1, ASSIST_BASE_LINE - guide_height1),
         Point(guide_center + guide_width1, ASSIST_BASE_LINE + guide_height1), Scalar(0, 255, 255), 1, 0);
    line(result_img, Point(IMG_Width / 2 , ASSIST_BASE_LINE - guide_height1 * 1.5),
        Point(IMG_Width / 2, ASSIST_BASE_LINE + guide_height1 * 1.5), Scalar(255), 2, 0);
    line(result_img, Point(line_center, ASSIST_BASE_LINE - guide_height1 * 1.2),
        Point(line_center, ASSIST_BASE_LINE + guide_height1 * 1.2), Scalar(0, 0, 255), 2, 0);

    return result_img;
}

void car_control(int l_dir, int l_speed, int r_dir, int r_speed);
void car_stop();
void close_I2C(int fd);

void line_tracer_motor_control(int line_center) 
{
    int pwm_r, pwm_l;
    int steer_error = 0;
    
    steer_error = line_center - IMG_Width / 2;
    
    pwm_r = PWM_BASE - steer_error * P_gain;
    pwm_l = PWM_BASE + steer_error * P_gain;
    
    printf("PWM L : %3d | PWM R : %3d \n", pwm_l, pwm_r);

    car_control(1, pwm_l, 1, pwm_r);  // I2C를 통해 자동차 제어
}

void sig_Handler(int sig) 
{
    printf("\n\n\n\nProgram and Motor Stop\n\n\n");
    car_stop();  // I2C를 통해 자동차 정지
    close_I2C(file_I2C);  // I2C 파일 닫기
    exit(0);
}

int open_I2C(void)
{
    int file;
    if ((file = open(deviceName, O_RDWR)) < 0)
    {
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);
        exit(1);
    }
    printf("I2C: Connected\n");

    printf("I2C: acquiring bus to 0x%x\n", ADDRESS);

    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)
    {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
        exit(1);
    }
    return file;
}

void close_I2C(int fd)
{
    close(fd);
}

void car_control(int l_dir, int l_speed, int r_dir, int r_speed)
{
    unsigned char data[5] = {0x01, l_dir, l_speed, r_dir, r_speed};
    if (write(file_I2C, data, 5) != 5)
    {
        printf("Failed to write to the i2c bus.\n");
    }
}

void car_stop()
{
    unsigned char data[2] = {0x02, 0x00};
    if (write(file_I2C, data, 2) != 2)
    {
        printf("Failed to write to the i2c bus.\n");
    }
}

int main(void) 
{
    int fd;
    int pwm_r = 0;
    int pwm_l = 0;
    unsigned char test;
    
    int img_width, img_height;
    Point points[4];
    int no_label;
    int c_x, c_y;
    int c_x_sum = 0;
    int steer_error = 0;

    img_width = 640;
    img_height = 480;
    c_x = c_y = 0;

    Mat mat_image_org_color_Overlay;
    Mat mat_image_org_color;
    Mat mat_image_org_gray;
    Mat mat_image_org_gray_result;
    Mat mat_image_canny_edge;
    Mat mat_image_roi_canny_edge;

    Mat mat_image_labels, stats, centroids;
    int line_count;

    int center_x_sum = 0;

    signal(SIGINT, sig_Handler);
    file_I2C = open_I2C();  // I2C 초기화
    VideoCapture cap(0);

    if (!cap.isOpened()) 
    {
        printf("Camera open failed\n");
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    while (1) 
    {
        cap.read(mat_image_org_color);
        if (mat_image_org_color.empty()) 
        {
            printf("Image load failed\n");
            break;
        }

        mat_image_org_color_Overlay = mat_image_org_color.clone();
        cvtColor(mat_image_org_color, mat_image_org_gray, COLOR_RGB2GRAY);
        mat_image_canny_edge = Canny_Edge_Detection(mat_image_org_gray);
        
        points[0] = Point(50, 300);
        points[1] = Point(590, 300);
        points[2] = Point(590, 480);
        points[3] = Point(50, 480);

        mat_image_roi_canny_edge = region_of_interest(mat_image_canny_edge, points);
        
        line_count = connectedComponentsWithStats(mat_image_roi_canny_edge, mat_image_labels, stats, centroids, 8, CV_32S);

        int line_center_x = 0;
        int count_valid_line = 0;
        for (int j = 1; j < line_count; j++) 
        {
            int area = stats.at<int>(j, CC_STAT_AREA);
            if (area >= 50 && area <= 1000) 
            {
                line_center_x += centroids.at<double>(j, 0);
                count_valid_line++;
            }
        }

        if (count_valid_line > 0) 
        {
            line_center_x = line_center_x / count_valid_line;
            line_center = line_center_x;
        }

        if (line_center > 0) 
        {
            line(mat_image_org_color_Overlay, Point(line_center, 0), Point(line_center, IMG_Height), Scalar(0, 0, 255), 2);
        }

        line_tracer_motor_control(line_center);  // 모터 제어 함수 호출
        
        imshow("Display window", mat_image_org_color_Overlay);

        if (waitKey(1) >= 0) break;
    }

    cap.release();
    car_stop();  // I2C를 통해 자동차 정지
    close_I2C(file_I2C);  // I2C 파일 닫기
    return 0;
}
