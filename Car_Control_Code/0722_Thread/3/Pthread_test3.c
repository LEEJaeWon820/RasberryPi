#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <wiringPi.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <softPwm.h>


//////////////// GPIO MOTOR CONTROL //////////////
#define ENA 6  //Physerial 
#define IN1 4  //Physerial 
#define IN2 5  //Physerial 

#define ENB 0  //Physerial 
#define IN3 2  //Physerial 
#define IN4 3  //Physerial 


#define MAX_PWM_DUTY 100

/////////// Ultrasonic Sensor ////////

#define TRIG 21
#define ECHO 22
#define OBSTACLE_DISTANCE 10

void sig_Handler(int sig);

float u_sensor_data = 0;
int flag_obstacle_detection = 0;
int pwm_r, pwm_l;

void motor_control_r(int pwm)
{
   
   if(pwm>0)
   {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(ENA, pwm);
   }
   
   else if(pwm==0)
   {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(ENA, 0);
   }
   
   else
   {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      softPwmWrite(ENA, -pwm);
   }
   
}

void motor_control_l(int pwm)
{
   
   if(pwm>0)
   {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      digitalWrite(ENB, pwm);
   }
   
   else if(pwm==0)
   {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(ENB, 0);
   }
   
   else
   {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      softPwmWrite(ENA, -pwm);
   }
   
}


int GPIO_control_setup(void)
{
   if(wiringPiSetup() == -1)
   {
      printf("wiringPi Setup error !\n");
      return -1;
   }
   
   pinMode(ENA, OUTPUT);
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   
   pinMode(ENB, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
   
   pinMode(TRIG, OUTPUT);
   pinMode(ECHO, INPUT);
   
   softPwmCreate(ENA, 1, MAX_PWM_DUTY);
   softPwmCreate(ENB, 1, MAX_PWM_DUTY);
   
   softPwmWrite(ENA, 0);
   softPwmWrite(ENB, 0);
   
   pwm_r = pwm_l = 0;
   
   return 0;
}

float ultrasonic_sensor()
{
   long start_time, end_time;
   long temp_time1, temp_time2;
   int duration = -1;
   float distance = 0;
   
   digitalWrite(TRIG, LOW);
   delayMicroseconds(5);
   digitalWrite(TRIG, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG, LOW);
   
   delayMicroseconds(200);    // wait for burst signal. 40kHz x 8 = 8x25us = 200
   
   //printf("200msec \n");
   temp_time1 = micros();
   
   
   while(digitalRead(ECHO) == LOW)   // wait untiol ECHO pin is HIGH
   {
      temp_time2 = micros();
      duration = temp_time2 - temp_time1;
      if(duration>1000) return -1;
      
   }
   
   
   
   start_time = micros();
   
   //printf("echo signal high \n");

   while(digitalRead(ECHO) == HIGH)   // wait untiol ECHO pin is LOW
   {
      temp_time2 = micros();
      duration = temp_time2 - temp_time1;
      if(duration > 4000) return -1;
      
   }
   end_time = micros();
   
   //printf("echo signal low \n");
   
	   
   duration = end_time - start_time;
   distance = duration / 58;

   return distance;
}


void* ultrasonic_sensor_thread(void* num)
{	
   while(1)
   {
	   u_sensor_data = ultrasonic_sensor();
	   if(u_sensor_data <= OBSTACLE_DISTANCE) 
	   {
		   printf("Obstacle detected !! \n\n");
		   flag_obstacle_detection = 1;
	   }
   }
	//pthread_exit(NULL);
}

void* motor_control_thread(void* num)
{
	while(1)
	{
		motor_control_r(pwm_r);
        motor_control_l(pwm_l);
	}
}

int main(void)
{
	pthread_t pthread_A, pthread_B;
	
	int cnt = 0;
	
	if(GPIO_control_setup() == -1)
   {
      
      return -1;
   }

   signal(SIGINT, sig_Handler);
	
	printf("Create Thread A \n");
	pthread_create(&pthread_A, NULL, ultrasonic_sensor_thread, NULL);
	
	printf("Create Thread B \n");
	pthread_create(&pthread_B, NULL, motor_control_thread, NULL);
	
	//pthread_join(ultrasonic_sensor_thread,NULL);
	//pthread_join(pthread_B,NULL);
	
	while(1)
	{
		printf("Ultrasonic Sensro : %6.3lf [cm] \n", u_sensor_data);
		printf("Thread Test : %3d \n",cnt); 
		cnt++;
		cnt = cnt%100;
		pwm_r=pwm_l = cnt;
		delay(100);
	}
	
	return 1;
}

void sig_Handler(int sig)
{
   printf("\n\n\n\nProgram and Motor Stop\n\n\n");
   //motor_control_r(0);
   //motor_control_l(0);
   exit(0);
}
