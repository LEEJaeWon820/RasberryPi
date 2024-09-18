#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termio.h>
#include <softPwm.h>

#define GPIO0 0  //Physerial 11
#define GPIO3 3  //Physerial 15

#define ENA 6  //Physerial 
#define IN1 4  //Physerial 
#define IN2 5  //Physerial 

#define ENB 0  //Physerial 
#define IN3 2  //Physerial 
#define IN4 3  //Physerial 


#define MAX_PWM_DUTY 100
#define baud_rate 115200

void sig_Handler(int sig);

int getch(void)
{

int ch;
struct termios buf;
struct termios save;

tcgetattr(0, &save);
buf = save;
buf.c_lflag &= ~(ICANON|ECHO);
buf.c_cc[VMIN] = 1;
buf.c_cc[VTIME] = 0;
tcsetattr(0, TCSAFLUSH, &buf);
ch = getchar();
tcsetattr(0, TCSAFLUSH, &save);
return ch;
}


int motor_control_setup(void)
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
   
   softPwmCreate(ENA, 1, MAX_PWM_DUTY);
   softPwmCreate(ENB, 1, MAX_PWM_DUTY);
   
   softPwmWrite(ENA, 0);
   softPwmWrite(ENB, 0);
   return 0;
}
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

int main(void)
{
   
   int fd;
   int pwm_r =0;
   int pwm_l =0;
   unsigned char test, receive_char;
   
   if(motor_control_setup() == -1)
   {
      
      return -1;
   }

   signal(SIGINT, sig_Handler);
   test = 'B';
   
   while(1)
   {
      test = getch();
      
      switch(test)
      {
         case 'w':
                  motor_control_r(pwm_r);
                  motor_control_l(pwm_l);
                  pwm_r++;
                  pwm_l++;
                  break;
         case 's':
                  motor_control_r(0);
                  motor_control_l(0);
                  break;
         case 'x':
                  motor_control_r(-pwm_r);
                  motor_control_l(-pwm_l);
                  pwm_r--;
                  pwm_l--;
                  break;
      }
      
      //delay(500);
      
   }
   return 0;
}
void sig_Handler(int sig)
{
   printf("\n\n\n\nProgram and Motor Stop\n\n\n");
   motor_control_r(0);
   motor_control_l(0);
   exit(0);
}
