#include <PIDController.h>
#include "DualMC33926MotorShield.h"
#include "ADNS2610.h"
#include <math.h>

//Pins of the optical mouse
#define SCLK1 24                            // Serial clock pin on the Arduino
#define SDIO1 26

// Create an instance of the ADNS2610 object for each mouse
ADNS2610 Optical1 = ADNS2610(SCLK1, SDIO1);

// x and y coordinates mesured with the optical mice
signed long x1 = 0;                        
signed long y1 = 0; 

//Create a object Motor driver (md)
DualMC33926MotorShield md;
DualMC33926MotorShield mdr;

// Motor Encoder variables
#define  A_PHASE_M 18 // green cable
#define  B_PHASE_M 19 // white cable

int flag_A_M = 0;
int flag_B_M = 0;
const float encoder_resolution = 600;

#define  A_PHASE 20 // green cable
#define  B_PHASE 21 // white cable

int flag_A = 0;
int flag_B = 0;

float encoder_angle;
PIDController pos_pid; 
int motor_value;

float encoder_angle2;
PIDController pos_pid_r; 
int motor_value2;

int pot;
int pot2;
float angle_1;
float angle_2;

// Inverse kinematics variables

double x;
double y;
const double d = 65; // 65 mm
const double l1f = 120; //120 mm
const double l2f = 120;
const double l1r = 120;
const double l2r = 120;
const double x0 = 35;
const double y0 = 175;

void setup() 
{

  Serial.begin(9600);

  Optical1.begin(); 
  
  pinMode(A_PHASE_M, INPUT_PULLUP);
  pinMode(B_PHASE_M, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A_PHASE_M), interrupt2, RISING); //Interrupt trigger mode: RISING
  
  pinMode(A_PHASE, INPUT_PULLUP);
  pinMode(B_PHASE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A_PHASE), interrupt, RISING); //Interrupt trigger mode: RISING

  // Initialize the motor driver (Motor setup)
  md.init();
  mdr.init();
  
  pos_pid.begin();    
  pos_pid.tune(65, 0.1, 0.6); 
  pos_pid.limit(-400, 400);

  pos_pid_r.begin();    
  pos_pid_r.tune(65, 0.1, 0.6);   
  pos_pid_r.limit(-400, 400);

}

void loop() 
{
  //Read the x and y coordinates from the optical mouse
  x1 += Optical1.dx();
  y1 += Optical1.dy();  
  
  //The value of x and y coordinates 
  x = x0+((x1*10)/120);
  y = y0+((y1*10)/120);

  //Calculate q1 and q2 using the inverse kinematics functions
  angle_1 = InverseKinematics_q1 (l2f, l1f, x, y);
  angle_2 = InverseKinematics_q2 (l2r, l1r, x, y, d);
  
  M_left_write (angle_1);

  M_right_write (angle_2);
    
  Serial.print("x:");
  Serial.print(x);
  Serial.print(" ; ");
  Serial.print(" y:");
  Serial.println(y);
//  Serial.print(" Desired Angle 1: ");
//  Serial.print(angle_1);
//  Serial.print(" ; ");
//  Serial.print(" Desired Angle 2: ");
//  Serial.print(angle_2); 
//  Serial.print(" Angle 1: ");
//  Serial.print(encoder_angle);
//  Serial.print(" ; ");
//  Serial.println(micros());
//  Serial.print(" ; ");
//  Serial.print(" Angle 2: ");
//  Serial.print(encoder_angle2);
//  Serial.print(" Error1:");
//  Serial.print(abs((encoder_angle-angle_1)/angle_1)*100);
//  Serial.print(" Error2:");
//  Serial.println(abs((encoder_angle2-angle_2)/angle_2)*100);
 
}

void M_left_write (float angle)
{
  encoder_angle = 120+(-1*(flag_A_M-flag_B_M)*0.6); 
  pos_pid.setpoint(angle);
  motor_value = pos_pid.compute(encoder_angle);
  md.setM1Speed(motor_value);
}

void M_right_write (float angle2)
{
  encoder_angle2 = 60+(-1*(flag_A-flag_B)*0.6); 
  pos_pid_r.setpoint(angle2);
  motor_value2 = pos_pid_r.compute(encoder_angle2);
  mdr.setM2Speed(motor_value2);
}

void interrupt()
{ char i;
  i = digitalRead(B_PHASE);
  if (i == 1)
    flag_A += 1;
  else
    flag_B += 1;
}

void interrupt2()
{ char k;
  k = digitalRead(B_PHASE_M);
  if (k == 1)
    flag_A_M += 1;
  else
    flag_B_M += 1;
}

double InverseKinematics_q1 (double l2f, double l1f, double x, double y)
{
  double q1;
  q1 = atan2(y,x) + acos((-1*pow(l2f,2)+pow(l1f,2)+pow(x,2)+pow(y,2))/(2*l1f*sqrt(pow(x,2)+pow(y,2))));
  return q1*(360/(2*PI)); // return q1 in degrees
  //return q1;  // return q1 in radians
}

double InverseKinematics_q2 (double l2r, double l1r, double x, double y, double d)
{
  double dif_dx = (d-x);
  double q2;
  q2 = PI - atan2(y,dif_dx) - acos((-1*pow(l2r,2)+pow(l1r,2)+pow(dif_dx,2)+pow(y,2))/(2*l1r*sqrt(pow(dif_dx,2)+pow(y,2))));
  return q2*(360/(2*PI)); // return q2 in degrees
  //return q2; // return q2 in radians
}
