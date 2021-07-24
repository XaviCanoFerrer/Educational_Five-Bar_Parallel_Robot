#include "TeensyStep.h"
#include <math.h>

const double d = 80;
const double l1f = 100; 
const double l2f = 100;
const double l1r = 100;
const double l2r = 100;

double X0;
double Y0;

int X1 = 40;
int Y1 = 170;

//double x ;
//double y ;

double xa ;
double ya ;

double x[5] = {X1, X1, X1-30,X1-30, X1};
double y[5] = {Y1, Y1+15,Y1+15,Y1, Y1};

double angle_1;
double angle_2;

int angle_1_steps;
int angle_2_steps;

const int DIR1 = 10;
const int STEP1 = 9;

const int DIR2 = 19;
const int STEP2 = 20;

const int SW1 = 11;
const int SW2 = 12;

const int steps_rev = 3200;

int pd = 2000;

Stepper q1(STEP1, DIR1);       
Stepper q2(STEP2, DIR2);     
 
StepControl controller; 

void setup()
{
    // setup the motors 
   q1
    .setMaxSpeed(10000)       // steps/s
    .setAcceleration(10000); // steps/s^2 

   q2
    .setMaxSpeed(10000)       // steps/s
    .setAcceleration(10000); // steps/s^2 

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  
  // Homing routine

  while (digitalRead(SW1) == HIGH)      
  {
    TurnClockwise_q1 (pd);
    TurnClockwise_q2 (pd);
    Serial.println(digitalRead(SW1));
  }

  int q2_initial_steps = 0;
 
  while (digitalRead(SW2) == HIGH)      
  {
    TurnCounterClockwise_q1 (pd);
    TurnCounterClockwise_q2 (pd);
    Serial.println(digitalRead(SW2));
    q2_initial_steps++;
  }

  controller.stop();

  double initial_q2 = step_to_deg(q2_initial_steps);
  double initial_q1 = 180;
  
  Serial.print(q2_initial_steps);
  Serial.print(" ");
  Serial.println(initial_q2);

  angle_1 = initial_q1;
  angle_2 = initial_q2;
  
  
  angle_1_steps = deg_to_step(angle_1);
  angle_2_steps = deg_to_step(angle_2);
   
  q1.setTargetAbs(angle_1_steps);
  q2.setTargetAbs(angle_2_steps);

  X0 = ForwardKinematics_x (l2f,l1f,l2r,l1r,d,angle_1,angle_2);
  Y0 = ForwardKinematics_y (l2f,l1f,l2r,l1r,d,angle_1,angle_2);

  Serial.print("X0 = ");
  Serial.print(X0);
  Serial.print("Y0 = ");
  Serial.print(Y0);
  Serial.print("   ");
  Serial.print(angle_1_steps);
  Serial.print("   ");
  Serial.print(angle_2_steps);
  Serial.println("Homing completed");
  
}

void loop() 
{

//// Move the end effector making the shape of a rectangle

for (int i = 0; i < 5; i = i + 1) 
  {
    

    if (digitalRead(SW1) == LOW || digitalRead(SW2) == LOW)
    {
      controller.stop();
    }

    //Calculate q1 and q2 using the inverse kinematics functions
    angle_1 = InverseKinematics_q1 (l2f, l1f, x[i], y[i]);
    angle_2 = InverseKinematics_q2 (l2r, l1r, x[i], y[i], d);
  
    angle_1_steps = deg_to_step(angle_1);
    angle_2_steps = deg_to_step(angle_2);
  
    
    q1.setTargetAbs(angle_1_steps);
    q2.setTargetAbs(angle_2_steps);
    controller.move(q1,q2);

    Serial.print(x[i]);
    Serial.print("   ");
    Serial.println(y[i]);
    
  }

}


// Function to get the angle from the steps
double step_to_deg (int steps)
{
  int deg;
  deg = steps*360/steps_rev;
  return deg;
}

// Function to get the steps from the angle
int deg_to_step (double deg)
{
  int steps;
  steps = deg*(steps_rev/360);
  return steps;
}

//Function to get the angle q1 using the inverse kinematics equation
double InverseKinematics_q1 (double l2f, double l1f, double x, double y)
{
  double q1;
  q1 = atan2(y,x) + acos((-1*pow(l2f,2)+pow(l1f,2)+pow(x,2)+pow(y,2))/(2*l1f*sqrt(pow(x,2)+pow(y,2))));
  return q1*(360/(2*PI)); // return q1 in degrees
  //return q1;  // return q1 in radians
}

//Function to get the angle q2 using the inverse kinematics equation
double InverseKinematics_q2 (double l2r, double l1r, double x, double y, double d)
{
  double dif_dx = (d-x);
  double q2;
  q2 = PI - atan2(y,dif_dx) - acos((-1*pow(l2r,2)+pow(l1r,2)+pow(dif_dx,2)+pow(y,2))/(2*l1r*sqrt(pow(dif_dx,2)+pow(y,2))));
  return q2*(360/(2*PI)); // return q2 in degrees
  //return q2; // return q2 in radians
}

//Function to get the x coordinate of the end effector
double ForwardKinematics_x (double l2f, double l1f, double l2r, double l1r, double d, double q1_deg, double q2_deg)
{
  double q1;
  double q2;
  double a;
  double b;
  double c;
  double x0;
  double theta1;
  double theta2;

  q1 = q1_deg*PI/180;
  q2 = q2_deg*PI/180;
  a = 2*l2r*l1r*sin(q2)-2*l1f*l2r*sin(q1);
  b = 2*l2r*d-2*l1f*l2r*cos(q1)+2*l2r*l1r*cos(q2);
  c = pow(l1f,2)-pow(l2f,2)+pow(l2r,2)+pow(l1r,2)+pow(d,2)-l1f*l1r*sin(q1)*sin(q2)-2*l1f*d*cos(q1)+2*l1r*d*cos(q2)-2*l1f*l1r*cos(q1)*cos(q2);
  theta2 = 2*atan2((a+sqrt(pow(a,2)+pow(b,2)-pow(c,2))),(b-c));
  theta1 = asin((l2r*sin(theta2)+l1r*sin(q2)-l1f*sin(q1))/l2f);
  x0 = l1f*cos(q1)+l2f*cos(theta1);
  return x0;
}

//Function to get the y coordinate of the end effector
double ForwardKinematics_y (double l2f, double l1f, double l2r, double l1r, double d, double q1_deg, double q2_deg)
{
  double q1;
  double q2;
  double a;
  double b;
  double c;
  double y0;
  double theta1;
  double theta2;

  q1 = q1_deg*PI/180;
  q2 = q2_deg*PI/180;
  a = 2*l2r*l1r*sin(q2)-2*l1f*l2r*sin(q1);
  b = 2*l2r*d-2*l1f*l2r*cos(q1)+2*l2r*l1r*cos(q2);
  c = pow(l1f,2)-pow(l2f,2)+pow(l2r,2)+pow(l1r,2)+pow(d,2)-l1f*l1r*sin(q1)*sin(q2)-2*l1f*d*cos(q1)+2*l1r*d*cos(q2)-2*l1f*l1r*cos(q1)*cos(q2);
  theta2 = 2*atan2((a+sqrt(pow(a,2)+pow(b,2)-pow(c,2))),(b-c));
  theta1 = asin((l2r*sin(theta2)+l1r*sin(q2)-l1f*sin(q1))/l2f);
  y0 = l1f*sin(q1)+l2f*sin(theta1);
  return y0;
}

//Function to rotate the left motor CCW
void TurnCounterClockwise_q1 (int pd)
{
   digitalWrite(DIR1,0);
   digitalWrite(STEP1,HIGH);
   //delayMicroseconds(pd);
   digitalWrite(STEP1,LOW);
   delayMicroseconds(pd);
}

//Function to rotate the right motor CCW
void TurnCounterClockwise_q2 (int pd)
{
   digitalWrite(DIR2,0);
   digitalWrite(STEP2,HIGH);
   //delayMicroseconds(pd);
   digitalWrite(STEP2,LOW);
   delayMicroseconds(pd);
}

//Function to rotate the left motor CW
void TurnClockwise_q1 (int pd)
{
   digitalWrite(DIR1,1);
   digitalWrite(STEP1,HIGH);
   //delayMicroseconds(pd);
   digitalWrite(STEP1,LOW);
   delayMicroseconds(pd);
}

//Function to rotate the right motor CW
void TurnClockwise_q2 (int pd)
{
   digitalWrite(DIR2,1);
   digitalWrite(STEP2,HIGH);
   //delayMicroseconds(pd);
   digitalWrite(STEP2,LOW);
   delayMicroseconds(pd);
}
