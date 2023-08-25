/*************************************************** 
Created by Brevin Banks August 13 2023
Modified by Brevin Banks August 21 2023
This script will control all 7 servos of the Revyn arm using the PCA9685 driver.
The script can be run by sending an array of 7 numbers into the serial monitor or through
a matlab or python script that sends 7 numbers indicating the state each joint or servo
should move towards. This script interprets the serial input and moves the robot (as long
as those inputs are within acceptable joint limits and the servo torque can handle the position).
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//  Use the address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// General MG996R Servo PWM Limits. Assigning values outside of this range will cause the servo to drop torque
#define SERVOMIN 72 // Lower PWM limit
#define SERVOMAX 545 // Upper PWM limit

// Joint 1 Servo Range
#define BASEZERO 95 // 0 Degrees
#define BASEHOME 295 // 90 Degrees
#define BASE180 497 // 180 Degrees

// Joint 2 Servo Range
#define SHOULDERZERO 131 // 0 Degrees
#define SHOULDERMIN 265 // 60 Degrees - Hardware restriction
#define SHOULDERHOME 310 // 90 Degrees
#define SHOULDERMAX 388 // 120 Degrees - Hardware restriction
#define SHOULDER180 520 // 180 Degrees

// Joint 3 Servo Range
#define ELBOWZERO 115 // 0 Degrees
#define ELBOWHOME 317 // 90 Degrees
#define ELBOW180 500 // 180 Degrees

// Joint 4 Servo Range
#define FOREARMZERO 107 // 0 Degrees
#define FOREARMHOME 310 // 90 Degrees
#define FOREARM180 516 // 180 Degrees

// Joint 5 Servo Range
#define WRISTZERO 110 // 0 Degrees
#define WRISTHOME 307 // 90 Degrees
#define WRIST180 505 // 180 Degrees

// Joint 6 Servo Range
#define WRIST2ZERO 110 // 0 Degrees
#define WRIST2HOME 307 // 90 Degrees
#define WRIST2180 505 // 180 Degrees

// Joint 7 Servo Range
#define EFWIDEOPEN 230 // Open the EF Jaws wide
#define EFHOME 300 // Open the Ef Jaws a samller distance, Default Jaw position
#define EFSLIGHTCLOSED 340 // Open the Ef Jaws a tiny distance
#define EFCLOSED 365 // Close the EF Jaws basically so they are touching
#define EFTIGHTCLOSED 380 // Tight
#define EFTIGHTERCLOSED 420 // max tightness

// Servo Joint number # counter
uint8_t BaseServo = 1;
uint8_t ShoulderServo = 2;
uint8_t ElbowServo = 3;
uint8_t ForearmServo = 4;
uint8_t WristServo = 5;
uint8_t Wrist2Servo = 6;
uint8_t EFServo = 7;

// Servo potentiometer reading values correspond to servo joint numbers
int Sencode1 = A0;
int Sencode2 = A1;
int Sencode3 = A2;
int Sencode4 = A3;
int Sencode5 = A4;
int Sencode6 = A5;
int Sencode7 = A6;

// Starting Joint Values. They will be initiated at the home position which is 
// 90 degrees for every joint
float J1val=90;
float J2val=90;
float J3val=90;
float J4val=90;
float J5val=90;
float J6val=90;

// Joint value variable used to store the current joint value when calculating
// Trajectories
float Current_Jval1 = 90;
float Current_Jval2 = 90;
float Current_Jval3 = 90;
float Current_Jval4 = 90;
float Current_Jval5 = 90;
float Current_Jval6 = 90;

// Starting PWM Joint Values. They will be initiated at the home position which is 
// 90 degrees for every joint
float PWM_Val1 = BASEHOME;
float PWM_Val2 = SHOULDERHOME;
float PWM_Val3 = ELBOWHOME;
float PWM_Val4 = FOREARMHOME;
float PWM_Val5 = WRISTHOME;
float PWM_Val6 = WRIST2HOME;
float PWM_Val7 = EFHOME;

// Each Servo is calibrated with a linear fit between the 0 and 180 Degree angle
// according to output corresponding PWM values. Each slope - m and intercept b
// is used to interpolate a PWM value from a given angle. See 'RevynCalibration.txt'
float m1 = 2.2348;
float b1 = 95.227;
float m2 = 2.0587;
float b2 = 127.5;
float m3 = 2.1602;
float b3 = 120.33;
float m4 = 2.2721;
float b4 = 106.35;
float m5 = 2.1952;
float b5 = 109.76;
float m6 = 2.1952;
float b6 = 109.76;

unsigned long lastRead = 0; // used to count the last read time in milisecond used on the move control loop
int loop_count = 0; // checks to see if the loop has been started or not. Used primarily to initially home the robot

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Robot Controller");
  Serial.print("7 Channel Servo Communication Initiated");
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates. Use 50 to be a little more safe
  Serial.println(": Servos Connected");
  Serial.println("Homing Robot Joints");
}


void loop() {

 if (loop_count==0){ // if this is the first time the loop is running home all the joints
  // The first argument in the setPWM function asks for the servo number corresponding to the 
  // number on the servo driver chip. The numbers start on zero so we subtract 1 to make joint 1 
  // actually 0 and so forth
   home_robot_initial();

 }
 
 else{// move if looped at least once
  if (loop_count>1){
    Serial.println("Pose Achieved");
  }
  Serial.println("Enter Desired Joint Angles");
  while (!Serial.available()) {
  }

  // Read the recieved joint angles/states from the serial monitor
  float J1_new = Serial.parseFloat();
  float J2_new = Serial.parseFloat();
  float J3_new = Serial.parseFloat();
  float J4_new = Serial.parseFloat();
  float J5_new = Serial.parseFloat();
  float J6_new = Serial.parseFloat();
  float J7_new = Serial.parseFloat();
  Serial.read(); // Clear the serial buffer

  // Adjust 0 points from IK and FK to correspond with servo callibration
  J1_new = J1_new+90.0;
  J2_new = J2_new+90.0;
  J3_new = 180.0-J3_new;
  J4_new = J4_new+90.0;
  J5_new = J5_new+90;
  J6_new = J6_new+90;
  
//      Serial.print("Angles Recieved: [");
      Serial.print(J1_new);
      Serial.print(',');
      Serial.print(J2_new);
      Serial.print(',');
      Serial.print(J3_new);
      Serial.print(',');
      Serial.print(J4_new);
      Serial.print(',');
      Serial.print(J5_new);
      Serial.print(',');
      Serial.print(J6_new);
      Serial.print(',');
      Serial.println(J7_new);
//      Serial.print("] ");
  if (J1_new>181.0 || J2_new>181.0 || J3_new>181.0 || J4_new>181.0 || J5_new>181.0 || J6_new>181.0){
      Serial.println(" Joint angle out of range");
//      Serial.print('\n');
      Serial.read();}
  else if (J1_new<-1.0 || J2_new<-1.0|| J3_new<-1.0|| J4_new<-1.0 || J5_new<-1.0 || J6_new<-1.0){
      Serial.println(" Joint angle out of range");
//      Serial.print('\n');
      Serial.read();}
  else if (J2_new>120.0){
      Serial.println(" Joint angle out of range");
//      Serial.print('\n');
      Serial.read();}
  else if (J2_new<60.0){
      Serial.println(" Joint angle out of range");
//      Serial.print('\n');
      Serial.read();}
  else{



  // Calculate and print the corresponding PWM signals for the given joint angles/states
  AngToPWM(J1_new,J2_new,J3_new,J4_new,J5_new,J6_new,J7_new);


  // Store the state of the joints currently before trajectory planning
  Current_Jval1 = J1val;
  Current_Jval2 = J2val;
  Current_Jval3 = J3val;
  Current_Jval4 = J4val;
  Current_Jval5 = J5val;
  Current_Jval6 = J6val;

  // Initiate variables for observing the direction the trajectory is moving (1 = adding to angle, -1= subtracting from angle)
  float sign_flip1 = 1.0;
  float sign_flip2 = 1.0;
  float sign_flip3 = 1.0;
  float sign_flip4 = 1.0;
  float sign_flip5 = 1.0;
  float sign_flip6 = 1.0;
 

  //Trajectory planning
  float Step_speed = 0.25; 
  float Step_count1 = (J1_new-Current_Jval1)/Step_speed;
  float Step_count2 = (J2_new-Current_Jval2)/Step_speed;
  float Step_count3 = (J3_new-Current_Jval3)/Step_speed;
  float Step_count4 = (J4_new-Current_Jval4)/Step_speed;
  float Step_count5 = (J5_new-Current_Jval5)/Step_speed;
  float Step_count6 = (J6_new-Current_Jval6)/Step_speed;

  if (Step_count1<0){
    Step_count1 = Step_count1*-1;
    sign_flip1 = -1.0;}
  if (Step_count2<0){
    Step_count2 = Step_count2*-1;
    sign_flip2 = -1.0;}
  if (Step_count3<0){
    Step_count3 = Step_count3*-1;
    sign_flip3 = -1.0;}
  if (Step_count4<0){
    Step_count4 = Step_count4*-1;
    sign_flip4 = -1.0;}
  if (Step_count5<0){
    Step_count5 = Step_count5*-1;
    sign_flip5 = -1.0;}
  if (Step_count6<0){
    Step_count6 = Step_count6*-1;
    sign_flip6 = -1.0;}

  float assign_ang1 = Current_Jval1;
  float assign_ang2 = Current_Jval2;
  float assign_ang3 = Current_Jval3;
  float assign_ang4 = Current_Jval4;
  float assign_ang5 = Current_Jval5;
  float assign_ang6 = Current_Jval6;
  
  float assign_pwm1 = BASEHOME;
  float assign_pwm2 = SHOULDERHOME;
  float assign_pwm3 = ELBOWHOME;
  float assign_pwm4 = FOREARMHOME;
  float assign_pwm5 = WRISTHOME;
  float assign_pwm6 = WRIST2HOME;

  float k1 = 1.0;
  float k2 = 1.0;
  float k3 = 1.0;
  float k4 = 1.0;
  float k5 = 1.0;
  float k6 = 1.0;

  float all_steps_done = 0;
  // Perform Trajectory assignment
  while(all_steps_done<6){ // Wait for all joints to reach their goals
    if (k1>abs(Step_count1)-1){// if the steps for joint 1 are finished
      all_steps_done = all_steps_done+1; // increase the steps done counter
      k1 = -1;}
    else if(k1<abs(Step_count1) && k1>0){
      assign_ang1 = assign_ang1+(Step_speed)*sign_flip1;
      assign_pwm1 = AngToPWMTraj(assign_ang1,1);
      k1 = k1+1;
      pwm.setPWM(BaseServo-1, 0, assign_pwm1);}
    if (k2>abs(Step_count2)-1){// if the steps for joint 2 are finished
      all_steps_done = all_steps_done+1; // increase the steps done counter
      k2 = -1;}
    else if (k2<abs(Step_count2) && k2>0){
      assign_ang2 = assign_ang2+(Step_speed)*sign_flip2;
      assign_pwm2 = AngToPWMTraj(assign_ang2,2);
      k2 = k2+1;
      pwm.setPWM(ShoulderServo-1, 0, assign_pwm2);}
    if (k3>abs(Step_count3)-1){// if the steps for joint 3 are finished
      all_steps_done = all_steps_done+1; // increase the steps done counter
      k3 = -1;}
    else if (k3<abs(Step_count3) && k3>0){
      assign_ang3 = assign_ang3+(Step_speed)*sign_flip3;
      assign_pwm3 = AngToPWMTraj(assign_ang3,3);
      k3 = k3+1;
      pwm.setPWM(ElbowServo-1, 0, assign_pwm3);}
    if (k4>abs(Step_count4)-1){// if the steps for joint 4 are finished
      all_steps_done = all_steps_done+1; // increase the steps done counter
      k4 = -1;}
    else if (k4<abs(Step_count4) && k4>0){
      assign_ang4 = assign_ang4+(Step_speed)*sign_flip4;
      assign_pwm4 = AngToPWMTraj(assign_ang4,4);
      k4 = k4+1;
      pwm.setPWM(ForearmServo-1, 0, assign_pwm4);}
    if (k5>abs(Step_count5)-1){// if the steps for joint 5 are finished
      all_steps_done = all_steps_done+1; // increase the steps done counter
      k5 = -1;}
    else if (k5<abs(Step_count5) && k5>0){
      assign_ang5 = assign_ang5+(Step_speed)*sign_flip5;
      assign_pwm5 = AngToPWMTraj(assign_ang5,5);
      k5 = k5+1;
      pwm.setPWM(WristServo-1, 0, assign_pwm5);}
    if (k6>abs(Step_count6)-1){// if the steps for joint 6 are finished
      all_steps_done = all_steps_done+1; // increase the steps done counter
      k6 = -1;}
    else if (k6<abs(Step_count6) && k6>0){
      assign_ang6 = assign_ang6+(Step_speed)*sign_flip6;
      assign_pwm6 = AngToPWMTraj(assign_ang6,6);
      k6 = k6+1;
      pwm.setPWM(Wrist2Servo-1, 0, assign_pwm6);}
      delay(30);}

  pwm.setPWM(EFServo-1, 0, PWM_Val7);

        delayMicroseconds(3000);
  J1val = J1_new;
  J2val = J2_new;
  J3val = J3_new;
  J4val = J4_new;
  J5val = J5_new;
  J6val = J6_new;
  Serial.read();
  delay(2);
  }
 }

 
loop_count++;
}


// This line ensures no serial junk gets stuck inbetween sending messages
void waitForSerial(){
  while (!Serial.available()) {
  }
  Serial.println(Serial.read());
}


// This function changes the new joint values for all 7 joints to servo PWM assignment values
// Input
//  NONE
// Output
//  NONE
void AngToPWM(float J1_new,float J2_new,float J3_new,float J4_new,float J5_new,float J6_new,float J7_new){
  // Use the linear regression results to convert angles to pwm
  PWM_Val1 = J1_new*m1 + b1;
  PWM_Val2 = J2_new*m2 + b2;
  PWM_Val3 = J3_new*m3 + b3;
  PWM_Val4 = J4_new*m4 + b4;
  PWM_Val5 = J5_new*m5 + b5;
  PWM_Val6 = J6_new*m6 + b6;
  //for joint 7 use values 1 2 0 and -1 to assign EF state
  if      (J7_new==1.0){
     PWM_Val7 = EFCLOSED;
    }
  else if (J7_new==2.0){
     PWM_Val7 = EFTIGHTCLOSED;
    }
  else if (J7_new==0.0){
     PWM_Val7 = EFHOME;
    }
  else if (J7_new==-1.0){
     PWM_Val7 = EFWIDEOPEN;
    }
  else if (J7_new==3.0){
     PWM_Val7 = EFTIGHTERCLOSED;}
  else if (J7_new==0.5){
     PWM_Val7 = EFSLIGHTCLOSED;}
     
}


//  This function changes the trajectory joint values for a single jointf along the trajectory
//  Angle - the current angle in the trajectory for a joint
//  Joint_num - the corresponding joint to the desired angle
// Output
//  PWM_Val - the calculated pwm value for the respective joint given an angle
float AngToPWMTraj(float Angle, int Joint_num){
 float PWM_Val = 300;
 switch(Joint_num){
  case 1:
    PWM_Val = Angle*m1 + b1;
    break;
  case 2:
     PWM_Val = Angle*m2 + b2;
     break;
  case 3:
     PWM_Val = Angle*m3 + b3;
     break;
  case 4:
     PWM_Val = Angle*m4 + b4;
     break;
  case 5:
     PWM_Val = Angle*m5 + b5;
     break;
  case 6:
     PWM_Val = Angle*m6 + b6;
     break;
  case 7:
     if (Angle==1.0){
       PWM_Val = EFCLOSED;}
     else if (Angle==2.0){
       PWM_Val = EFTIGHTCLOSED;}
     else if (Angle==0.0){
       PWM_Val = EFHOME;}
     else if (Angle==-1.0){
       PWM_Val = EFWIDEOPEN;}
     else if (Angle==3.0){
       PWM_Val = EFTIGHTERCLOSED;}
     else if (Angle==0.5){
       PWM_Val = EFSLIGHTCLOSED;}
     break;
  default:
     break;
}
 return PWM_Val;
}


// This function homes the robot on the first loop of the system.
// because no reference joint values are used to home the robot,
// The servos will jump quickly to the home position. Be careful on startup
// Input
//  NONE
// Output
//  NONE
void home_robot_initial(){
  pwm.setPWM(BaseServo-1, 0, (BASEHOME));
  pwm.setPWM(ShoulderServo-1, 0, (SHOULDERHOME));
  pwm.setPWM(ElbowServo-1, 0, (ELBOWHOME));
  pwm.setPWM(ForearmServo-1, 0, (FOREARMHOME));
  pwm.setPWM(WristServo-1, 0, (WRISTHOME));
  pwm.setPWM(Wrist2Servo-1, 0, (WRIST2HOME));
  pwm.setPWM(EFServo-1, 0, (EFHOME));
  Serial.read(); // Clear the serial buffer
  // Assign all Joint angle values to joint variables
  J1val=90;
  J2val=90;
  J3val=90;
  J4val=90;
  J5val=90;
  J6val=90;
}

// This function homes the robot at any point during control
// Input
//  NONE
// Output
//  NONE
void home_robot(){
  pwm.setPWM(BaseServo-1, 0, (BASEHOME));
  pwm.setPWM(ShoulderServo-1, 0, (SHOULDERHOME));
  pwm.setPWM(ElbowServo-1, 0, (ELBOWHOME));
  pwm.setPWM(ForearmServo-1, 0, (FOREARMHOME));
  pwm.setPWM(WristServo-1, 0, (WRISTHOME));
  pwm.setPWM(Wrist2Servo-1, 0, (WRIST2HOME));
  pwm.setPWM(EFServo-1, 0, (EFHOME));
  Serial.read(); // Clear the serial buffer
  // Assign all Joint angle values to joint variables
  J1val=90;
  J2val=90;
  J3val=90;
  J4val=90;
  J5val=90;
  J6val=90;
}
