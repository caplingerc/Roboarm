
/* 
 Robot arm control using hacked servos and a master arm with pots as control in
 master-slave system.
 
 +y is vertical, +x is to the right
 
 drawing line/circle:
 http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 
 inverse kinematics:
 http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 http://www.learnaboutrobots.com/inverseKinematics.htm
 */

#include <AFMotor.h>
#include <math.h>
#include <PID_v1.h>
#include <IVSense.h>

// Instantiate both motors
AF_DCMotor shoulder(3);  
AF_DCMotor elbow(4);

// declare pins
int ElbowPin = A1;  // to potentiometer on elbow motor
int ShoulderPin = A0; // to potentiometer on shoulder motor

// INITIALIZE CONSTANTS
double Kp_Elbow = 20;    // this is the proportional gain
double Kp_Shoulder = 20; 
double Ki_Elbow = 0.1;    // this is the integral gain
double Ki_Shoulder = 0.1; 
double Kd_Elbow = 1.0;  // this is the derivative gain
double Kd_Shoulder = 0.75;

double Elbow_neg = 970;      // joint limits of robotic arm using right hand rule for sign
double Elbow_pos = 31;
double Shoulder_neg = 210;
double Shoulder_pos = 793; 

const double a1 = 200;     // shoulder-to-elbow "bone" length (mm)
const double a2 = 220;     // elbow-to-wrist "bone" length (mm) - longer c bracket

double highY = 350; // line drawing targets
double lowY = 250; 
double constantX = 200;

boolean elbowup = false; // true=elbow up, false=elbow down

// // VOLTAGE & CURRENT SENSE CONSTANTS/VARIABLES // // 

const int analogElbowVoltagePin = A4;
const int analogShoulderVoltagePin = A5;
const int analogElbowCurrentPin = A2;
const int analogShoulderCurrentPin = A3;
const int rScale = 1;

IVSense elbow_sensor(analogElbowCurrentPin, analogElbowVoltagePin, rScale);
IVSense shoulder_sensor(analogShoulderCurrentPin, analogShoulderVoltagePin, rScale);

// // // // // // // // // // // // // // // // // // 

// VARIABLES
double rawElbowAngle = 0.0;      // initialize all angles to 0 
double rawShoulderAngle = 0.0; 

double elbowAngle = 0.0;      // initialize all angles to 0 
double shoulderAngle = 0.0;   

double theta1 = 0.0;  // target angles as determined through IK
double theta2 = 0.0;

double actualX = 0.0;
double actualY = 0.0;
double y = 0.0;

double c2 = 0.0; // is btwn -1 and 1
double s2 = 0.0;

double pwmTemp = 0.0;

double pwmElbow = 50.0;  // between 0 to 255
double pwmShoulder = 50.0;

//Syntax: PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID elbowPID(&elbowAngle, &pwmElbow, &theta2, Kp_Elbow, Ki_Elbow, Kd_Elbow, DIRECT);
PID shoulderPID(&shoulderAngle, &pwmShoulder, &theta1, Kp_Shoulder, Ki_Shoulder, Kd_Shoulder, DIRECT);

int highV = 0;


void setup() {  //happens once
  Serial.begin(115200);           // set up Serial library 

  elbowPID.SetSampleTime(5);  // (ms) how often new PID calc is performed, default is 1000
  shoulderPID.SetSampleTime(5);

  elbow.setSpeed(pwmElbow);     // set the speed to pwmElbow
  shoulder.setSpeed(pwmShoulder);     // set the speed to pwmElbow

  elbowPID.SetMode(AUTOMATIC);
  shoulderPID.SetMode(AUTOMATIC);

  elbowPID.SetOutputLimits(-255,255);
  shoulderPID.SetOutputLimits(-255,255);

  move_to_start();  //get to starting position
  //line_y();
}

void loop() {  // happens forever
  line_y();
}

void move_to_start() {
  do {  
    get_angles(constantX, lowY);
    drive_joints();  // drive joints until actual equals expected 
  }  
  while( abs(theta1 - shoulderAngle) > 2 && abs(theta2 - elbowAngle) > 2 ); // bail when it's close
}

void line_y() {
  for(y = lowY; y < highY; y += 1) {  // draw straight line up
    get_angles(constantX,y);
    drive_joints();
  }
  for(y = highY; y > lowY; y -= 1) {  // draw straight line down
    get_angles(constantX, y);
    drive_joints();
  }
}

void get_xy() {
  actualX = degrees(a1*cos(radians(theta1)) + a2*cos(radians(theta1+theta2)));
  actualY = degrees(a1*sin(radians(theta1)) + a2*sin(radians(theta1+theta2)));
}

// Given target(Px, Py) solve for theta1, theta2 (inverse kinematics)
void get_angles(double Px, double Py) {
  // first find theta2 where c2 = cos(theta2) and s2 = sin(theta2)
  c2 = (pow(Px,2) + pow(Py,2) - pow(a1,2) - pow(a2,2))/(2*a1*a2); // is btwn -1 and 1

  if (elbowup == false) {
    s2 = sqrt(1 - pow(c2,2));  // sqrt can be + or -, and each corresponds to a different orientation
  }
  else if (elbowup == true) {
    s2 = -sqrt(1 - pow(c2,2));
  }
  theta2 = degrees(atan2(s2,c2));  // solves for the angle in degrees and places in correct quadrant

  // now find theta1 where c1 = cos(theta1) and s1 = sin(theta1)
  theta1 = degrees(atan2(-a2*s2*Px + (a1 + a2*c2)*Py, (a1 + a2*c2)*Px + a2*s2*Py)); 
}

void drive_joints() {
  // read the actual values from all the pots
  rawElbowAngle = analogRead(ElbowPin);
  rawShoulderAngle = analogRead(ShoulderPin);

  // constrain robot arm to ignore out of range values
  elbowAngle = constrain(rawElbowAngle, Elbow_pos, Elbow_neg);
  shoulderAngle = constrain(rawShoulderAngle, Shoulder_neg, Shoulder_pos);

  // now map the angles so they correspond correctly
  elbowAngle = map(elbowAngle, Elbow_neg, Elbow_pos, -110.0, 85.0);  
  shoulderAngle = map(shoulderAngle, Shoulder_neg, Shoulder_pos, 5.0, 135.0);  

  elbowPID.Compute();
  shoulderPID.Compute();  

  // DRIVE ELBOW: CW is forward, CCW is backward
  pwmTemp = abs(pwmElbow);   
  elbow.setSpeed(pwmTemp);     // now use the new PID output to set the speed 

  if (pwmElbow < 0) {
    elbow.run(FORWARD);      // turn it on   
  }
  else if ( pwmElbow > 0) { 
    elbow.run(BACKWARD);      // turn it on  
  }
  else elbow.run(RELEASE);      // stopped

  elbow_sensor.readData();   // elbow joint sensing  

  // DRIVE SHOULDER: CCW is forward, CW is backward
  pwmTemp = abs(pwmShoulder);
  shoulder.setSpeed(pwmTemp);     // set the speed 

  if (pwmShoulder < 0) {
    shoulder.run(BACKWARD);      // turn it on 
  }
  else if (pwmShoulder > 0) { 
    shoulder.run(FORWARD);      // turn it on 
  }
  else  shoulder.run(RELEASE);      // stopped   
  
  shoulder_sensor.readData();      // shoulder joint sensing
  
  //highV = analogRead(HighPin);
  //get_xy();
  
  
  // send current data serially
  Serial.print(  millis() );
  Serial.print(',');
  /*
  Serial.print(elbowAngle);
  Serial.print(',');
  Serial.print(shoulderAngle);
  Serial.print(',');
  Serial.print(pwmElbow);
  Serial.print(',');
  Serial.print(pwmShoulder);
  Serial.print(',');
  */
  Serial.print(elbow_sensor.getCurrent(), BIN);
  Serial.print(',');
  Serial.print(elbow_sensor.getVoltage(), BIN);
  Serial.print(',');
  Serial.print(shoulder_sensor.getCurrent(), BIN);
  Serial.print(',');
  Serial.println(shoulder_sensor.getVoltage(), BIN);
  

  // need to know time, angle for velocity calcs
 /*
  Serial.print(millis());
  Serial.print(" , ");
  Serial.print("Elbow: ");
  Serial.print(" , ");
  Serial.print(elbowAngle);
  Serial.print(" , ");
  Serial.print(theta2);
  Serial.print(" , ");
  Serial.print("Shoulder: ");
  Serial.print(" , ");
  Serial.print(shoulderAngle);
  Serial.print(" , ");
  Serial.print(theta1);
  Serial.print(" , ");
  Serial.print("Voltage:");
  Serial.print(" , ");
  Serial.print(highV);
  Serial.print(" , ");
  Serial.print("X:");
  Serial.print(" , ");
  Serial.print(actualX);
  Serial.print(" , ");
  Serial.print("Y: ");
  Serial.print(" , ");
  Serial.print(y);
  Serial.print(" , ");
  Serial.println(actualY);
*/
}










