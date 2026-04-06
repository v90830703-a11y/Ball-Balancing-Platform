#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Servo.h>

//put inputs here
float height = 200; //default screen plane level //ideally between 165 and rouphly bc tliting plate reqiures one arm to extend more than the others
int thetaX = 0; //plane tilt around x (from y to z) in degrees
int thetaY = 0; //tilt around y (from z to x) in degrees

//inputs end




// Pins
const int servoPin1 = 2;
const int servoPin2 = 3;
const int servoPin3 = 4;
const int buttonPin1 = 5;
const int buttonPin2 = 6;

const int PinX = A0;
const int PinY = A1;

// Per-servo degree offsets
const int offset[] = {15, 15, 10};

// Servo Arm Angle limits
const int minAngle  = 0;
const int softLimit = 90;
const int maxAngle  = 270;

const int minThetaX = -15;
const int maxThetaX = 15;
const int minThetaY = -15;
const int maxThetaY = 15;


//unchangeable structural dimension values
const float a = 100;//lower arm length
const float b = 130;//upper arm length
const float l = 89/sqrt(3);//at button surface distance a=OA=OD=OG
const float d = 150/sqrt(3);//at upper surface distance l=HC=HF=HI
const float hh = 16.5;//different than h, this is the difference in height between touch screen top surface plane and plane CFI

//height = O-HH, h=O-H, hh = H-HH  //set default height here
float h = height-hh;
const int heightmin = 113.5+hh;//160//
const int heightmax = 223.5+hh;//240//max h is when arms are straight definitely < 100+130 because they are not vertical. manually set max = 223.5
//the height does not include consideration of tiltin --- it is for when the plate is horizontal
 




//start linear algebra namespace
using namespace BLA;

// Servo objects
Servo myservo1;
Servo myservo2;
Servo myservo3;

//Servo angles 
float Theta1;
float Theta2;
float Theta3;


//set origin coord//target point where you want the ball to reach equilibrium at
int Xo = 0;
int Yo = 0;
float Xl;
float Yl;
float sumErrorX;
float sumErrorY;


//PID tuning
const float Kpx = 15;
const float Kpy = 15;
const float Kix = 0;
const float Kiy = 0;
const float Kdx = 0;
const float Kdy = 0;




//calculation rate
float rate = 100;
unsigned long  period = 1000/rate; //in ms




//finish declaration of value variables and constants





//sum of deviation from set origin
float sumErrorOperation(float currentSumError, float error){
  return currentSumError + error;
}



// Convert logical angle + per-servo offset → microseconds
// Returns 0 and prints error if out of range
int PWMConvert(int servoNumber, int angle) {
  int adjusted = angle + offset[servoNumber - 1];
  if (angle >= minAngle && angle <= softLimit) {
    return map(adjusted, 0, 270, 500, 2500);
  }
  Serial.print("Angle Overshoot Error: servo=");
  Serial.print(servoNumber);
  Serial.print(" angle=");
  Serial.println(angle);
  return 0;
}



// Returns true when button is pressed (INPUT_PULLUP, so LOW = pressed)
bool readButton(int pin) {
  return digitalRead(pin) == LOW;
}




// Safe write — ignores invalid PWM values
void writeServo(Servo &servo, int pwm) {
  if (pwm > 0) {
    servo.writeMicroseconds(pwm);
  }
}




// Read and average analog inputs
int readScreen(int whatPin) {
  long sum = 0;

  for (int i = 0; i < 50; i++) {
    sum += analogRead(whatPin) * 5 / 3.3;
  }

  if (whatPin == PinY){
    sum = sum*600/1024;
  }
  else if(whatPin == PinX){
    sum = sum*800/1024;
  } else{Serial.println("Error");}

  int value = sum / 50;

  if (value < 10) value = 0;

  return value;
}




float safeAcos(float x){
  return acos(constrain(x,-1.0,1.0));
}





void matrixOperations(int thetaX, int thetaY){


  //from here, linear algebra
  //convert to radians
  float RthetaX = thetaX*PI/180.0;
  float RthetaY = thetaY*PI/180.0;

  //matrices declaration
  BLA::Matrix<3> HC = {-150.0/2, -150.0/sqrt(3)/2,0};
  BLA::Matrix<3> HF = {0,d,0};
  BLA::Matrix<3> HI = {150.0/2,-150.0/sqrt(3)/2,0};

  //homogeneous tranformation matrices

  BLA::Matrix<4,4> T1 = {1,0,0,0, 
                        0,cos(RthetaX), -sin(RthetaX),0
                        ,0,sin(RthetaX),cos(RthetaX),h,
                        0,0,0,1};
  BLA::Matrix<4,4> R2 = {cos(RthetaY),0,-sin(RthetaY),0,
                        0,1,0,0,
                        sin(RthetaY),0,cos(RthetaY),0,
                        0,0,0,1};

  //combination T1*R2
  BLA::Matrix<4,4> Tc = T1*R2;

  //adding fourth dimension to HC,HF,HI to make them 4*1
  BLA::Matrix<1> wutIsThis = {1};
  BLA::Matrix<4,1> fourHC = HC && wutIsThis;
  BLA::Matrix<4,1> fourHF = HF && wutIsThis;
  BLA::Matrix<4,1> fourHI = HI && wutIsThis;

  //use Tc transformation to find transformed vectors HC, HF, HI
  BLA::Matrix<4,1> fourOC = Tc*fourHC;
  BLA::Matrix<4,1> fourOF = Tc*fourHF;
  BLA::Matrix<4,1> fourOI = Tc*fourHI;

  //truncate 4th dimension
  BLA::Matrix<3,4> FourProjectionToThree= {1,0,0,0,
                                          0,1,0,0,
                                          0,0,1,0}; //declare 3*4 projection matrix

  BLA::Matrix<3> threeOC = FourProjectionToThree * fourOC;
  BLA::Matrix<3> threeOF = FourProjectionToThree * fourOF;
  BLA::Matrix<3> threeOI = FourProjectionToThree * fourOI;

  //declare OA, OD, OG
  BLA::Matrix<3> OA = {-89.0/2, -89.0/4,0};
  BLA::Matrix<3> OD = {0,l,0};
  BLA::Matrix<3> OG = {89.0/2,-89.0/4,0};

  //find AC, DF, GI
  BLA::Matrix<3> AC = threeOC-OA;
  BLA::Matrix<3> DF = threeOF-OD;
  BLA::Matrix<3> GI = threeOI-OG;


  //angles used in calculation
  float angleAlphaA;
  float angleThetaA;

  float angleAlphaD;
  float angleThetaD;

  float angleAlphaF;
  float angleThetaF;

  //find angle between OA-AC,OD-DF,OG-GI,      and find angle between AB-AC, DE-DF, GH-GI
  float dot = AC(0)*OA(0) + AC(1)*OA(1) + AC(2)*OA(2);
  float mag2 = sqrt(pow(AC(0),2) + pow(AC(1),2) + pow(AC(2),2));
  float mag1 = sqrt(pow(OA(0),2) + pow(OA(1),2) + pow(OA(2),2));
  angleAlphaA = safeAcos(dot / (mag1 * mag2));
  angleThetaA = safeAcos((pow(mag2,2)+pow(a,2)-pow(b,2))/(2*a*mag2));

  dot = OD(0)*DF(0) + OD(1)*DF(1) + OD(2)*DF(2);
  mag1 = sqrt(pow(OD(0),2) + pow(OD(1),2) + pow(OD(2),2));
  mag2 = sqrt(pow(DF(0),2) + pow(DF(1),2) + pow(DF(2),2));
  angleAlphaD = safeAcos(dot / (mag1 * mag2));
  angleThetaD = safeAcos((pow(mag2,2)+pow(a,2)-pow(b,2))/(2*a*mag2));

  dot = OG(0)*GI(0) + OG(1)*GI(1) + OG(2)*GI(2);
  mag1 = sqrt(pow(OG(0),2) + pow(OG(1),2) + pow(OG(2),2));
  mag2 = sqrt(pow(GI(0),2) + pow(GI(1),2) + pow(GI(2),2));
  angleAlphaF = safeAcos(dot / (mag1 * mag2));
  angleThetaF = safeAcos((pow(mag2,2)+pow(a,2)-pow(b,2))/(2*a*mag2));



  //finally angles between OA-AB, OD-DE, OF-FG, they are exactly the angles for the three servos
  Theta1 = (angleAlphaA - angleThetaA)/PI*180;
  Theta2 = (angleAlphaD - angleThetaD)/PI*180;
  Theta3 = (angleAlphaF - angleThetaF)/PI*180;
  //add constrains to prevent overshoot
  Theta1 = constrain(Theta1, 0, softLimit);
  Theta2 = constrain(Theta2, 0, softLimit);
  Theta3 = constrain(Theta3, 0, softLimit);

}


void setup() {
  Serial.begin(115200);
  delay(2000);

  // Attach the 3 servos to pin 2,3,4. Specify min and max pulse widths in microseconds.
  // This defines the full range for your specific 270-degree servo.
  // 500 us is typically 0 degrees, 2500 us is typically 270 degrees.
  myservo1.attach(servoPin1, 500, 2500); 
  myservo2.attach(servoPin2, 500, 2500); 
  myservo3.attach(servoPin3, 500, 2500); 

  //add pinMode()

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);



  //initial setup push up to 80 degrees 
  int pwm1 = PWMConvert(1, 80);
  int pwm2 = PWMConvert(2, 80);
  int pwm3 = PWMConvert(3, 80);
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width
  delay(1500);

  //return to 0 height
  pwm1 = PWMConvert(1, 0);
  pwm2 = PWMConvert(2, 0);
  pwm3 = PWMConvert(3, 0);
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width
  delay(1500);

  //return to default height;
  matrixOperations(0,0);
  pwm1 = PWMConvert(1, int(Theta1));
  pwm2 = PWMConvert(2, int(Theta2));
  pwm3 = PWMConvert(3, int(Theta3));
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width




  //timer should initialize here:
  unsigned long initialTime = millis();
  

  //initialize error
  sumErrorX = 0;
  sumErrorY = 0;
  Xl = -(readScreen(PinX)-400)/400.0;
  Yl = (readScreen(PinY)-300)/300.0;

  //wait till one period
  while (millis()-initialTime < period){}

}



//LOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
void loop() {

  //timer should reset here:
  unsigned long initialTime = millis();


  //PID control
  float Xcoord = -(readScreen(PinX)-400)/400.0;
  float Ycoord = (readScreen(PinY)-300)/300.0;

  Serial.println(Xcoord);
  Serial.println(Ycoord);
  sumErrorX = sumErrorOperation(sumErrorX, Xcoord-Xo);
  sumErrorY = sumErrorOperation(sumErrorY, Ycoord-Yo);
  sumErrorX = constrain(sumErrorX, -5, 5);
  sumErrorY = constrain(sumErrorY, -5, 5);

    //speed in x and y directions for PID
  float speedX = (Xcoord - Xl)/period*1000;
  float speedY = (Ycoord - Yl)/period*1000;
  //final thetaX and thetaY to put into matrix calculations
  thetaX = (Ycoord-Yo)*Kpy + speedY * Kdy + sumErrorY * Kiy;
  thetaY = (Xcoord-Xo)*Kpx + speedX * Kdx + sumErrorX * Kix;


  if (abs(Xcoord) < 0.02) thetaY = 0;
  if (abs(Ycoord) < 0.02) thetaX = 0;

  //constrain the angles
  thetaX = constrain(thetaX, minThetaX, maxThetaX);
  thetaY = constrain(thetaY, minThetaY, maxThetaY);


  //linear algebra magic
  matrixOperations(thetaX, thetaY);
  //so that the variables theta1-3 are updated


  //pwm convertion and actuator write, using the theta1,2,3 values obtained from previous function


  int pwm1 = PWMConvert(1, int(Theta1));
  int pwm2 = PWMConvert(2, int(Theta2));
  int pwm3 = PWMConvert(3, int(Theta3));
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width



  //update values for next loop
  Xl = Xcoord;
  Yl = Ycoord;
    
    
  //wait till one period
  while (millis()-initialTime < period){}


}


