#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Servo.h>

//put intermediate-input-variables here
float height = 200; //default screen plane level //ideally between 165 and rouphly bc tliting plate reqiures one arm to extend more than the others
int thetaX = 0; //plane tilt around x (from y to z) in degrees
int thetaY = 0; //tilt around y (from z to x) in degrees

///////////////
//inputs end//
//////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pins & Buttons
const int servoPin1 = 2;
const int servoPin2 = 3;
const int servoPin3 = 4;
const int buttonPin1 = 5;
const int buttonPin2 = 6;

const int PinX = A0;
const int PinY = A1;

#define XP 7   // Green
#define XM A8  // Blue
#define YP A9  // Yellow
#define YM 8   // Orange


const float Xmax = 950;
const float Xmin = 200;
const float Ymax = 900;
const float Ymin = 200;

// Per-servo degree offsets
const int offset[] = {14,16,11};//14,14,10

// Servo Arm Angle limits
const int minAngle  = 0;
const int softLimit = 90;

const int minPlateAngle = -15;
const int maxPlateAngle = 15;


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
 

//how long in pixels does the ball roll for each edge, when in Mode 2: geometrical shape framing movement
const int linearLength = 700;
const int TriangularEdgeLength = 500;
const int SqaureEdgeLength = 500;
const int CircularRadius = 250;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
float lastTheta1;
float lastTheta2;
float lastTheta3;

//initialization states
bool isInitialized = false;  // so it will do a one-time initialization (the up-down motion of plate)
bool isReset = false;  //utilzed this to prevent detaching servos on repeat
bool isResetForMode2 = false;
bool isExtremeAngle;

//Set button states (using counts of presses of buttons)
int button1PressCount = 1; //treat first time as already pressed
int button2PressCount = 0;
int lastState1 = 0;
int lastState2 = 0;



//set origin coord//target point where you want the ball to reach equilibrium at
int Xo = -0.1;
int Yo = -0.2;
float Xl;
float Yl;
int desiredX = 0;
int desiredY = 0;
int lastDesiredX = 0;
int lastDesiredY = 0;
float sumErrorX;
float sumErrorY;


//PID tuning
const float Kpx = 2.55;
const float Kpy = 2.55;
const float Kix = 0.45;
const float Kiy = 0.45;
const float Kdx = 2.45;
const float Kdy = 2.45;

//cycle rate
float rate = 50;
unsigned long  period = 1000/rate; //in ms
long freezeInitialTime;


const float alpha = 1; // smoothing factor



/////////////////////////////////////////////////////////
//finish declaration of value variables and constants////
/////////////////////////////////////////////////////////

//Start functions/methods

//sum of deviation from set origin
float sumErrorOperation(float currentSumError, float error){
  return currentSumError + error;
}



// Convert logical angle + per-servo offset → microseconds
// Returns 0 and prints error if out of range
// int PWMConvert(int servoNumber, int angle) {
//   int adjusted = angle + offset[servoNumber - 1];
//   if (angle >= minAngle && angle <= softLimit) {
//     Serial.println(map(adjusted, 0, 270, 500, 2500));
//     return constrain(map(adjusted, 0, 270, 500, 2500),0,map(softLimit,0,270,500,2500));
//   }
//   Serial.print("Angle Overshoot Error: servo=");
//   Serial.print(servoNumber);
//   Serial.print(" angle=");
//   Serial.println(angle);
//   return 0;
// }


int PWMConvert(int servoNumber, int angle) {
  int adjusted = angle + offset[servoNumber - 1];

  if (angle >= minAngle && angle <= softLimit) {
    float pwm = 500.0 + (adjusted / 270.0) * 2000.0;
    return constrain((int)pwm, 500, 2500);
  }

  return 0;
}



// Returns true when button is pressed (INPUT_PULLUP, so LOW = pressed)
void buttonCountUpdate(int buttonPin) {

  int &lastState = (buttonPin == buttonPin1) ? lastState1 : lastState2;
  int thisState = digitalRead(buttonPin);

  if (thisState == 0) {thisState = 1;}//flip low to high
  else if (thisState == 1) {thisState = 0;}//flip high to low

  if (thisState != lastState){
    if (thisState == 1){

      if (buttonPin == 5) {++button1PressCount;Serial.println(button1PressCount);}
      else if (buttonPin == 6) {++button2PressCount;Serial.println(button2PressCount);}
      

    }
  }
  lastState = thisState;//update lastState for next round
}





// Safe write — ignores invalid PWM values
void writeServo(Servo &servo, int pwm) {
  if (pwm > 0) {
    servo.writeMicroseconds(pwm);
  }
}




// Read and average analog inputs
int readScreen(int whatPin) {
  if (whatPin == PinX) {
    pinMode(YP, OUTPUT); digitalWrite(YP, LOW);
    pinMode(YM, OUTPUT); digitalWrite(YM, HIGH);

    pinMode(XM, INPUT);
    pinMode(XP, INPUT);



    float sumXcoord = 0;
    for (int i = 0; i<10; ++i){
      float xValue = analogRead(XM);  // read along X layer
      xValue = constrain(xValue, Xmin, Xmax);
      float Xcoord = (xValue-Xmin)/(Xmax-Xmin)*800.00;
      //constrain(Xcoord,0,800);
      sumXcoord+=Xcoord;
    }
    int theXvalue = sumXcoord/10;
    if (theXvalue > 790 ){theXvalue=400;}
    Serial.println(theXvalue);
    delay(10);
    return theXvalue;}

  if (whatPin == PinY) {
    pinMode(XP, OUTPUT); digitalWrite(XP, HIGH);
    pinMode(XM, OUTPUT); digitalWrite(XM, LOW);

    pinMode(YP, INPUT);
    pinMode(YM, INPUT);

    float sumYcoord = 0;
    for (int i = 0; i<10; ++i){
      float yValue = analogRead(YP);  // read along X layer
      yValue = constrain(yValue, Ymin, Ymax);
      float Ycoord = (yValue-Ymin)/(Ymax-Ymin)*600.00;
      //constrain(Ycoord,0,600);
      sumYcoord+=Ycoord;
    }

    int theYvalue = sumYcoord/10;
    if (theYvalue >590){theYvalue=300;}
    Serial.println(theYvalue);
    delay(10);
    return theYvalue;}



}



//do a constrain so there is no scenario that requires acos(x) operation where |x| > 1
float safeAcos(float x){
  return acos(constrain(x,-1.0,1.0));
}




void matrixOperations(int thetaX, int thetaY){
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



//PID
void PID(int desiredX, int desiredY, float Xcoord, float Ycoord, float Xnorm, float Ynorm){
  // Calculate error
  float errorX = Xcoord - desiredX;
  float errorY = Ycoord - desiredY;

  // Integral term (clamped to prevent wind-up)
  sumErrorX += errorX;
  sumErrorY += errorY;
  sumErrorX = constrain(sumErrorX, -1, 1);
  sumErrorY = constrain(sumErrorY, -1, 1);

  // ---------------------------
  // Derivative term (smoothed)
  const int N = 5; // moving average window size
  static float xHistory[N] = {0};
  static float yHistory[N] = {0};

  // shift old values
  for(int i=N-1; i>0; i--) xHistory[i] = xHistory[i-1];
  for(int i=N-1; i>0; i--) yHistory[i] = yHistory[i-1];

  // add newest normalized value
  xHistory[0] = Xnorm;
  yHistory[0] = Ynorm;

  // derivative = difference between newest and oldest
  float dX = (xHistory[0] - xHistory[N-1]) / (N * period / 1000.0);
  float dY = (yHistory[0] - yHistory[N-1]) / (N * period / 1000.0);

  // clamp derivative contribution
  float maxDerivative = 10.0; // tweak between 5~15 for response vs jitter
  dX = constrain(Kdx * dX, -maxDerivative, maxDerivative);
  dY = constrain(Kdy * dY, -maxDerivative, maxDerivative);

  // ---------------------------
  // PID output to plate angles
  thetaX = (Ycoord - desiredY) * Kpy + dY + sumErrorY * Kiy;
  thetaY = (Xcoord - desiredX) * Kpx + dX + sumErrorX * Kix;

  // constrain plate tilt
  thetaX = constrain(thetaX, minPlateAngle, maxPlateAngle);
  thetaY = constrain(thetaY, minPlateAngle, maxPlateAngle);

  // ---------------------------
  // Update servo angles
  matrixOperations(thetaX, thetaY);

  if(abs(Theta1 - lastTheta1) > 0.15){
      writeServo(myservo1, PWMConvert(1, int(Theta1)));
      lastTheta1 = Theta1;
  }
  if(abs(Theta2 - lastTheta2) > 0.15){
      writeServo(myservo2, PWMConvert(2, int(Theta2)));
      lastTheta2 = Theta2;
  }
  if(abs(Theta3 - lastTheta3) > 0.15){
      writeServo(myservo3, PWMConvert(3, int(Theta3)));
      lastTheta3 = Theta3;
  }
}





void initialization(){ // be written as 'initialize if isInitialized is false'
  // Attach the 3 servos to pin 2,3,4. Specify min and max pulse widths in microseconds.
  // This defines the full range for your specific 270-degree servo.
  // 500 us is typically 0 degrees, 2500 us is typically 270 degrees.
  // myservo1.attach(servoPin1, 500, 2500); 
  // myservo2.attach(servoPin2, 500, 2500); 
  // myservo3.attach(servoPin3, 500, 2500); 

  //return to default height;
  matrixOperations(0,0);
  int pwm1 = PWMConvert(1, int(Theta1));
  int pwm2 = PWMConvert(2, int(Theta2));
  int pwm3 = PWMConvert(3, int(Theta3));
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width

  //after the first one, every initialization (ny pressing button 1) tilts the plate around 720 degree x and y axis4
  double stepAngle = 2*PI/100; //step angle in degrees converted to radians
  for (double i=0;i<3*PI;i+=stepAngle){
    thetaX = constrain(sin(i)*maxPlateAngle,minPlateAngle,maxPlateAngle);
    thetaY = constrain(cos(i)*maxPlateAngle,minPlateAngle,maxPlateAngle);
    matrixOperations(thetaX,thetaY);
    pwm1 = PWMConvert(1, int(Theta1));
    pwm2 = PWMConvert(2, int(Theta2));
    pwm3 = PWMConvert(3, int(Theta3));
    writeServo(myservo1,pwm1); // Set servo position using pulse width
    writeServo(myservo2,pwm2); // Set servo position using pulse width
    writeServo(myservo3,pwm3); // Set servo position using pulse width
    delay(15);//so it takes approx~ 3.6s to complete one full 720-degree-tilt
  }
  delay(350);


  //return to default 0 tilt;
  matrixOperations(0,0);
  pwm1 = PWMConvert(1, int(Theta1));
  pwm2 = PWMConvert(2, int(Theta2));
  pwm3 = PWMConvert(3, int(Theta3));
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width




  //timer should INITIALIZE here:
  unsigned long initialTime = millis();
  
  //initialize error
  sumErrorX = 0;
  sumErrorY = 0;
  Xl = -(readScreen(PinX)-400)/400.0;
  Yl = (readScreen(PinY)-300)/300.0;

  isInitialized = true;
  isReset = false;

  //wait till one period
  while (millis()-initialTime < period){}

}

void resetAndDeinitialize(){
  int pwm1 = PWMConvert(1, 0);
  int pwm2 = PWMConvert(2, 0);
  int pwm3 = PWMConvert(3, 0);
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width    

  delay(500);

  // //detach the servos  // it does not detach anyways
  // myservo1.detach(); 
  // myservo2.detach(); 
  // myservo3.detach(); 

  //reset initialization status and other statuses
  isInitialized = false;
  isReset = true;
  button2PressCount = 0;
}




void setup() {
  delay(1500);
  Serial.begin(115200);//delete after code completed and debugged

  int pwm1;
  int pwm2;
  int pwm3;

  //add buttons' pinMode

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

///////////////////////////////////////////////////////////////////////////////
  //Only do this up-down motion once for every plug-in-and-out of wall outlet//


  pwm1 = PWMConvert(1, 0);
  pwm2 = PWMConvert(2, 0);
  pwm3 = PWMConvert(3, 0);
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width


  // Attach the 3 servos to pin 2,3,4. Specify min and max pulse widths in microseconds.
  // This defines the full range for your specific 270-degree servo.
  // 500 us is typically 0 degrees, 2500 us is typically 270 degrees.
  myservo1.attach(servoPin1, 500, 2500); 
  myservo2.attach(servoPin2, 500, 2500); 
  myservo3.attach(servoPin3, 500, 2500); 

  delay(1000);

  pwm1 = PWMConvert(1, 90);
  pwm2 = PWMConvert(2, 90);
  pwm3 = PWMConvert(3, 90);
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width

  delay(1500);

  pwm1 = PWMConvert(1, 0);
  pwm2 = PWMConvert(2, 0);
  pwm3 = PWMConvert(3, 0);
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width

  delay(1500);

////////////////////////////////////////////////////////////////////////////////


  //return to default height;
  matrixOperations(0,0);
  lastTheta1 = Theta1;
  lastTheta2 = Theta2;
  lastTheta3 = Theta3;
  pwm1 = PWMConvert(1, int(Theta1));
  pwm2 = PWMConvert(2, int(Theta2));
  pwm3 = PWMConvert(3, int(Theta3));
  writeServo(myservo1,pwm1); // Set servo position using pulse width
  writeServo(myservo2,pwm2); // Set servo position using pulse width
  writeServo(myservo3,pwm3); // Set servo position using pulse width

 
  //timer should INITIALIZE here:
  unsigned long initialTime = millis();
  

  //initialize error
  sumErrorX = 0;
  sumErrorY = 0;
  Xl = -(readScreen(PinX)-400)/400.0;
  Yl = (readScreen(PinY)-300)/300.0;

  //wait till one period
  while (millis()-initialTime < period){}

  isInitialized = true;
  freezeInitialTime = millis();
}



//LOOOOOOOOOOOOOOOOOOOOOOOOOOOOP here it goes the majority of the process flow chart
void loop() {
  //read buttons they are important!
  buttonCountUpdate(buttonPin1);
  buttonCountUpdate(buttonPin2);


  //logic flow according to button statuses
  if (button1PressCount%2 == 1){//if state is on
    //check if initialized
    if (!isInitialized) {
      //then initialize
      initialization();
    }

    //button2 initially 0, if not pressed once
    if (button2PressCount%2 == 0){//if state is off
      //normal mode / state 1

      //timer should RESET here:
      unsigned long initialTime = millis();

      float Xcoord = readScreen(PinX);
      float Ycoord = readScreen(PinY);

      // ===== FIXED SMOOTHING (normalized FIRST) =====
      float Xnorm = (Xcoord - 400) / 400.0;
      float Ynorm = (Ycoord - 300) / 300.0;

      float Xsmooth = alpha * Xnorm + (1 - alpha) * Xl;
      float Ysmooth = alpha * Ynorm + (1 - alpha) * Yl;

      float dFromLastPosition = sqrt(pow(Xsmooth - Xl, 2) + pow(Ysmooth - Yl, 2));

      if (millis() - freezeInitialTime < 3000){
        //return to default height;
        matrixOperations(0,0);
      
        if (abs(Theta1 - lastTheta1) > 0.1) {
          writeServo(myservo1, PWMConvert(1, int(Theta1)));
          lastTheta1 = Theta1;
        }

        if (abs(Theta2 - lastTheta2) > 0.1) {
          writeServo(myservo2, PWMConvert(2, int(Theta2)));
          lastTheta2 = Theta2;
        }

        if (abs(Theta3 - lastTheta3) > 0.1) {
          writeServo(myservo3, PWMConvert(3, int(Theta3)));
          lastTheta3 = Theta3;
        }
        //freezeInitialTime = millis(); //may be unecessary
      }

      //if (Xcoord == 0 && Ycoord == 0 && dFromLastPosition>0.1 && dFromLastPosition <0.9){
      //   //freee 3s
      //   freezeInitialTime = millis();

      // }else if(Xcoord == 0 && Ycoord == 0 && dFromLastPosition >= 0.95){
      //   //extreme angle 3s
      //   freezeInitialTime = millis();

      //   PID(Xo, Yo, Xsmooth, Ysmooth, Xnorm, Ynorm);

      //   //pwm convertion and actuator write, using the theta1,2,3 values obtained from previous function
      //   if (abs(Theta1 - lastTheta1) > 0.1) {
      //     writeServo(myservo1, PWMConvert(1, int(Theta1)));
      //     lastTheta1 = Theta1;
      //   }

      //   if (abs(Theta2 - lastTheta2) > 0.1) {
      //     writeServo(myservo2, PWMConvert(2, int(Theta2)));
      //     lastTheta2 = Theta2;
      //   }

      //   if (abs(Theta3 - lastTheta3) > 0.1) {
      //     writeServo(myservo3, PWMConvert(3, int(Theta3)));
      //     lastTheta3 = Theta3;
      //   }
        


      // }

      if (!(Xcoord == 0 && Ycoord == 0)){
        //the PID function updates Theta1,2,3 and others
        PID(Xo, Yo, Xsmooth, Ysmooth, Xnorm, Ynorm);

        //pwm convertion and actuator write, using the theta1,2,3 values obtained from previous function
        if (abs(Theta1 - lastTheta1) > 0.1) {
          writeServo(myservo1, PWMConvert(1, int(Theta1)));
          lastTheta1 = Theta1;
        }

        if (abs(Theta2 - lastTheta2) > 0.1) {
          writeServo(myservo2, PWMConvert(2, int(Theta2)));
          lastTheta2 = Theta2;
        }

        if (abs(Theta3 - lastTheta3) > 0.1) {
          writeServo(myservo3, PWMConvert(3, int(Theta3)));
          lastTheta3 = Theta3;
        }

        freezeInitialTime = millis();
      }
      //wait till one period
      while (millis()-initialTime < period){}
    }

    else if (button2PressCount % 2 == 1){ //if button 2 state is on
      //hence enters the robot into state 2 (repetitive geometrical motions)
      // 0: eliminate this case, because when remainder is 0, the geometric pattern mode is off/ it's operating on normal mode
      if (button2PressCount % 5 == 0){
        ++button2PressCount;
      }

      switch (button2PressCount % 5) {

        // 1 = linear motion (back and forth)
        case 1:

          //timer should RESET here:
          unsigned long initialTime = millis();
          
          //wait till one period
          while (millis()-initialTime < period){}
          break;

        // 2 = triangular motion
        case 2:

          break;

        // 3 = square motion
        case 3:

          break;

        // 4 = circlular motion
        case 4:
        
          break;
        
        default:
          Serial.println("How the Hell did This Happen?");
          break;
      }
    }

  }else{
    //if button 1 is to off status, the robot goes into sleep mode (0 height, motors detached)
    if (!isReset){
      resetAndDeinitialize();
    }

  }


}


