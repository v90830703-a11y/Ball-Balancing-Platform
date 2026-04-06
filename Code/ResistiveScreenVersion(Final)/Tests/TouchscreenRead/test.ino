#define XP 7   // Green
#define XM A8  // Blue
#define YP A9  // Yellow
#define YM 8   // Orange


const float Xmax = 950;
const float Xmin = 200;
const float Ymax = 900;
const float Ymin = 200;



void setup() {
  Serial.begin(115200);
}

void loop() {
  pinMode(XP, OUTPUT); digitalWrite(XP, HIGH);
  pinMode(XM, OUTPUT); digitalWrite(XM, LOW);

  pinMode(YP, INPUT);
  pinMode(YM, INPUT);

  float yValue = analogRead(YP);  // read along X layer
  Serial.println(yValue);
  yValue = constrain(yValue, Ymin, Ymax);
  float Ycoord = (yValue-Ymin)/(Ymax-Ymin)*600.00;
  //constrain(Ycoord,0,600);
  Serial.println(Ycoord);
  
  
  
  
  pinMode(YP, OUTPUT); digitalWrite(YP, LOW);
  pinMode(YM, OUTPUT); digitalWrite(YM, HIGH);

  pinMode(XM, INPUT);
  pinMode(XP, INPUT);

  float xValue = analogRead(XM);  // read along X layer
  Serial.println(xValue);
  xValue = constrain(xValue, Xmin, Xmax);
  float Xcoord = (xValue-Xmin)/(Xmax-Xmin)*800.00;
  //constrain(Xcoord,0,800);
  Serial.println(Xcoord);

  delay(300);


}