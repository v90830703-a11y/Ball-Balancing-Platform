// #include <Servo.h>

// Servo myservo1;

// const int servoPin1 = 2;
// const int servoMinPWM = 500;
// const int servoMaxPWM = 2500;

// void setup() {
//   Serial.begin(115200);
//   myservo1.attach(servoPin1, servoMinPWM, servoMaxPWM);

//   Serial.println("Servo test start");
// }

// void loop() {
//   // Sweep from 0° to 90°
//   for (int angle = 0; angle <= 90; angle += 5) {
//     int pwm = map(angle, 0, 270, servoMinPWM, servoMaxPWM); // full servo range
//     myservo1.writeMicroseconds(pwm);
//     Serial.print("Angle: "); Serial.print(angle);
//     Serial.print(" PWM: "); Serial.println(pwm);
//     delay(500);
//   }

//   // Sweep back 90° to 0°
//   for (int angle = 90; angle >= 0; angle -= 5) {
//     int pwm = map(angle, 0, 270, servoMinPWM, servoMaxPWM);
//     myservo1.writeMicroseconds(pwm);
//     Serial.print("Angle: "); Serial.print(angle);
//     Serial.print(" PWM: "); Serial.println(pwm);
//     delay(500);
//   }
// }