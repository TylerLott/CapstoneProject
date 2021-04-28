#include <VarSpeedServo.h> 
 
VarSpeedServo myservo;    // create servo object to control a servo 
 
void setup() {
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object 
  myservo.write(0, 20, true);
} 
 
void loop() {
  myservo.write(0, 20, true);
  delay(5000);
  myservo.write(135, 20, true);
  delay(5000);
}
