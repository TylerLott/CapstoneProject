// Actuators have  around 94340 steps in their 8 inch movement, must be moving at 255!!
//     if actuator speed changes so do the steps in movement
//     11792 steps per inch

long pos = 0;                   // Actuator Position in Pulses
volatile long steps = 0;                 // Pulses from  Hall Effect sensors
float conNum = 0.000285;        // Convert to Inches
bool dir = 0;                   // Direction of Actuator (0=Retract, 1=Extend)
int Speed = 255;
long prevTimer;
long prevPos = 0;
long prevSteps = 0;
bool homeFlag = 0;              // Flag use to know if the Actuator is home
int piVal = 0;
char inputBuffer[16];

void setup() {
  // SETUP for hall sensor
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), countSteps, RISING);

  //SETUP for motor
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  //SETUP for serial
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

}

void loop () {

  piVal = 0;
  
  // Recieves an int from the pi telling it how many steps to move
  if (Serial.available() > 0){
    Serial.println(1);
    
    byte val_1 = Serial.read();
    if (val_1 == 'n'){
      piVal = Serial.parseInt();              // MAX steps is 32000 at a time
    }
    if (val_1 == 'b') {                         // bh is recieved if home requested from pi
      String val = Serial.readString();
      if (val == "h"){
        homeActuator();
        homeFlag = 0;
        Serial.println("done");
      } else if (val == "c"){               // cc is recieved if calibrate requested from pi
        moveActuator(11792);
        Serial.println("done");
      }
    }
    while(Serial.available());
  }  

  if (piVal < 0) { dir = 0; } else { dir = 1; }

  piVal = abs(piVal);

  if (piVal != 0){
    moveActuator(piVal);
    Serial.println("done");
  }

}




bool posFlag = 0;

// move the actuator a distance in a direction
void moveActuator(long stepsToMove) {
  steps = 0;
  prevTimer = millis();
  while (steps < stepsToMove) {
    if (dir == 0) {
      analogWrite(10, 0);
      analogWrite(11, Speed);
    } else if (dir == 1) {
      analogWrite(10, Speed);
      analogWrite(11, 0);
    }
    if (prevSteps != steps) {
      prevSteps = steps;
      prevTimer = millis();
    }
    if (millis() - prevTimer > 10){
      break;
    }
  }
  analogWrite(10, 0);
  analogWrite(11, 0);
  
}


long updateSteps(long steps) {
  return steps;
}


unsigned long lastStepTime = 0; // Time stamp of last pulse
int trigDelay = 500;            // Delay bewteen pulse in microseconds

void countSteps(void) {
  if (micros() - lastStepTime > trigDelay) {
    steps++;
    lastStepTime = micros();
  }
}

void homeActuator(void) {
  prevTimer = millis();
  while (homeFlag == 0) {
    Speed = 255;
    analogWrite(10, 0);
    analogWrite(11, Speed);
    delay(100);
    if (prevSteps == steps) {
      if (millis() - prevTimer > 10) {
        analogWrite(10, 0);
        analogWrite(11, 0);
        steps = 0;
        Speed = 0;
        homeFlag = 1;
      }
    } else {
      prevSteps = steps;
      prevTimer = millis();
    }
  }
}
