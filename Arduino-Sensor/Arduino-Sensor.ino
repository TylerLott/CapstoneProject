
#include <SoftwareSerial.h>
float offset = 0;
float sensorReading;


SoftwareSerial sensorSerial(2,3); // RX, TX
// pin d10 used for servo

void setup() {
  // SETUP for the serial over USB connection to Pi
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // SETUP for the software serial connection to sensor
  sensorSerial.begin(9600);
}


void loop() {

  sensorReading = getMeasurment(offset);
//  Serial.println(sensorReading);

  // If the program recieves "get value\n" from the pi it sends the most recent sensor reading
  // If the program recieves "do calibr\n" from the pi it sends the most recent sensor reading
  if (Serial.available() > 0) {
    String data = Serial.readString();
    data.trim();
    if (data.equals("g")){
      Serial.println(sensorReading, 3);
    }
    else if (data == "c") {
      offset = runCalibration();
      Serial.println("d");
    }else{
      Serial.println("r");
      while(Serial.available());
    }
  }

  delay(1);

}


//FUNCTION to get the measurement from the sensor
//  The sensor is accurate to 0.01 only after the RMS of 16 measurements
float getMeasurment(float offset){

  int numMeasurments = 20;
  float sumMeasure = 0;
  int negCount = 0;
  float sign = 1;
  
  for (int i = 0; i < numMeasurments; i++){
    float measured = singleMeasure();
    if (measured != -666){
      sumMeasure += sq(measured);
      if (measured < 0) {negCount++;}
    }else{
      Serial.println("e");
      i--;
    }
  }
  if (negCount > (numMeasurments / 2)) { sign = -1; }
  float avgSq = sumMeasure / numMeasurments;
  return sqrt(avgSq) * sign;
  
}


// FUNCTION takes a single measurement from the sensor
float singleMeasure(){
  byte message[] = {0x77, 0x04, 0x00, 0x01, 0x05};
  sensorSerial.write(message, sizeof(message));
  delay(16);

  byte rec[9];
  int i = 0;
  
  while (sensorSerial.available() > 0) {
      byte b = sensorSerial.read();
      rec[i] = b;
      // Serial.println(b);
      i++;
    }
    
  
  if (isValidReturn(rec)){
    // if valid then bytes 4-7 are the measurment
    float angle;
    float sign;
    
    //find sign and first int
    if (rec[4] >= 16){
      sign = -1;
    }else{
      sign = 1; 
    }

    angle = (rec[4]%16) * 100;
    angle += ((int)rec[5]/16) * 10 +(rec[5]%16);
    angle += ((int)rec[6]/16) * 0.1 + (rec[6]%16) * 0.01;
    angle += ((int)rec[7]/16) * 0.001;
    angle = angle * sign;

    return angle;
  }
  else{ return -666;}
  
}


// FUNCTION to check the message to make sure its been returned correctly
bool isValidReturn(byte message[]){
  
  // TODO actually put the logic to check the result

  // simple check of the identifier bit 
  // return (message[0]==119);
  return true;
}


//FUNCTION to complete calibration cycle
float runCalibration(){
  // TODO put logic to run calibration and return calibration constant
  return 0;
}
