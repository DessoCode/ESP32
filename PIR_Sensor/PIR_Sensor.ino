#define GROVE_mPIR 14 //REL pin to GPIO 14

void setup() 
{
  // put your setup code here, to run once:
  pinMode(GROVE_mPIR, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(isMotionDetected())
  {
    //Note that this will stay on for 5 seconds thanks to the Grove onboard stuff
    Serial.println("Boop boop motion detected!");
      
  }
  else
    Serial.println("No motion.");
}

boolean isMotionDetected()
{
  int sensorValue = digitalRead(GROVE_mPIR);
  if(sensorValue == HIGH)
    return true;
  else
    return false;  
  
}
