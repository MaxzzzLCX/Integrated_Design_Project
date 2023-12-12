/*
This file "flashing_blue_detections.ino" includes functions to control blue LED flashing, TOF, and ultrasound
*/

// function that does the timing to turn on/off
void flash(bool movement){
  blue_current = millis(); 
  if (((blue_start - blue_current) / 250) % 2 == 0 && movement == true) {
    digitalWrite(blueLEDPIN, HIGH);
  }
  else {
    digitalWrite(blueLEDPIN, LOW);
  }
}

// store five recordings of TOF to prevent accidental triggering
int ToF_detection(){
  ToFCounter += 1;
  if ((ToFCounter) % 10 == 0) {
    detections[0] = sensor.getDistance();
  }
  if ((ToFCounter) % 10 == 2) {
    detections[1] = sensor.getDistance();
  }
  if ((ToFCounter) % 10 == 4) {
    detections[2] = sensor.getDistance();
  }
    if ((ToFCounter) % 10 == 6) {
    detections[3] = sensor.getDistance();
  }
    if ((ToFCounter) % 10 == 8) {
    detections[4] = sensor.getDistance();
  }

  Serial.println("--------TOF----------");
  Serial.println(detections[0]);
  Serial.println(detections[1]);
  Serial.println(detections[2]);
  Serial.println(detections[3]);
  Serial.println(detections[4]);

  return (detections[0] + detections[1] + detections[2])/3;
}

// store five recordings of ultrasound to prevent accidental triggering
int ultrasound_detection(){
  ultrasound_counter += 1;
  sensity_t = analogRead(sensityPin);
  dist_ultrasound = sensity_t * MAX_RANGE / ADC_SOLUTION;

  if ((ultrasound_counter) % 10 == 0) {
    ultrasound_detections[0] = dist_ultrasound;
  }
  if ((ultrasound_counter) % 10 == 2) {
    ultrasound_detections[1] = dist_ultrasound;
  }
  if ((ultrasound_counter) % 10 == 4) {
    ultrasound_detections[2] = dist_ultrasound;
  }
    if ((ultrasound_counter) % 10 == 6) {
    ultrasound_detections[3] = dist_ultrasound;
  }
    if ((ultrasound_counter) % 10 == 8) {
    ultrasound_detections[4] = dist_ultrasound;
  }

  Serial.println("--------ULTRA----------");
  Serial.println(ultrasound_detections[0]);
  Serial.println(ultrasound_detections[1]);
  Serial.println(ultrasound_detections[2]);
  Serial.println(ultrasound_detections[3]);
  Serial.println(ultrasound_detections[4]);

  return (ultrasound_detections[0] + ultrasound_detections[1] + ultrasound_detections[2])/3;
}

