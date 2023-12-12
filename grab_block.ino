/*
This file "grab_block.ino" includes functions that grab block after detection
*/

// move forward and grab block
bool grab_block(bool sweep) { 
  int distance = 0;   
  distance = sensor.getDistance();
  LMotor->run(FORWARD);
  RMotor->run(FORWARD);

  // grabbing block not during line-following, in central block cases
  if (sweep == true) {
    while (distance > 100) {    
      LMotor->setSpeed(50); 
      RMotor->setSpeed(50);
    }
    LMotor->setSpeed(0); 
    RMotor->setSpeed(0);

    claw();
    return true;
  }

  // grabbing block during line-following 
  if (distance < 100 && sweep == false){  
    LMotor->setSpeed(0); 
    RMotor->setSpeed(0);
    claw();
    return true;
  }

  else {
    return false;
  }


}

// dropping claw servo
void claw() {
  int mag = 0;

  servoPos = 0;    
  int lowerPos = 25; // pos when dropped 
  int upperPos = 90; // pos when raised
  int long start_time;
  int long end_time;
  int long time_diff;
  int long shimmy_end;
  int long shimmy_duration;

  block_counter += 1;

  //lowers claw in steps of 1 degree
  for (servoPos = upperPos; servoPos >= lowerPos; servoPos -= 1) { 
    clawServo.write(servoPos); // tell servo to go to position in variable 'pos'
    delay(15); // waits 15 ms for the servo to reach the position
  }
  delay(100);
  Serial.println("run");

  start_time = millis();
  time_diff = 0;


  while (time_diff < 2600) {
    mag = digitalRead(magnetPIN);
    end_time = millis();
    time_diff = end_time - start_time;

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    float gradual_decrease_speed;
    movement = false;
    flash(movement);
    
    // after dropping claw, do a little shake
    if (time_diff<1500){
      shimmy(speed, FLLineResult, FRLineResult);
    }

    // after shake, travel forward a little
    if (time_diff > 1500 && time_diff < 2500 && speed > 0) { //MAX: Changed from 1000 to 500
      gradual_decrease_speed = (3500.0 - time_diff) / 1000 * speed;
      error_correction(speed, FLLineResult, FRLineResult);
    }

    // if detected magnetic block
    if (mag == HIGH) { 
      magnetic = true;
    }

    // if time's up
    if (time_diff > 2500) {
      RMotor->setSpeed(0);
      LMotor->setSpeed(0);
      movement = false;
    }
  }


  // if block magnetic
  if (magnetic == true){
    digitalWrite(greenLEDPIN, LOW);
    digitalWrite(redLEDPIN, HIGH);
    digitalWrite(blueLEDPIN, LOW);
    delay(5000);
    digitalWrite(redLEDPIN, LOW);
  }

  // if block non-magnetic
  else {
    digitalWrite(redLEDPIN, LOW); 
    digitalWrite(greenLEDPIN, HIGH);
    digitalWrite(blueLEDPIN, LOW);
    delay(5000);
    digitalWrite(greenLEDPIN, LOW);
  }

  LMotor->run(FORWARD);
  RMotor->run(FORWARD);    
  return_to_base = true;

}



