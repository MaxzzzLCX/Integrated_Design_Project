/*
This file "sweep_detection.ino" includes functions that does the sweeping search for block
*/

// sweep until TOF detects a block
void sweep(){

  end = false;
  long int sweep_start = millis();
  long int sweep_end = millis();
  long int time_diff = 0;
  sweep_duration = 0;
  movement = true;

  while (end == false){

    sweep_end = millis();
    time_diff = sweep_end - sweep_start;

    flash(movement);

    LMotor->run(FORWARD);
    LMotor->setSpeed(65);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(65);

    dist_t = ToF_detection();
    Serial.println("HERE: "); Serial.println(dist_t);

    if (dist_t > 20 && dist_t < 350) {
      Serial.println(dist_t);
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      end = true;
      sweep_duration = time_diff;

      // reset the detections readings
      for (int i=0; i<5; i+=1) {
        detections[i] = 1000;
      }
    }
  }
}

// after TOF detects a block, go forward towards the block until the TOF distance is below a certain threshold
void forward_after_sweep(){

  for (int i=0; i<5; i+=1) {
    detections[i] = 1000;
    //ultrasound_detections[i] = 1000;
  }

  end = false;

  while (end == false){
    LMotor->setSpeed(90);
    RMotor->setSpeed(90);
    dist_t = ToF_detection();

    if (dist_t > 20 && dist_t < far_away) {
      end = true;
      Serial.println(dist_t);
      grabbed = grab_block(false);
      if (grabbed == true) {
        return_to_base = true;
        Serial.println("triggered");
      }
    }
  }
}

// reverse the amount of rotation during the sweeping
void reverse_sweep(){
  left_turn(65, sweep_duration);
}
 
// simple function for detecting block
bool detected(int distance) {           

  if (distance < 200) {
    Serial.println("DETECTED");              // detection parameter for block
    return true;
  }

  else {
    return false;
  }
}

 

