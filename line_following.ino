/*
This file "line_following.ino" includes functions for line-following
*/


// setup the route left, for line-following
void line_route() {
  turn_counter = 0;

  if (return_to_base == false) {
          
    if (return_to_base == false){
      straight_to_T_passing_cross(1);
    }

    if (return_to_base == false) {    // stops line route if return_to_base is true
      turn_left(turn_delay, 0, 300);         // parameters are detection modes
      turn_counter += 1;
      Serial.print("L1");
    }    
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      straight_to_T_and_count(1);         // parameters are detection modes
      Serial.print("T2");
    }    
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      turn_left(turn_delay, 0, 300);         // parameters are detection modes
      turn_counter += 1;
      Serial.print("L2");
    }    
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      straight_to_branchLeft(1,1);         // parameters are detection modes
    }
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      turn_left(turn_delay, 0, 300);         // parameters are detection modes
      turn_counter += 1;
      left_route = false;
    }
    
    if (return_to_base == false) {    
      straight_to_cross(1);         
    }

    if (return_to_base == false) {    
      turn_left(0, 0, 50);         
    }


    Serial.println("FINISH***************************");

  }
}

// setup the route right, for line-following
void line_route_right() {
  turn_counter = 0;
  line_blocks = true;

  if (return_to_base == false) {
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      straight_to_T(1);
      Serial.println("START OF RIGHT LOOP");
    }

    if (return_to_base == false) {    // stops line route if return_to_base is true
      turn_right(turn_delay, 0, 300);        
      turn_counter += 1;
      Serial.print("L1");
      Serial.println("TURN RIGHT");
    }    
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      straight_to_T_and_count(1);         // parameters are detection modes
      Serial.print("T2");
      Serial.println("STRAIGHT TO T");
    }    
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      turn_right(turn_delay, 0, 300);         // parameters are detection modes
      turn_counter += 1;
      Serial.print("L2");
    }    
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      straight_to_branchRight(1,1);         // parameters are detection modes
    }
    
    if (return_to_base == false) {    // stops line route if return_to_base is true
      turn_right(turn_delay, 0, 300);         // parameters are detection modes
      turn_counter += 1;
      // left_route = false;
    }
    
    if (return_to_base == false) {    
      straight_to_cross(1);         
    }

    if (return_to_base == false) {    
      turn_right(0, 0, 50);         
    }

    Serial.println("FINISH***************************");
  }
}

// leave home and go on line
void leave_home(){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  grabbed = false;

  LMotor->setSpeed(100);
  RMotor->setSpeed(100);
  LMotor->run(FORWARD);
  RMotor->run(FORWARD);
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    
    flash(movement);

    end_time = millis();
    time_diff = end_time - start_time;
    
    LMotor->setSpeed(100);
    RMotor->setSpeed(100);
    LMotor->run(FORWARD);
    RMotor->run(FORWARD);
    movement = true;
     
    if (FLLineResult == 1 && FRLineResult == 1 && time_diff > 100) { //MAX CHANGE
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      end = true;
    }
  }
}

// line-following, stop at T
void straight_to_T(int detection){

  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  grabbed = false;
  movement = true;

  while (end == false){
    flash(movement);

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);

    if (FLLineResult == 0 && FRLineResult == 0) { //FOR TESTING
      LMotor->setSpeed(speed+7);
      RMotor->setSpeed(speed);
      if (leftLineResult == 1 && rightLineResult == 1 )  { //&& time_diff > 3000
        movement = false;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        digitalWrite(blueLEDPIN, LOW);
        end = true;
      }
    }

    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    if (detection == 2 && time_diff > 1000) {
      dist_t = ultrasound_detection();
      if (dist_t < 95) {
        ultrasound_detected = true;
        ultrasound_dist = dist_t;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        Serial.println("BREAKING HERE");
        break;
      }
    }
  }
}

// line-following, stop at T, but determines whether passed through the cross on its way
void straight_to_T_passing_cross(int detection){

  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  long int branch_start = millis();
  long int branch_end = millis();
  long int branch_diff = 0;
  grabbed = false;
  movement = true;
  cross_passed = false;

  while (end == false){
    flash(movement);

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);

    branch_end = millis();
    branch_diff = branch_end - branch_start;

    
    if (FLLineResult == 0 && FRLineResult == 0) { 
      LMotor->setSpeed(speed+7);
      RMotor->setSpeed(speed);

      // when reached T
      if (FLLineResult == 0 && FRLineResult == 0 && leftLineResult == 1 && rightLineResult == 1 && time_diff > 3000) { 
        movement = false;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        digitalWrite(blueLEDPIN, LOW);
        end = true;
      }
    }
    
    // count cross
    if (FLLineResult == 1 && FRLineResult == 1 && leftLineResult == 1 && rightLineResult == 1 && branch_diff > 1000){
      cross_passed = true;
      Serial.print("CROSSING CROSS"); Serial.println(cross_passed);
      branch_start = millis();
      branch_end = millis();
      branch_diff = 0;
    }

    // TOF detections 
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE / ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }
  }
}

// line-following, stop at T, but counts the number of "branches" it passes through on its way
void straight_to_T_and_count(int detection){

  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  long int branch_start = millis();
  long int branch_end = millis();
  long int branch_diff = 0;
  grabbed = false;
  movement = true;
  branch_counter = 0;

  while (end == false){
    flash(movement);

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);

    branch_end = millis();
    branch_diff = branch_end - branch_start;
    
    // when reached T
    if (FLLineResult == 0 && FRLineResult == 0 && leftLineResult == 1 && rightLineResult == 1 && time_diff > 3000) {
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      digitalWrite(blueLEDPIN, LOW);
      end = true;
    }
    
    // count branches
    if (FLLineResult == 1 && FRLineResult == 1 && (leftLineResult == 1 || rightLineResult == 1) && branch_diff > 1000){
      branch_counter += 1;
      Serial.print("New Branch. Current Branch Count: "); Serial.println(branch_counter);
      branch_start = millis();
      branch_end = millis();
      branch_diff = 0;
    }

    // TOF detection
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      dist_t = ultrasound_detection();
      if (dist_t < 95) {
        ultrasound_detected = true;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        Serial.println("BREAKING HERE");
        break;
        // return ("DETECTED");
      }
    }
  }
}

// line-following, stop at corss
void straight_to_cross(int detection){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  grabbed = false;

  LMotor->setSpeed(100);
  RMotor->setSpeed(100);
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    
    flash(movement);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);
    
    // when reached cross
    if (FLLineResult == 1 && FRLineResult == 1 && leftLineResult == 1 && rightLineResult == 1 && time_diff > 500) { 
      delay(200);
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      digitalWrite(blueLEDPIN, LOW);
      end = true;
    }

    // TOF detection
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }
  }
}

// line-following, stop at corner_right
void straight_to_cornerRight(int detection){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  grabbed = false;

  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    
    flash(movement);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);


    // when reach corner_right
    if (FLLineResult == 0 && FRLineResult == 0 && rightLineResult == 1 && time_diff > 3000) { 
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      digitalWrite(blueLEDPIN, LOW);
      end = true;
    }

    // TOF detection
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound dtection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    } 
  }
}

// line-following, stop at corner_left
void straight_to_cornerLeft(int detection){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  grabbed = false;
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);
    flash(movement);

    // when reached corner_left
    if (FLLineResult == 0 && FRLineResult == 0 && leftLineResult == 1 && rightLineResult == 0 && time_diff > 3000) {
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      digitalWrite(blueLEDPIN, LOW);
      end = true;
    }

    // TOF detection
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }
  }
}

// line-following, stop at the nth branch_left. the parameter number is for the number n (which branch do you stop at)
void straight_to_branchLeft(int detection, int number){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  int current_number = 0;
  grabbed = false;
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);
    flash(movement);

    // when reached branch_left
    if (FLLineResult == 1 && FRLineResult == 1 && leftLineResult == 1 && rightLineResult == 0 && time_diff > 500) {
      current_number += 1;

      // if this is the nth branch now
      if (current_number == number){
        movement = false;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        digitalWrite(blueLEDPIN, LOW);
        end = true;
      } 
      // if not the nth branch
      else {
        Serial.println("DETECTED BRANCH LEFT");
        start_time = millis();
        end_time = millis();
        time_diff = end_time - start_time;
      }
    }

    // TOF dections
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }

  }
}

// line-following, stop at the nth branch_right. the parameter number is for the number n (which branch do you stop at)
void straight_to_branchRight(int detection, int number){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  int current_number = 0;
  grabbed = false;
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction(speed, FRLineResult, FLLineResult);
    flash(movement);

    // when reach branch right
    if (FLLineResult == 1 && FRLineResult == 1 && leftLineResult == 0 && rightLineResult == 1 && time_diff > 500) {
      current_number += 1;

      // if this is the nth branch
      if (current_number == number){
        movement = false;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        end = true;
      }
      // if not the nth branch
      else{
        Serial.println("DETECTED BRANCH RIGHT");
        start_time = millis();
        end_time = millis();
        time_diff = end_time - start_time;
      }
    }

    // TOF dectecion
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t > 20 && dist_t < far_away) {
        Serial.println(dist_t);
        grabbed = grab_block(false);
        if (grabbed == true) {
          return_to_base = true;
          Serial.println("triggered");
          break;
        }
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }
  }
}

// turn_right until front-line sensors touch the line
void turn_right(int delay_time, int detection, int threshold){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    flash(movement);

    LMotor->run(FORWARD);
    LMotor->setSpeed(speed);
    delay(10);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed);
    delay(10);
     
    // when front line sensors touch the line again, we think the turning is done
    if (FLLineResult == 1 && FRLineResult == 1 && time_diff > threshold) { 
      delay(delay_time);
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      digitalWrite(blueLEDPIN, LOW);
      end = true;
    }

    // TOF detection
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }
  }
}

// turn_left until front-line sensors touch the line
void turn_left(int delay_time, int detection, int threshold){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  movement = true;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    flash(movement);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed);
    delay(10);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speed);
    delay(10);

  
    // when front line sensors touch the line again, we think the turning is done
    if (FLLineResult == 1 && FRLineResult == 1 && time_diff > threshold) { //FOR TEST
      delay(delay_time);
      movement = false;
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      digitalWrite(blueLEDPIN, LOW);
      end = true;
    }

    // TOF detection
    if (detection == 1) {
      dist_t = ToF_detection();
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }

    // ultrasound detection
    if (detection == 2) {
      sensity_t = analogRead(sensityPin);
      dist_t = sensity_t * MAX_RANGE /ADC_SOLUTION;
      if (dist_t < 50) {
        return ("DETECTED");
      }
    }
  }
}

// turn_right, but counting the number of lines traversed. this enables us to turn over multiple lines
void turn_right_count(int delay_time, int detection, int number){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  movement = true;

  int current_number = 0;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    end_time = millis();
    time_diff = end_time - start_time;

    flash(movement);

    LMotor->run(FORWARD);
    LMotor->setSpeed(speed);
    delay(10);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed);
    delay(10);

    // when front line sensors touch the line
    if (FLLineResult == 1 && FRLineResult == 1 && time_diff > 1000) {

      current_number += 1;

      // if is the number of turns wanted
      if (current_number == number){
        Serial.println("FINISH TURNING");
        movement = false;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        delay(delay_time);
        end = true;
      }

      // if no the number of turns wanted
      else{
        Serial.println("DETECTED A LINE");
        start_time = millis();
        end_time = millis();
        time_diff = end_time - start_time;
      }
    }
  }
}

// turn_left, but counting the number of lines traversed. this enables us to turn over multiple lines
void turn_left_count(int delay_time, int detection, int number){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  movement = true;

  int current_number = 0;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    
    //testing_line_sensors(FLLineResult, FRLineResult, leftLineResult, rightLineResult);

    end_time = millis();
    time_diff = end_time - start_time;

    flash(movement);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed);
    delay(10);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speed);
    delay(10);
     
    // when front line sensors touch the line
    if (FLLineResult == 1 && FRLineResult == 1 && time_diff > 1000) { 
      current_number += 1;

      // if is the number of turns wanted
      if (current_number == number){
        Serial.println("FINISH TURNING");
        movement = false;
        LMotor->setSpeed(0);
        RMotor->setSpeed(0);
        delay(delay_time);
        end = true;
      }
      // if no the number of turns wanted
      else{
        Serial.println("DETECTED A LINE");
        start_time = millis();
        end_time = millis();
        time_diff = end_time - start_time;
      }
    }
  }
}

