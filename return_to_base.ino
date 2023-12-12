/*
The file "return_to_base.ino" include functions that return the robot to base after grabbing the block
*/

void U_turn(){
  turn_right(turn_delay,0, 300);
}

// returning to cross from right-hand side of the court
void return_cross_right(){

  Serial.println("Turn Counter: ");
  Serial.println(turn_counter);
  Serial.println("Branch Counter: ");
  Serial.println(branch_counter);
  
  // depending the number of turns and number of branches the robot goes over during the serach phase, it will have a different return route. 
  if (turn_counter == 0){
    U_turn();
    straight_to_cross(0);
    turn_right_count(50, 0, 2);
  } 
  else if (turn_counter == 1){
    U_turn();
    straight_to_branchLeft(0, branch_counter+1);
    turn_left(turn_delay, 0, 300);
    straight_to_cross(0);
    turn_right_count(50, 0, 2);
  }
  else if (turn_counter == 2){
    U_turn();
    straight_to_branchLeft(0, 1);
    turn_left(turn_delay, 0, 300);
    straight_to_branchLeft(0, 2);
    turn_left(turn_delay, 0, 300);
    straight_to_cross(0);
    turn_right_count(50, 0, 2);
  }
  else if (turn_counter == 3){
    straight_to_cross(0);
    turn_right(turn_delay,0, 300);
  }
}

// returning to cross from left-hand side of the court
void return_cross_left(){

  Serial.println("Turn Counter: ");
  Serial.println(turn_counter);
  Serial.println("Branch Counter: ");
  Serial.println(branch_counter);

  // depending the number of turns and number of branches the robot goes over during the serach phase, it will have a different return route.
  if (turn_counter == 0){
    if (cross_passed == false){
      straight_to_cross(0);
    } else{
      U_turn();
      straight_to_cross(0);
      turn_right_count(50, 0, 2);
    }
  } 
  else if (turn_counter == 1){
    U_turn();
    straight_to_branchRight(0, branch_counter+1);
    turn_right(turn_delay, 0, 300);
    straight_to_cross(0);
    turn_right_count(50, 0, 2);
  }
  else if (turn_counter == 2){
    //U_turn();
    turn_left(turn_delay,0, 300); // THIS IS THE U-TURN
    straight_to_branchRight(0, 1);
    turn_right(turn_delay, 0, 300);
    straight_to_branchRight(0, 2);
    turn_right(turn_delay, 0, 300);
    straight_to_cross(0);
    turn_right_count(50, 0, 2);
  }
  else if (turn_counter == 3){
    straight_to_cross(0);
    turn_left(turn_delay,0, 300);

  }
}

// lifting the claw
void lift_claw(){
  int lowerPos = 15;
  int upperPos = 90;

  movement = false;

  for (servoPos = lowerPos; servoPos <= upperPos; servoPos += 1) { //lowers claw in steps of 1 degree
    clawServo.write(servoPos); // tell servo to go to position in variable 'pos'
    delay(15); // waits 15 ms for the servo to reach the position
  }
}

// after dropping a block and returning to home, stay for 5 seconds to gain the point
void stay_at_home(){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    movement = false;
    digitalWrite(blueLEDPIN, HIGH);

    LMotor->run(FORWARD);
    LMotor->setSpeed(0);
    RMotor->run(FORWARD);
    RMotor->setSpeed(0);

    end_time = millis();
    time_diff = end_time - start_time;

    if (time_diff > 5000) {
      end = true;
      digitalWrite(blueLEDPIN, LOW);
    }
  }
}

// move back and stop at cross
void back_to_cross(){

  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  bool grabbed = false;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    movement = true;
    flash(movement);
    

    end_time = millis();
    time_diff = end_time - start_time;

    error_correction_backward(speed, FLLineResult, FRLineResult);
    
    // when reaching cross
    if (FLLineResult == 1 && FRLineResult == 1 && leftLineResult == 1 && rightLineResult == 1 && time_diff > 2000) { 
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
      movement = false;
      end = true;
    }
  }
}

// (1) return to base (2) go to the corners to drop blocks
void return_to_base_and_drop_blocks() {

  // reversing back to home
  reverse_to_square();

  // depending on magnetic or not, go to one of the sides and drop block
  if (magnetic == true) {
    drop_magnetic_cube();
  } else {
    drop_non_magnetic_cube();
  }
}

// move backward to the home base
void reverse_to_square() {


  // (1) refine against the cross, move backward
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    back_refinement(speed, leftLineResult, rightLineResult);

    mag = digitalRead(magnetPIN);    

    if (mag == HIGH) { // check if the input is HIGH
      magnetic = true;
    }

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  // (2) BACK A BIT MORE after refined against cross
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;

  while (end == false){

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed);
    delay(10);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed);
    delay(10);
    flash(movement);

    end_time = millis();
    time_diff = end_time - start_time;


    mag = digitalRead(magnetPIN);    

    if (mag == HIGH) { // check if the input is HIGH
      magnetic = true;
    }

    if (time_diff > 700) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
      movement = false;
    }
  }


  // (3) refine against home
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    back_refinement(speed, leftLineResult, rightLineResult);

    mag = digitalRead(magnetPIN);    

    if (mag == HIGH) {
      magnetic = true;
    }

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }


  // (4) BACK A BIT MORE after refined against home
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;

  while (end == false){

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed);
    delay(10);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed);
    delay(10);
    flash(movement);

    end_time = millis();
    time_diff = end_time - start_time;

    mag = digitalRead(magnetPIN);    

    if (mag == HIGH) { // check if the input is HIGH
      magnetic = true;
    }

    if (time_diff > 1000) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
      movement = false;
    }
  }
}

// droping magnetic cube
void drop_magnetic_cube() {
  Serial.println(magnetic);

  // (1) turn to face the drop-off corer
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);


    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speed);

    end_time = millis();
    time_diff = end_time - start_time;


    if (time_diff > 1300) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }
  }

  // (2) move forward and refine orientation with the home base lines
  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }


  // (3) move forward a bit more
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);

    LMotor->run(FORWARD);
    LMotor->setSpeed(speed_low+7);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;

    if (time_diff > 1000) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }



  // (4) move forward until reaching the drop-off corner
  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  // (5) lift claw to drop
  lift_claw();

  // (6) BACK a bit from drop off area
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed_low+7);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;


    if (time_diff > 1000) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }

  // (7) back until touches the home base line again
  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    back_refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  // (8) back a bit more to fully enter the home square
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed_low+7);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;

    if (time_diff > 1300) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }
  }

  // (9) stay at home
  stay_at_home();



  // (10) turn around to face the cross, prepare for next go
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);


    LMotor->run(FORWARD);
    LMotor->setSpeed(speed_low);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;

    if (time_diff > 1300) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }

  // (11) move forward and refine against the home-base lines

  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  int FLLineResult = digitalRead(frontLeftLineSensorPIN);
  int FRLineResult = digitalRead(frontRightLineSensorPIN);
  if (FLLineResult == 0 && FRLineResult == 0){
    turn_left(0,0,0);
  }

}


void drop_non_magnetic_cube() { 
  Serial.println(magnetic);

  // (1) turn to face the drop-off corer
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);


    LMotor->run(FORWARD);
    LMotor->setSpeed(speed_low);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;


    if (time_diff > 1300) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }
  }


  // (2) move forward and refine orientation with the home base lines
  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  // (3) move forward a bit more

  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);


    LMotor->run(FORWARD);
    LMotor->setSpeed(speed_low+7);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;


    if (time_diff > 1000) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }


  // (4) move forward until reaching the drop-off corner
  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  // (5) lift claw
  lift_claw();

  // (6) BACK a bit from drop off area
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);


    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed_low+7);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;


    if (time_diff > 1000) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }

  // (7) back until touches the home base line again

  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    back_refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }



  // (8) back a bit more to fully enter the home square
  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;

  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    movement = true;
    flash(movement);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed_low+7);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;

    if (time_diff > 1500) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }

  // (9) stay at home

  stay_at_home();

  // (10) turn around to face the cross, prepare for next go 

  end = false;
  start_time = millis();
  end_time = millis();
  time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    movement = true;
    flash(movement);


    LMotor->run(BACKWARD);
    LMotor->setSpeed(speed_low);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speed_low);

    end_time = millis();
    time_diff = end_time - start_time;


    if (time_diff > 1100) {
      
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      end = true;
    }

  }

  // (11) move forward and refine against the home-base lines

  end = false;

  while (end == false){
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);
    refinement(speed, leftLineResult, rightLineResult);

    movement = true;
    flash(movement);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }

  int FLLineResult = digitalRead(frontLeftLineSensorPIN);
  int FRLineResult = digitalRead(frontRightLineSensorPIN);
  if (FLLineResult == 0 && FRLineResult == 0){
    turn_right(0,0,50);
  }

}



