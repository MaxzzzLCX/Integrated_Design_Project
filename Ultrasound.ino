/*
This file "Ultrasound.ino" includes functions for the ultrasound route. 
*/

void ultrasound_route(){

  // initialize
  ultrasound_dist = 1000;
  ultrasound_detected = false;

  // route to serach for central block
  straight_to_cross(0);
  turn_left(turn_delay, 0, 200);
  straight_to_T(0);
  turn_right(turn_delay, 0, 200);
  straight_to_branchRight(0, 1);
  turn_right(turn_delay, 0, 200);

  straight_to_T(2);

  // after ultrasound sensor detects the block
  Serial.print("ultrasound_detected is: ");
  Serial.println(ultrasound_detected);


  // (1) turn
  left_turn(speed,1300);


  // (2) move back and "refine" against the line
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;

  while (end == false){ 

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);
    int leftLineResult = digitalRead(leftLineSensorPIN);
    int rightLineResult = digitalRead(rightLineSensorPIN);

    // error_correction(speed, FRLineResult, FLLineResult);
    back_refinement(speed, leftLineResult, rightLineResult);

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }


  // (3) move forward 
  forward(speed, (dist_t - 10)*25);


  // (4) sweep for block, move forward and grab the block
  sweep();
  delay(500);
  forward_after_sweep();


  // (5) reverse the sweep motion
  reverse_sweep();


  // (6) go backward to the line
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

    if (leftLineResult == 1 && rightLineResult == 1){
      end = true;
      Serial.println("REFINED");
      LMotor->setSpeed(0);
      RMotor->setSpeed(0);
    }
  }


  // (7) returning to cross
  if (branch_counter == 0){
    turn_right(turn_delay, 0, 200);
    straight_to_branchRight(0,2);
    turn_right(turn_delay, 0, 200);
  } else if (branch_counter == 1){
    turn_right(turn_delay, 0, 200);
    turn_right(turn_delay, 0, 200);
    straight_to_branchRight(0,1);
  } else if (branch_counter == 2){
    turn_left(turn_delay, 0, 200);
    straight_to_branchLeft(0,1);
    turn_left(turn_delay, 0, 200);
  } else if (branch_counter == 3){
    turn_left(turn_delay, 0, 200);
    straight_to_branchLeft(0,2);
    turn_left(turn_delay, 0, 200);
  }

  straight_to_cross(0);


  // (8) after reaching cross, drop bloack and return to base
  turn_right_count(turn_delay, 0, 2);
  return_to_base_and_drop_blocks();

  }
}
