/*
This file "error_correction.ino" includes functions that auto-corrects the robot during line-following AND aligns the robot's orientation with a line
*/

// traveling forward, auto-correct when derailling
void error_correction(int speeds, int FLLineResult, int FRLineResult) {
  // if on line
  if (FLLineResult == 1 && FRLineResult == 1) {
    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds);
  } 

  // if derail to right
  else if (FLLineResult == 1 && FRLineResult == 0)  {
    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds-50);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds+50);
  } 

  // if derail to left
  else if (FLLineResult == 0 && FRLineResult == 1)  {
    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds+50);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds-50);
  } 
}

// traveling backward, auto-correct when derailling
void error_correction_backward(int speeds, int FLLineResult, int FRLineResult){
  // if on line
  if (FLLineResult == 1 && FRLineResult == 1) {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds+4);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);
  } 
  
  // if derail to right
  else if (FLLineResult == 1 && FRLineResult == 0)  {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds-50+4);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);
  } 
  
  // if derail to left
  else if (FLLineResult == 0 && FRLineResult == 1)  {
    LMotor->run(BACKWARD);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds-50);
    LMotor->setSpeed(speeds+4);
  }
}

// align robot's orientation with a line, moving forward
void refinement(int speeds, int leftLineResult, int rightLineResult) {

  // when aligned with line
  if (leftLineResult == 1 && rightLineResult == 1) {
    LMotor->run(FORWARD);
    LMotor->setSpeed(0);
    RMotor->run(FORWARD);
    RMotor->setSpeed(0);
  } 
  
  // if off-line
  else if(leftLineResult == 0 && rightLineResult == 0){
    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds+9);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds);
  }

  // derail to right
  else if (leftLineResult == 1 && rightLineResult == 0)  {
    LMotor->run(FORWARD);
    LMotor->setSpeed(0);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds);
  } 
  
  // derail to left
  else if (leftLineResult == 0 && rightLineResult == 1)  {
    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds);
    RMotor->run(FORWARD);
    RMotor->setSpeed(0);
  }

}

// align robot's orientation with a line, moving backward
void back_refinement(int speeds, int leftLineResult, int rightLineResult) {

  // when aligned with line
  if (leftLineResult == 1 && rightLineResult == 1) {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(0);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(0);
  } 
  
  // if off-line
  else if(leftLineResult == 0 && rightLineResult == 0){
    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds+4);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);
  }

  // derail to right
  else if (leftLineResult == 1 && rightLineResult == 0)  {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(0);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);
  } 
  
  // derail to left
  else if (leftLineResult == 0 && rightLineResult == 1)  {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(0);
  }
}

// after grabbing block, do a little shake to ensure magnetic sensor detected
void shimmy(int speeds, int FLLineResult, int FRLineResult){
  if (FLLineResult == 1 && FRLineResult == 1) {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);
  } 
  else if (FLLineResult == 1 && FRLineResult == 0)  {
    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds-50);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);
  } 
  else if (FLLineResult == 0 && FRLineResult == 1)  {
    LMotor->run(BACKWARD);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds-50);
    LMotor->setSpeed(speeds);
  }
}

