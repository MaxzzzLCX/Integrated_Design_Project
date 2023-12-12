/*
This file "delay_functions.ino" includes functions that does certain movement for a controled amount of time
*/

// move forward for a controled time
void forward(int speeds, int duration){

  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;


  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds+7);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds);

    end_time = millis();
    time_diff = end_time - start_time;

    movement = true;
    flash(movement);

    // condition: checking duration of current motion. when duration reached a threshold stop. 
    if (time_diff > duration) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      movement = false;
      end = true;
    }
  }
}

// move backward for a controled time
void backward(int speeds, int duration){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds+7);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);

    end_time = millis();
    time_diff = end_time - start_time;

    movement = true;
    flash(movement);

    // condition: checking duration of current motion. when duration reached a threshold stop.
    if (time_diff > duration) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      movement = false;
      end = true;
    }
  }
}

// turn left for a controled time
void left_turn(int speeds, int duration){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    LMotor->run(BACKWARD);
    LMotor->setSpeed(speeds);
    RMotor->run(FORWARD);
    RMotor->setSpeed(speeds);

    end_time = millis();
    time_diff = end_time - start_time;

    movement = true;
    flash(movement);

    // condition: checking duration of current motion. when duration reached a threshold stop.
    if (time_diff > duration) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      movement = false;
      end = true;
    }
  }
}

// turn right for a controled time
void right_turn(int speeds, int duration){
  end = false;
  long int start_time = millis();
  long int end_time = millis();
  long int time_diff = 0;
  
  while (end == false){

    int FLLineResult = digitalRead(frontLeftLineSensorPIN);
    int FRLineResult = digitalRead(frontRightLineSensorPIN);

    LMotor->run(FORWARD);
    LMotor->setSpeed(speeds);
    RMotor->run(BACKWARD);
    RMotor->setSpeed(speeds);

    end_time = millis();
    time_diff = end_time - start_time;

    movement = true;
    flash(movement);

    // condition: checking duration of current motion. when duration reached a threshold stop.
    if (time_diff > duration) {
      LMotor->run(BACKWARD);
      LMotor->setSpeed(0);
      RMotor->run(BACKWARD);
      RMotor->setSpeed(0);
      movement = false;
      end = true;
    }
  }
}