
// Including Libraries used
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "DFRobot_VL53L0X.h"


// Define global variables
#define MAX_RANGE (520) //the max measurement value of the module is 520cm (a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0) 


// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RMotor = AFMS.getMotor(1);
Adafruit_DCMotor *LMotor = AFMS.getMotor(2);


// creating sensor and servo object
DFRobot_VL53L0X sensor;
Servo clawServo;


// declaring pins
const int blueLEDPIN = 2; 
const int buttonPIN = 3; 
const int greenLEDPIN = 4; 
const int redLEDPIN = 5; 
const int frontLeftLineSensorPIN = 6; 
const int frontRightLineSensorPIN = 7; 
const int leftLineSensorPIN = 8; 
const int rightLineSensorPIN = 9; 
// servo attached to pin 10, defined later
const int magnetPIN = 12;
const int sensityPin = A0;


// declaring "status" variables
bool end = false; // used for various while-loops
bool first_time = true; // is it the first time robot leaves home
bool button_pressed = false; // button for starting to move
int buttonstate = 0;
int button_sweep_state = 0;


// declaring "measurement" varibles - record measurements
int dist_t = 0;     // sensor distance sensitivity
float sensity_t = 0;
float dist_ultrasound = 0.0;
int detections[5] = {1000, 1000, 1000, 1000, 1000}; // array that records the 5-most-recent TOF readings
float ultrasound_detections[5] = {1000, 1000, 1000, 1000, 1000}; // array that records the 5-most-recent ultrasound readings
int speed = 160; 
int speed_low = 160; 
int ToFCounter = 0;
int ultrasound_counter = 0;
int sweep_dist = 20;
int ultrasound_dist = 1000;
int far_away = 120;
int turn_delay = 20;
long int sweep_duration = 0;
int servoPos = 0;


// declaring "counter" variables - record counters
int turn_counter = 0;
int branch_counter = 0; 
int block_counter = 0;


// declaring "route" variables - variables that determine how the route goes
bool left_route = true;
bool line_blocks = true;
bool return_to_base = false;
bool magnetic = false;
int mag = 0;
bool grabbed = false;
bool cross_passed = false;
bool ultrasound_detected = false;


// variables used for controlling blue LED flashing
int blue_start = 0;
int blue_current = 0;
bool movement = false;






// defined functions

bool grab_block();
void sweep();
bool detected();
void error_correction();
void error_correction_backward();
void shimmy();
void claw();
void flash();
void leave_block();

void line_route();
void straight_to_T();
void straight_to_T_count();
void straight_to_T_passing_cross();
void straight_to_T_count_right();
void straight_to_cross();
void straight_to_cornerRight();
void straight_to_cornerLeft();
void straight_to_branchLeft();
void straight_to_branchRight();
void turn_right(int delay_time);
void turn_left(int delay_time);
void leave_home(); //MAX
void return_to_base_and_drop_blocks();
void reverse_to_square();
void drop_magnetic_cube();
void drop_non_magnetic_cube();

void return_cross();
bool back_to_node();
bool forward_to_node();
void return_cross_left(); 

void home_to_base();
void lift_claw();

void refinement();
void back_refinement();


// setup 
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Wire.begin();
  sensor.begin(0x50);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  LMotor->setSpeed(50); 
  RMotor->setSpeed(50);
  LMotor->run(BACKWARD);
  RMotor->run(BACKWARD);
  LMotor->run(RELEASE);
  RMotor->run(RELEASE);

  clawServo.attach(10); // servo PIN: 10


  // setting pinmodes for all sensors and LEDs
  pinMode(frontLeftLineSensorPIN, INPUT);
  pinMode(frontRightLineSensorPIN, INPUT);
  pinMode(leftLineSensorPIN, INPUT);
  pinMode(rightLineSensorPIN, INPUT);

  pinMode(blueLEDPIN, OUTPUT);
  pinMode(greenLEDPIN, OUTPUT);
  pinMode(redLEDPIN, OUTPUT);

  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  sensor.start();

  // initialize variables 
  int distance = 0;
  bool line_blocks = true; // variable that determines either corner blocks or central blocks
  blue_start = millis(); // timer used for blue LED flashing control
  clawServo.write(115); // initialize servo position

}


void loop() {

  // after 2 corner blocks are grabbed
  if (block_counter == 2) {
    line_blocks = false;
  }
  // after all 4 blocks are grabbed, reset to corner blocks
  else if (block_counter == 4) {
    line_blocks = true;
    block_counter = 0;
  } 

  // wait until the start button is pressed
  while (button_pressed == false){
    delay(100);
    buttonstate = digitalRead(buttonPIN);

    if (buttonstate == HIGH){
      button_pressed = true;
    }
  }


  Serial.print("START");

  return_to_base = false; // when true, means block is already grabbed and robot is on return

  // if this is the first time leaving home
  if (first_time == true) {
      leave_home();
  }

  // if targeting line-blocks
  if (line_blocks == true) { 
    // searching left route first
    if (left_route == true) {
      Serial.println("LEFT ROUTE");

      // move and search
      line_route();
      Serial.println(left_route);

      // when block is found and grabbed
      if (grabbed == true)  {
        Serial.println("*RETURN STARTS*");

        // from where block is grabbed, return to cross
        return_cross_left();

        // from cross, drop block, return base
        return_to_base_and_drop_blocks();
        
        // reset status
        magnetic = false;
        grabbed = false;
      }

    }

    // seraching for right route
    else {
      Serial.println("RUN RIGHT");

      // seraching right route 
      line_route_right();

      // when block is found and grabbed
      if (grabbed == true)  {
        Serial.println("*RETURN STARTS*");

        // from where block is grabbed, return to cross
        return_cross_right();

        // from cross, drop block, return base
        return_to_base_and_drop_blocks();
        
        // reset status
        magnetic = false;
        grabbed = false;
      }
    }

  }

  
  // targeting the central blocks
  else if (line_blocks == false){
    ultrasound_route();
  }

  // reseting sensor readings
  for (int i=0; i<5; i+=1) {
    detections[i] = 1000;
    ultrasound_detections[i] = 1000;
  }

}

// used for printing line sensor results. used when debugging
void testing_line_sensors (int FLLineResult, int FRLineResult, int leftLineResult, int rightLineResult){
  Serial.print("Front Left: "); Serial.println(FLLineResult);
  Serial.print("Front Right: "); Serial.println(FRLineResult);
  Serial.print("Left: "); Serial.println(leftLineResult);
  Serial.print("Right: "); Serial.println(rightLineResult);
}


// void move_to_block(){
//   long int start_time = millis();
//   long int end_time = millis();
//   long int time_diff = end_time - start_time;

//   Serial.println("MOVE TO BLOCK");

//   while (time_diff < 3000){
//     end_time = millis();
//     time_diff = end_time - start_time;

//     LMotor->run(FORWARD);
//     LMotor->setSpeed(120);
//     delay(10);
//     RMotor->run(FORWARD);
//     RMotor->setSpeed(120);
//     delay(10);
//   }
//   LMotor->setSpeed(0);
//   RMotor->setSpeed(0);
//   delay(3000);

//   move_back(time_diff);

// }

// void move_back(int duration){         // 
//   long int start_time = millis();
//   long int end_time = millis();
//   long int time_diff = end_time - start_time;

//   Serial.println("MOVE TO BLOCK");

//   while (time_diff < duration){
//     end_time = millis();
//     time_diff = end_time - start_time;

//     LMotor->run(BACKWARD);
//     LMotor->setSpeed(120);
//     delay(10);
//     RMotor->run(BACKWARD);
//     RMotor->setSpeed(120);
//     delay(10);
//   }
//   LMotor->setSpeed(0);
//   RMotor->setSpeed(0);

// }



