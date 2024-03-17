#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <arduino.h>
Servo armServo; // create servo objects to control a servo
Servo stringServo;

// set pins for components
int leftlinesensorPin = 3;
int rightlinesensorPin = 2;
int LLlinesensorPin = 4;
int RRlinesensorPin = 5;
int buttonPin = 6;
int blueLEDPin = 7;
int greenLEDPin = 8;
int redLEDPin = 9;
int distanceSensorPin = A3;
int crashSensorPin = 12;

int IR = 13;  // Define pin for colour sensor

// set vairables used to control pathing
int turnCounter = 0;
int preventLeft = 0;
int preventRight = 0;

// set variables used to control blue flashing LED
int blueN = 4; // set at 4 to create a timeing offset
int blueM = 1; // set at 1 to avoid turning on LED when mod(0) = 0

// set variables used to control the grabber mechanism
int pos_arm = 0; // grabber arm servo position (calibrate when attaching arm)
int pos_string = 0; // string servo position

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor* leftMotor = AFMS.getMotor(2);

// set miscellaneous variables
int baycounter = 0;
int pickupcounter = 0;
int turnRcounter = 0;
int changelist = 0;
int movearmdown = 0;
int extraarmup = 15;
int pos_stringoffset = 0;
int lowarm = 20; //when attemting to pickup
int higharm = 90; // max lift
int midarm = 60; // dropping off
int tightstring = 190; //picking up block
int midstring = tightstring - higharm + lowarm; //keeping block as the arm goes up
int loosestring = 0; //loose with arm up

// define a function to compare two arrays
int compare(char list[], char path[]) {
int i = 0;
  while (list[i] != 'E'){
    if (list[i] == path[i]){
      i = i+1;
    }
    if (list[i] != path[i]){
      return 0;
    }
  }
  return 1;
}

// All the different paths that may the robot may be required to take
// R - Right, L - Left, S - Straight, U - Pickup block, D - Deposit block at RED, G - Deposit block at GREEN, F - Finish in box, E - End of array
char FBFS[] = {'R', 'U', 'E'};
char FBTR[] = {'L', 'S', 'R', 'D', 'E'};
char FBTG[] = {'R', 'L', 'G', 'E'};
char SBFR[] = {'S', 'L', 'L', 'U', 'E'};
char SBFG[] = {'S', 'R', 'S', 'R', 'U', 'E'};
char SBTR[] = {'R', 'R', 'S', 'D', 'E'};
char SBTG[] = {'L', 'S', 'L', 'S', 'G', 'E'};
char TBFR[] = {'S', 'S', 'L', 'U', 'E'};
char TBFG[] = {'S', 'S', 'S', 'R', 'E'};
char TBTR[] = {'R', 'S', 'S', 'D', 'E'};
char TBTG[] = {'L', 'S', 'S', 'S', 'G','E'};
char FFBFR[] = {'S', 'S', 'S', 'L', 'R', 'U', 'E'};
char FFBFG[] = {'S', 'S', 'R', 'R', 'U', 'E'};
char FFBTR[] = {'L', 'R', 'S', 'S', 'S', 'D', 'E'};
char FFBTG[] = {'L', 'L', 'S', 'S', 'G', 'E'};
char EFR[] = {'L', 'L', 'S', 'F', 'E'};            
char EFG[] = {'R', 'S', 'R', 'S', 'F', 'E'};        

// A copy of the paths so we are able to compare arrays
char ZFBFS[] = {'R', 'U', 'E'};  
char ZFBTR[] = {'L', 'S', 'R', 'D', 'E'};
char ZFBTG[] = {'R', 'L', 'G', 'E'};
char ZSBFR[] = {'S', 'L', 'L', 'U', 'E'};
char ZSBFG[] = {'S', 'R', 'S', 'R', 'U', 'E'};
char ZSBTR[] = {'R', 'R', 'S', 'D', 'E'};
char ZSBTG[] = {'L', 'S', 'L', 'S', 'G', 'E'};
char ZTBFR[] = {'S', 'S', 'L', 'U', 'E'};
char ZTBFG[] = {'S', 'S', 'S', 'R', 'E'};
char ZTBTR[] = {'R', 'S', 'S', 'D', 'E'};
char ZTBTG[] = {'L', 'S', 'S', 'S', 'G','E'};
char ZFFBFR[] = {'S', 'S', 'S', 'L', 'R', 'U', 'E'};
char ZFFBFG[] = {'S', 'S', 'R', 'R', 'U', 'E'};
char ZFFBTR[] = {'L', 'R', 'S', 'S', 'S', 'D', 'E'};
char ZFFBTG[] = {'L', 'L', 'S', 'S', 'G', 'E'};
char ZEFR[] = {'L', 'L', 'S', 'F', 'E'};            
char ZEFG[] = {'R', 'S', 'R', 'S', 'F', 'E'};  

// Sets first route (First Block From Start)
char *turn_List = ZFBFS;

// Line following
void lineFollow(int valLeft, int valRight) {
    if (valLeft == 1 && valRight == 1) {
        //Serial.println("STRAIGHT");
        rightMotor->setSpeed(250);
        leftMotor->setSpeed(255); // Counteract turning tendancy
        rightMotor->run(FORWARD);
        leftMotor->run(FORWARD);
    }
    if (valLeft == 0 && valRight == 1) {
        //Serial.println("TURN LEFT");
        rightMotor->setSpeed(255);
        leftMotor->setSpeed(100);
        rightMotor->run(FORWARD);
        leftMotor ->run(FORWARD);
    }
    if (valLeft == 1 && valRight == 0) {
        //Serial.println("TURN RIGHT");
        rightMotor->setSpeed(100);
        leftMotor->setSpeed(255);
        leftMotor->run(FORWARD);
        rightMotor ->run(FORWARD);
    }
    if (valLeft == 0 && valRight == 0) {
        //Serial.println("AHHH");
        rightMotor->run(FORWARD);
        leftMotor->run(FORWARD);
    }
}

// Go straight function, used at the start
void goStraight() {
    rightMotor->setSpeed(210);
    leftMotor->setSpeed(230);
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);
}

// Stop function, used when picking up / depositing
void Stop() {
    rightMotor->setSpeed(0);
    leftMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);
}

// Turn right
void turnRight() {
    rightMotor->setSpeed(220);
    leftMotor->setSpeed(100);
    rightMotor->run(FORWARD);
    leftMotor->run(BACKWARD);
}

// Turn left
void turnLeft() {
    rightMotor->setSpeed(100);
    leftMotor->setSpeed(220);
    rightMotor->run(BACKWARD);
    leftMotor->run(FORWARD);
}

// Do a 180 spin to the right
void spinRight() {
  rightMotor->setSpeed(185);
  leftMotor->setSpeed(255);
  rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
}

// Do a 180 to the left
void spinLeft() {
  rightMotor->setSpeed(250);
  leftMotor->setSpeed(190);
  rightMotor->run(BACKWARD);
  leftMotor->run(FORWARD);
}

// Reverse function
void Reverse(){
  rightMotor->setSpeed(190);
  leftMotor->setSpeed(190);
  rightMotor->run(BACKWARD);
  leftMotor->run(BACKWARD);
}

// A faster spin to the right, used after picking up
void spinMoreRight(){
  leftMotor->setSpeed(200);
  rightMotor->setSpeed(190);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}

// Pickup block function
void blockPickup() {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);

    //Lower arm
    Serial.print("Lowering arm");
    for (pos_arm = higharm; pos_arm >= lowarm; pos_arm -= 1) {
        armServo.write(pos_arm);
        delay(45); // waits for the servo to reach the position
    }

    if (digitalRead(IR) == 1){
        digitalWrite(redLEDPin, HIGH);
        red = 1;
        }
    if (digitalRead(IR) == 0){
        digitalWrite(greenLEDPin, HIGH);
        green = 1;
        }

    //String tighten
    Serial.print("String tighten");
    for (pos_string = 0; pos_string <= tightstring; pos_string += 1) {
        stringServo.write(pos_string + pos_stringoffset);
        delay(30);
    }

    //Raise arm
    Serial.print("Raising arm");
    for (pos_arm = lowarm; pos_arm <= higharm + extraarmup; pos_arm += 1) { // goes from 0 degrees to 90 + extraarmup
        // in steps of 1 degree
        armServo.write(pos_arm); // tell servo to go to position in variable 'pos'
        if (tightstring - pos_arm + lowarm > midstring) {
            stringServo.write(tightstring - pos_arm + lowarm + pos_stringoffset);
        }
        delay(45); // waits for the servo to reach the position
    }
    delay(5000);
}

//Deposit the block
void blockDeposit() {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);

    //Arm down
    for (pos_arm = higharm + extraarmup; pos_arm >= midarm; pos_arm -= 1) {
        stringServo.write(midstring - pos_arm + higharm + pos_stringoffset);
        armServo.write(pos_arm);
        Serial.print(pos_arm);
        delay(45); // waits for the servo to reach the position
        movearmdown = 1;
    }
  
    //String loosen
    Serial.print("string release");
    for (pos_string = (midstring - midarm + higharm); pos_string >= (higharm - midarm); pos_string -= 1) {
        Serial.print(pos_string);
        stringServo.write(pos_string + pos_stringoffset);
        delay(30);
    }
    //Arm up
    Serial.print("Raising arm");
    for (pos_arm = midarm; pos_arm <= higharm; pos_arm += 1) { // goes from 0 degrees to 90
        // in steps of 1 degree
        armServo.write(pos_arm); // tell servo to go to position in variable 'pos'
        stringServo.write(higharm - midarm + pos_stringoffset - pos_arm + midarm);
        Serial.print(pos_arm);
        delay(45); // waits for the servo to reach the position
    }
    // Add string offset when arm is up
    for (pos_string = pos_stringoffset; pos_string <= pos_stringoffset + 20; pos_string += 1) {
        Serial.print(pos_string);
        stringServo.write(pos_string + pos_stringoffset);
        delay(30);
    }
    pos_stringoffset = pos_stringoffset + 20;
    delay(1000);
}

void setup() {
    Serial.begin(9600); // Init the serial port
    pinMode(leftlinesensorPin, INPUT);
    pinMode(rightlinesensorPin, INPUT);
    pinMode(buttonPin, INPUT); // declare pushbutton as input
    pinMode(blueLEDPin, OUTPUT); // declare blue LED as output
    pinMode(redLEDPin, OUTPUT); // declare red LED as output
    pinMode(greenLEDPin, OUTPUT); // declare green LED as output
    armServo.attach(11); // attaches the raising servo on pin 11 to the servo object
    stringServo.attach(10); //attaches the tightening servo on pin 10
    pinMode(IR, INPUT); // colour sensor

    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield found.");
    // Set the speed to start, from 0 (off) to 255 (max speed)
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    rightMotor->run(RELEASE);
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    leftMotor->run(RELEASE);
    armServo.write(90);
    stringServo.write(0);
}

void loop() {
  Serial.print(turn_List);
  while (turn_List[turnCounter] != 'E') {
    int valLeft = digitalRead(leftlinesensorPin); // read left input value
    int valRight = digitalRead(rightlinesensorPin); // read right input value
    int valLL = digitalRead(LLlinesensorPin); // Wide left sensor output
    int valRR = digitalRead(RRlinesensorPin); // Wide right sensor output
    int valButton = digitalRead(buttonPin); // read input value
    int distanceSensor = analogRead(distanceSensorPin);
    float Distance = (520/1023.0)*distanceSensor;
    int crashSensor = digitalRead(crashSensorPin);
    char nextTurn = turn_List[turnCounter];

    // Flashing blue LED
    if (blueN%5 == 0){
      digitalWrite(blueLEDPin, HIGH);
      blueN = 1;
    }
    if (blueM%5 == 0){
      digitalWrite(blueLEDPin, LOW);
      blueM = 1;
    }
    blueN = blueN + 1;
    blueM = blueM + 1;

    // Movement
    Serial.print("NextTurn:");
    Serial.print(nextTurn);
    Serial.print("   Prevents:");
    Serial.print(preventLeft);
    Serial.print(preventRight);
    Serial.print("   TurnCounter:");
    Serial.print(turnCounter);
    Serial.print("  Distance:");
    Serial.print(Distance,0);
    Serial.println("                                                ");
    if (nextTurn == 'R'){
      preventLeft = 1;
      preventRight = 0;
    }

    if (nextTurn == 'L'){
      preventLeft = 0;
      preventRight = 1;
    }

    if (nextTurn == 'S'){
      preventLeft = 1;
      preventRight = 1;
    }

    if (valButton == HIGH) { // check if the input is HIGH
      Serial.print("*******");
      leftMotor->setSpeed(145);
      rightMotor->setSpeed(120);
    }
    else {
      lineFollow(valLeft, valRight);

      // Full left turn
      if ((valLL == 1) && (preventLeft == 0)) {
        Serial.println("Left90");
        delay(100);
        while ((valLeft == 1) && (valRight == 1)){
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            turnLeft();
          }
        while (valLeft == 0 || valRight == 0){
          turnLeft();
          Serial.print("L turning");
          valLeft = digitalRead(leftlinesensorPin);
          valRight = digitalRead(rightlinesensorPin);
          Serial.print(valLeft);
          Serial.print(valRight);
        }
        turnCounter = turnCounter + 1;
      }

      //Full right turn
      if ((valRR == 1) && (preventRight == 0)) {
          Serial.println("Right90");
          while (turnRcounter < 50){       // change this value to change right turn alignment
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            lineFollow(valLeft, valRight);
            turnRcounter = turnRcounter + 1;
        }
        turnRcounter = 0;
          while ((valLeft == 1) && (valRight == 1)){
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            turnRight();
          } 
          while (valLeft == 0 || valRight == 0){
            turnRight();
            Serial.print("R turning");
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            Serial.print(valLeft);
            Serial.print(valRight);
          }
        turnCounter = turnCounter + 1;
      }
     
      // Straight
      if ((preventLeft == 1) && (preventRight == 1)){
        if ((valLL == 1) || (valRR == 1)){
          turnCounter = turnCounter + 1;
          Serial.print("STRAIGHT ON");
          while ((valLL == 1) || (valRR == 1)){
            valLL = digitalRead(LLlinesensorPin);
            valRR = digitalRead(RRlinesensorPin);
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            lineFollow(valLeft, valRight);
          }
        }
       
      }

      // Pickup
      if (nextTurn == 'U' && Distance < 7) {
        Serial.println("Pickup");
        while (pickupcounter < 45){       // change this value to change distance from block when picking up
          valLeft = digitalRead(leftlinesensorPin);
          valRight = digitalRead(rightlinesensorPin);
          lineFollow(valLeft, valRight);
          pickupcounter = pickupcounter + 1;
        }
        blockPickup();
        Serial.println("BLOCK PICKED UP");
        while (baycounter < 200){   // determines how long it drives forward before spinning around
          valLeft = digitalRead(leftlinesensorPin);
          valRight = digitalRead(rightlinesensorPin);
          lineFollow(valLeft, valRight);
          baycounter = baycounter + 1;
        }
        while ((valLeft == 1) || (valRight == 1)){
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            spinMoreRight();
          }  
        while (valLeft == 0 || valRight == 0){
          spinMoreRight();
          Serial.print("Spinning");
          valLeft = digitalRead(leftlinesensorPin);
          valRight = digitalRead(rightlinesensorPin);
          Serial.print(valLeft);
          Serial.print(valRight);
        }
        turnCounter = turnCounter + 1;
      }

      // Deposit at RED (Colour determines the spin direction)
      if (nextTurn == 'D' && crashSensor == LOW){
        blockDeposit();
        Serial.println("BLOCK DEPOSITED");
        delay(1000);
        while ((valLeft == 1) || (valRight == 1)){
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            spinRight();
          }
        while (valLeft == 0 || valRight == 0){
          spinRight();
          Serial.print("Spinning");
          valLeft = digitalRead(leftlinesensorPin);
          valRight = digitalRead(rightlinesensorPin);
          Serial.print(valLeft);
          Serial.print(valRight);
          }
        turnCounter = turnCounter + 1;
      }

      // Deposit block at GREEN (colour determines spin direction)
      if (nextTurn == 'G' && crashSensor == LOW){
        blockDeposit();
        Serial.println("BLOCK DEPOSITED");
        delay(1000);
        while ((valLeft == 1) || (valRight == 1)){
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            spinLeft();
          }
        while (valLeft == 0 || valRight == 0){
          spinLeft();
          Serial.print("Spinning");
          valLeft = digitalRead(leftlinesensorPin);
          valRight = digitalRead(rightlinesensorPin);
          Serial.print(valLeft);
          Serial.print(valRight);
          }
        turnCounter = turnCounter + 1;
      }
     
      // Stopinbox turn
      while ((nextTurn == 'F') && (Distance < 10)){
          Stop();
          digitalWrite(blueLEDPin, LOW);
        }
    }
  }
 
// Pathing
 while (changelist != 1){
    if ((compare(turn_List, FBFS) == 1) && (digitalRead(IR) == 1)){
      turn_List = ZFBTR;
      Serial.println("Colour: RED");
      break;
    }
    if ((compare(turn_List, FBFS) == 1) && (digitalRead(IR) == 0)){
      turn_List = ZFBTG;
      Serial.println("Colour: BLACK ");
      break;
    }
    if (compare(turn_List, FBTR) == 1){
      turn_List = ZSBFR;
      break;
    }
    if (compare(turn_List, FBTG) == 1){
      turn_List = ZSBFG;
      break;
    }
    if (((compare(turn_List, SBFR) == 1)||(compare(turn_List, SBFG) == 1)) && (digitalRead(IR) == 1)){
      turn_List = ZSBTR;
      Serial.println("Colour: RED");
      break;
    }
    if (((compare(turn_List, SBFR) == 1)||(compare(turn_List, SBFG) == 1)) && (digitalRead(IR) == 0)){
      turn_List = ZSBTG;
      Serial.println("Colour: BLACK ");
      break;
    }
    if (compare(turn_List, SBTR) == 1){
      turn_List = ZTBFR;
      break;
    }
    if (compare(turn_List, SBTG) == 1){
      turn_List = ZTBFG;
      break;
    }
    if (((compare(turn_List, TBFR) == 1)||(compare(turn_List, TBFG) == 1)) && (digitalRead(IR) == 1)){
      turn_List = ZTBTR;
      Serial.println("Colour: RED");
      break;
    }
    if (((compare(turn_List, TBFR)== 1)||(compare(turn_List, TBFG) == 1)) && (digitalRead(IR) == 0)){
      turn_List = ZTBTG;
      Serial.println("Colour: BLACK ");
      break;
    }
    if (compare(turn_List, TBTR) == 1){
      turn_List = ZFFBFR;
      break;
    }
    if (compare(turn_List, TBTG) == 1){
      turn_List = ZFFBFG;
      break;
    }
    if (((compare(turn_List, FFBFR)== 1)||(compare(turn_List, FFBFG)== 1)) && (digitalRead(IR) == 1)){
      turn_List = ZFFBTR;
      Serial.println("Colour: RED");
      break;
    }
    if (((compare(turn_List, FFBFR) == 1)||(compare(turn_List, FFBFG) == 1)) && (digitalRead(IR) == 0)){
      turn_List = ZFFBTG;
      Serial.println("Colour: BLACK ");
      break;
    }
    if (compare(turn_List, FFBTR) == 1){
      turn_List = ZEFR;
      break;
    }
    if (compare(turn_List, FFBTG) == 1){
      turn_List = ZEFG;
      break;
    }
  }


  turnCounter = 0;
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  pickupcounter = 0;
  movearmdown = 0;
  baycounter = 0;
}
