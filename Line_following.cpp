/*
Line Sensor Module Example Code V1
Move the line sensor across the black and white line, monitor on serial
*/
#include <Adafruit_MotorShield.h>


int leftlinesensorPin = 3;
int rightlinesensorPin = 2;
int LLlinesensorPin = 4;
int RRlinesensorPin = 5;
int buttonPin = 6;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor* leftMotor = AFMS.getMotor(2);


void lineFollow(int valLeft, int valRight) {
    if (valLeft == 1 && valRight == 1) {
        Serial.println("STRAIGHT");
        rightMotor->setSpeed(250);
        leftMotor->setSpeed(290); // Counteract turning tendancy
        rightMotor->run(FORWARD);
        leftMotor->run(FORWARD);
    }
    if (valLeft == 0 && valRight == 1) {
        Serial.println("TURN LEFT");
        rightMotor->setSpeed(125);
        leftMotor->setSpeed(0);
        rightMotor->run(FORWARD);
    }
    if (valLeft == 1 && valRight == 0) {
        Serial.println("TURN RIGHT");
        rightMotor->setSpeed(0);
        leftMotor->setSpeed(125);
        leftMotor->run(FORWARD);
    }
    if (valLeft == 0 && valRight == 0) {
        Serial.println("AHHH");
        rightMotor->run(FORWARD);
        leftMotor->run(FORWARD);
    }
}


void turnLeft() {
    rightMotor->setSpeed(125);
    leftMotor->setSpeed(75);
    rightMotor->run(FORWARD);
    leftMotor->run(BACKWARD);
}

void turnRight() {
    rightMotor->setSpeed(75);
    leftMotor->setSpeed(125);
    rightMotor->run(BACKWARD);
    leftMotor->run(FORWARD);
}


void setup() {
    Serial.begin(9600); // Init the serial port
    pinMode(leftlinesensorPin, INPUT);
    pinMode(rightlinesensorPin, INPUT);
    pinMode(buttonPin, INPUT); // declare pushbutton as input

    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        /// if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield found.");


    // Set the speed to start, from 0 (off) to 255 (max speed)
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    // turn on motor
    rightMotor->run(RELEASE);


    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    // turn on motor
    leftMotor->run(RELEASE);
}

void loop() {
    int valLeft = digitalRead(leftlinesensorPin); // read left input value
    int valRight = digitalRead(rightlinesensorPin); // read right input value
    int valLL = digitalRead(LLlinesensorPin); // Wide left sensor output
    int valRR = digitalRead(RRlinesensorPin); // Wide right sensor output
    int valButton = digitalRead(buttonPin); // read input value


    Serial.print(valLeft);
    Serial.println(valRight);


    if (valButton == HIGH) { // check if the input is HIGH
        Serial.print("*******");
    }
    else {
        Serial.print("-------");

        delay(100);


        lineFollow(valLeft, valRight);

        if (valLL == 1) {
            Serial.println("Left90");
            turnLeft();
            delay(500);
            while (valLeft == 0 || valRight == 0) {
                turnLeft();
                Serial.print("L turning");
                valLeft = digitalRead(leftlinesensorPin);
                valRight = digitalRead(rightlinesensorPin);
                Serial.print(valLeft);
                Serial.print(valRight);
            }
        }
        if (valRR == 1) {
            Serial.println("Right90");
            turnRight();
            delay(500);
            while (valLeft == 0 || valRight == 0) {
                turnRight();
                Serial.print("R turning");
                valLeft = digitalRead(leftlinesensorPin);
                valRight = digitalRead(rightlinesensorPin);
                Serial.print(valLeft);
                Serial.print(valRight);
            }
        }
    }
}