/*
Line Sensor Module Example Code V1
Move the line sensor across the black and white line, monitor on serial
*/
#include <Adafruit_MotorShield.h>
#include <iostream>
#include <vector>
#include <tuple>

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
        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
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
    std::vector<std::tuple<char, int>> my_list = {
    std::make_tuple('L', 1),
    std::make_tuple('R', 2),
    std::make_tuple('R', 2)
    };                                      // made a list for direction
    int n = 100;

    if (valButton == HIGH) { // check if the input is HIGH
        Serial.print("*******");
        rightMotor->setSpeed(125);
        leftMotor->setSpeed(125);
        rightMotor->run(FORWARD);
        leftMotor->run(FORWARD);
        n = 0;                              // made the button start the car then start going through the while loop for the turn
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
        while (my_list.size > n) {
            valLeft = digitalRead(leftlinesensorPin);
            valRight = digitalRead(rightlinesensorPin);
            lineFollow(valLeft, valRight);
            std::tuple<char, int> current_tuple = my_list[n];
            char current_char = std::get<0>(current_tuple);
            int current_val = std::get<1>(current_tuple);
            int break_val = 1;
            while (break_val != 0) {
                int valLeft = digitalRead(leftlinesensorPin); 
                int valRight = digitalRead(rightlinesensorPin); 
                int valLL = digitalRead(LLlinesensorPin);
                int valRR = digitalRead(RRlinesensorPin); 
                int valButton = digitalRead(buttonPin);
                if (current_val == 1 && valLL == 1) {
                    if (current_char == L) {
                        turnLeft;                                   //i think this works just fix the turn left and turn right?
                        n = n + 1;
                        break_val = 0;
                        
                    }
                    if (current_char == R) {
                        turnRight;
                        n = n + 1;
                        break_val = 1;
                    }

                }
                if (current_val != 1 && (valLL == 1 || valRR == 1)) {
                    current_val = current_val - 1;
                }
            }



            }
        }

    }
}




