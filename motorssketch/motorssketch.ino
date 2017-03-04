#include <PID_v1.h>


#ifndef PSTR
#define PSTR // Make Arduino Due happy
#endif


//encoder pins (PINAx MUST be interrupts)
//NOTE: MOTOR1 is RIGHT, MOTOR2 is LEFT
#define PINA1 2
#define PINB1 4
#define PINA2 3
#define PINB2 5
//tapper pins
#define PINA3 20
#define PINB3 26
volatile byte PinA1Last;
volatile byte PinA2Last;
volatile byte PinA3Last;
volatile int duration1;
volatile int duration2;
volatile int duration3;
volatile boolean Direction1;
volatile boolean Direction2;
volatile boolean Direction3;
//motor control
#define MOT1_PIN1 40
#define MOT1_PIN2 42
#define MOT1_PWM 44
#define MOT2_PIN1 41
#define MOT2_PIN2 43
#define MOT2_PWM 45
//tapper control
#define MOT3_PIN1 50
#define MOT3_PIN2 48
#define MOT3_PWM 46
//motor modulus
int motorMod = 0;

// parsing inputs
String command; //used to store command from serial
String value; //used to store value from serial
String response; //used to store response to main program


void setup() {

    //start serial
    Serial.begin(115200);
    //start serial1 to motor controller
    //Serial1.begin(115200);
    //initialize encoders/motors
    EncoderInit();
    MotorInit();
    //Serial1.write("1f0\r");
    //delayMicroseconds(500);
    //Serial1.write("2f0\r");
    //send READY byte
    Serial.write('1');
}

void loop() {
    command = "";
    value = "";
    int addTo = 0; //0 for command, 1 for value
    //ButtonStates();
    if(Serial.available()){
        while (Serial.available() > 0)
        {
            char readIn = (char)Serial.read();
            if (readIn == '\n') {
                break;
            }
            else if (readIn == '|') {
                addTo = 1;
                continue;
            }
            //other stuff that is important
            if (addTo == 0) {
                command += readIn;
            }
            else if (addTo == 1) {
                value += readIn;
            }
        }
        //clear anything remaining in serial
        while (Serial.available() > 0) {
            Serial.read();
        }
        response = interpretCommand(command,value);
        Serial.println(response); //sends response with \n at the end
    }
    //small delay
    delay(20);

}

String interpretCommand(char command, String value) {

    switch(command){
        case "f":
            goForward();
        case "l":
            turnLeft();
        case "r":
            turnRight();
        case "t":
            performTap();
        case "c":
            calibrateWithIR(value);
    }



    //check if any BADs were obtained
    if (returnString == "BAD") {
        return "n";
    }
    return responseString;

}

//START OF MOTOR STUFF
void EncoderInit() {
    pinMode(PINB1,INPUT);
    pinMode(PINB2,INPUT);
    pinMode(PINB3,INPUT);
    attachInterrupt(digitalPinToInterrupt(PINA1),wheelSpeed1,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PINA2),wheelSpeed2,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PINA3),wheelSpeed3,CHANGE);
}

void MotorInit() {
    pinMode(MOT1_PIN1,OUTPUT);
    pinMode(MOT1_PIN2,OUTPUT);
    pinMode(MOT1_PWM,OUTPUT);
    pinMode(MOT2_PIN1,OUTPUT);
    pinMode(MOT2_PIN2,OUTPUT);
    pinMode(MOT2_PWM,OUTPUT);
    pinMode(MOT3_PIN1,OUTPUT);
    pinMode(MOT3_PIN2,OUTPUT);
    pinMode(MOT3_PWM,OUTPUT);
}

void SwitchInit() {
    pinMode(SWITCH_FR,INPUT);
    pinMode(SWITCH_FL,INPUT);
}


void changeDirection(int pwm1, int pwm2) {
    if (pwm1 >= 0) {
        digitalWrite(MOT1_PIN1,HIGH);
        digitalWrite(MOT1_PIN2,LOW);
    }
    else {
        digitalWrite(MOT1_PIN1,LOW);
        digitalWrite(MOT1_PIN2,HIGH);
    }
    //set direction for motor 2
    if (pwm2 >= 0) {
        digitalWrite(MOT2_PIN1,HIGH);
        digitalWrite(MOT2_PIN2,LOW);
    }
    else {
        digitalWrite(MOT2_PIN1,LOW);
        digitalWrite(MOT2_PIN2,HIGH);
    }
}

//ACTUAL implementation for direct motor controller
int runMotorsTill(int value1, int value2, int pwm1, int pwm2) {
    unsigned long lastGoCommand = millis();
    duration1 = 0;
    duration2 = 0;
    bool on1 = true;
    bool on2 = true;
    int slowDiff = 400;
    int slowPWM = 125;
    int slowestPWM = 90;
    //run motors
    //set direction for motor 1
    changeDirection(pwm1,pwm2);
    //set PWM for both with as little latency in between
    analogWrite(MOT1_PWM,abs(pwm1));
    digitalWrite(LED1,HIGH);
    analogWrite(MOT2_PWM,abs(pwm2));
    digitalWrite(LED2,HIGH);
    //do stuff while not done
    while (on1 || on2) {
        //if StopButton has been pressed, stop moving!
        if (StopState == '1') {
            digitalWrite(LED1,LOW);
            digitalWrite(LED2,LOW);
            break;
        }
        if (on1) {
            if (abs(duration1) >= value1) {
                analogWrite(MOT1_PWM,0);
                digitalWrite(LED1,LOW);
                on1 = false;
            }
            else if (abs(duration1) >= value1-slowDiff) {
                int actualPWM1 = map(abs(duration1),value1-slowDiff,value1,slowPWM,slowestPWM);
                analogWrite(MOT1_PWM,actualPWM1-5);
            }
        }
        if (on2) {
            if (abs(duration2) >= value2) {
                analogWrite(MOT2_PWM,0);
                digitalWrite(LED2,LOW);
                on2 = false;
            }
            else if (abs(duration2) >= value2-slowDiff) {
                int actualPWM2 = map(abs(duration2),value2-slowDiff,value2,slowPWM,slowestPWM);
                analogWrite(MOT2_PWM,actualPWM2);
            }
        }
    }
    //stop both motors now, promptly
    analogWrite(MOT1_PWM,0);
    analogWrite(MOT2_PWM,0);

    return 1;
}

//ACTUAL implementation for tapping
int runTappingTill(int value3, int pwm3) {
    duration3 = 0;
    bool on3 = true;
    int slowDiff = 400;
    int slowPWM = 255;
    int slowestPWM = 240;
    //set motor direction
    setTapperDirection(pwm3);
    //set PWM for motor to start it up
    analogWrite(MOT3_PWM,abs(pwm3));
    //do stuff while not done
    while (on3) {
        //if StopButton has been pressed, stop moving!
        if (StopState == '1') {
            break;
        }
        if (on3) {
            if (abs(duration3) >= value3) {
                analogWrite(MOT3_PWM,0);
                on3 = false;
            }
            else if (abs(duration3) >= value3-slowDiff) {
                int actualPWM3 = map(abs(duration3),value3-slowDiff,value3,slowPWM,slowestPWM);
                analogWrite(MOT3_PWM,actualPWM3-5);
            }
        }
    }
    //stop motor now!
    analogWrite(MOT3_PWM,0);

    return 1;
}

int expMovingAvg(int newVal,int oldVal, double prefVal) {
    return int(newVal*prefVal + oldVal*(1.0-prefVal));
}



String goForward() {
    //int actualDur = runMotorsTill(1500,1500,"1f9\r","2f9\r");
    int forwCount = 2512;
    int actualDur = runMotorsTill(forwCount-25,forwCount+25,251,255);
    return "1";
}

String performTap() {
    int encCount = 1725;
    int actualDur = runTappingTill(encCount,255);
    return "1";
}


String goBackward() {
    //int actualDur = runMotorsTill(1500,1500,"1f9\r","2f9\r");
    int backCount = 2512;
    int actualDur = runMotorsTill(backCount,backCount,-255,-255);
    return "1";
}

String turnLeft() {
    //int actualDur = runMotorsTill(1050,1050,"1f9\r","2r9\r");
    int actualDur = runMotorsTill(1202,1202,235,-235);
    return "1";
}

String turnRight() {
    //int actualDur = runMotorsTill(1100,1100,"1r9\r","2f9\r");
    int actualDur = runMotorsTill(1223,1223,-255,255);
    return "1";
}
//END OF MOTOR STUFF

//START OF SENSOR STUFF

//END OF SENSOR STUFF


